#include "MeshBuilder.h"
#include "MeshBuilder.private.h"
#include "FbxParser.private.h"
#include <algorithm>

namespace Fbx { namespace Builder {

SubMeshVertex::SubMeshVertex(uint32_t maxTexCoord) { uvs.resize(maxTexCoord); }

MikkTSpace::MikkTSpace(const std::vector<Importer::MeshImportData::MeshWedge>& wedges, const std::vector<Importer::MeshImportData::MeshFace>& faces, const std::vector<glm::vec3>& Points, bool bInComputeNormals, std::vector<glm::vec3>& vertexTangentsX, std::vector<glm::vec3>& vertexTangentsY,
                       std::vector<glm::vec3>& vertexTangentsZ)
    : wedges(wedges), faces(faces), points(Points), bComputeNormals(bInComputeNormals), tangentsX(vertexTangentsX), tangentsY(vertexTangentsY), tangentsZ(vertexTangentsZ) {}

int MikkGetNumFaces(const SMikkTSpaceContext* context) {
    MikkTSpace* userData = (MikkTSpace*)(context->m_pUserData);
    return static_cast<int>(userData->faces.size());
}

int MikkGetNumVertsOfFace(const SMikkTSpaceContext* context, const int faceIdx) {
    // Confirmed?
    return 3;
}

void MikkGetPosition(const SMikkTSpaceContext* context, float position[3], const int faceIdx, const int vertIdx) {
    MikkTSpace* userData = (MikkTSpace*)(context->m_pUserData);
    const glm::vec3& vertexPosition = userData->points[userData->wedges[userData->faces[faceIdx].iWedge[vertIdx]].iVertex];
    position[0] = vertexPosition.x;
    position[1] = vertexPosition.y;
    position[2] = vertexPosition.z;
}

void MikkGetNormal(const SMikkTSpaceContext* context, float normal[3], const int faceIdx, const int vertIdx) {
    MikkTSpace* userData = (MikkTSpace*)(context->m_pUserData);
    // Get different normals depending on whether they've been calculated or not.
    if (userData->bComputeNormals) {
        glm::vec3& vertexNormal = userData->tangentsZ[faceIdx * 3 + vertIdx];
        normal[0] = vertexNormal.x;
        normal[1] = vertexNormal.y;
        normal[2] = vertexNormal.z;
    } else {
        const glm::vec3& vertexNormal = userData->faces[faceIdx].tangentZ[vertIdx];
        normal[0] = vertexNormal.x;
        normal[1] = vertexNormal.y;
        normal[2] = vertexNormal.z;
    }
}

void MikkSetTSpaceBasic(const SMikkTSpaceContext* context, const float tangent[3], const float bitangentSign, const int faceIdx, const int vertIdx) {
    MikkTSpace* userData = (MikkTSpace*)(context->m_pUserData);
    glm::vec3& vertexTangent = userData->tangentsX[faceIdx * 3 + vertIdx];
    vertexTangent.x = bitangentSign * tangent[0];
    vertexTangent.y = bitangentSign * tangent[1];
    vertexTangent.z = bitangentSign * tangent[2];

    glm::vec3 bitangent;
    // Get different normals depending on whether they've been calculated or not.
    if (userData->bComputeNormals) {
        bitangent = glm::cross(userData->tangentsZ[faceIdx * 3 + vertIdx], vertexTangent);
    } else {
        bitangent = glm::cross(userData->faces[faceIdx].tangentZ[vertIdx], vertexTangent);
    }
    glm::vec3& vertexBitangent = userData->tangentsY[faceIdx * 3 + vertIdx];
    vertexBitangent.x = bitangent[0];
    vertexBitangent.y = bitangent[1];
    vertexBitangent.z = bitangent[2];
}

void MikkGetTexCoord(const SMikkTSpaceContext* context, float UV[2], const int faceIdx, const int vertIdx) {
    MikkTSpace* userData = (MikkTSpace*)(context->m_pUserData);
    const glm::dvec2& texCoord = userData->wedges[userData->faces[faceIdx].iWedge[vertIdx]].uvs[0];
    UV[0] = static_cast<float>(texCoord.x);
    UV[1] = static_cast<float>(texCoord.y);
}

OverlappingCorners::OverlappingCorners(const std::vector<glm::vec3>& inVertices, const std::vector<uint32_t>& inIndices, float comparisonThreshold) {
    const int numWedges = static_cast<int>(inIndices.size());

    // Create a list of vertex Z/index pairs
    std::vector<IndexAndZ> vertIndexAndZ;
    vertIndexAndZ.resize(numWedges);
    for (int wedgeIndex = 0; wedgeIndex < numWedges; wedgeIndex++) {
        vertIndexAndZ[wedgeIndex] = IndexAndZ(wedgeIndex, inVertices[inIndices[wedgeIndex]]);
    }

    // Sort the vertices by z value
    std::sort(vertIndexAndZ.begin(), vertIndexAndZ.end(), CompareIndexAndZ());

    Init(numWedges);

    // Search for duplicates, quickly!
    for (int i = 0; i < vertIndexAndZ.size(); i++) {
        // only need to search forward, since we add pairs both ways
        for (int j = i + 1; j < vertIndexAndZ.size(); j++) {
            if (Maths::Abs(vertIndexAndZ[j].z - vertIndexAndZ[i].z) > comparisonThreshold) break;  // can't be any more dups

            const glm::vec3& positionA = inVertices[inIndices[vertIndexAndZ[i].index]];
            const glm::vec3& positionB = inVertices[inIndices[vertIndexAndZ[j].index]];

            if (PointsEqual(positionA, positionB, comparisonThreshold)) {
                Add(vertIndexAndZ[i].index, vertIndexAndZ[j].index);
            }
        }
    }

    FinishAdding();
}

void OverlappingCorners::Init(int numIndices) {
    arrays.clear();
    sets.clear();
    bFinishedAdding = false;

    indexBelongsTo.resize(numIndices, -1);
}

void OverlappingCorners::Add(int key, int value) {
    ASSERT(key != value);
    ASSERT(bFinishedAdding == false);

    int containerIndex = indexBelongsTo[key];
    if (containerIndex == -1) {
        containerIndex = static_cast<int>(arrays.size());
        arrays.push_back(std::move(std::vector<int>()));
        std::vector<int>& Container = arrays.back();
        Container.push_back(key);
        Container.push_back(value);
        indexBelongsTo[key] = containerIndex;
        indexBelongsTo[value] = containerIndex;
    } else {
        indexBelongsTo[value] = containerIndex;

        std::vector<int>& arrayContainer = arrays[containerIndex];
        if (arrayContainer.size() == 1) {
            // Container is a set
            sets[arrayContainer.back()].insert(value);
        } else {
            // Container is an array
            if (std::find(arrayContainer.begin(), arrayContainer.end(), value) == arrayContainer.end()) {
                arrayContainer.push_back(value);
            }

            // Change container into set when one vertex is shared by large number of triangles
            if (arrayContainer.size() > 12) {
                int setIndex = static_cast<int>(sets.size());
                sets.push_back(std::move(std::set<int>()));
                std::set<int>& Set = sets.back();
                for (const auto e : arrayContainer) {
                    Set.insert(e);
                }

                // Having one element means we are using a set
                // An array will never have just 1 element normally because we add them as pairs
                arrayContainer.clear();
                arrayContainer.push_back(setIndex);
            }
        }
    }
}

void OverlappingCorners::FinishAdding() {
    ASSERT(bFinishedAdding == false);

    for (std::vector<int>& array : arrays) {
        // Turn sets back into arrays for easier iteration code
        // Also reduces peak memory later in the import process
        if (array.size() == 1) {
            std::set<int>& Set = sets[array.back()];
            array.clear();
            for (int i : Set) {
                array.push_back(i);
            }
        }

        // Sort arrays now to avoid sort multiple times
        std::sort(array.begin(), array.end());
    }

    sets.clear();

    bFinishedAdding = true;
}

const std::vector<int>& OverlappingCorners::FindIfOverlapping(int key) const {
    ASSERT(bFinishedAdding);
    int containerIndex = indexBelongsTo[key];
    return (containerIndex != -1) ? arrays[containerIndex] : emptyArray;
}

bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, float comparisonThreshold) {
    if (Maths::Abs(v1.x - v2.x) > comparisonThreshold || Maths::Abs(v1.y - v2.y) > comparisonThreshold || Maths::Abs(v1.z - v2.z) > comparisonThreshold) {
        return false;
    }
    return true;
}

bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, bool bUseEpsilonCompare) {
    const float epsilon = bUseEpsilonCompare ? THRESH_POINTS_ARE_SAME : 0.0f;
    return Maths::Abs(v1.x - v2.x) <= epsilon && Maths::Abs(v1.y - v2.y) <= epsilon && Maths::Abs(v1.z - v2.z) <= epsilon;
}

bool UVsEqual(const glm::dvec2& v1, const glm::dvec2& v2, float epsilon = 1.0f / 1024.0f) { return Maths::Abs(v1.x - v2.x) <= epsilon && Maths::Abs(v1.y - v2.y) <= epsilon; }

bool NormalsEqual(const glm::vec3& v1, const glm::vec3& v2) {
    const float epsilon = THRESH_NORMALS_ARE_SAME;
    return Maths::Abs(v1.x - v2.x) <= epsilon && Maths::Abs(v1.y - v2.y) <= epsilon && Maths::Abs(v1.z - v2.z) <= epsilon;
}

glm::vec3 GetSafeNormal(const glm::vec3& vec, const float tolerance) {
    const float squareSum = glm::length2(vec);

    // Not sure if it's safe to add tolerance in there. Might introduce too many errors
    if (squareSum == 1.f) {
        return glm::vec3(vec);
    } else if (squareSum < tolerance) {
        return glm::vec3(0);
    }
    const float scale = static_cast<float>(1.0 / sqrt(squareSum));
    return vec * scale;
}

// Magic numbers for numerical precision.
#define DELTA (0.00001f)

void CreateOrthonormalBasis(glm::vec3& xAxis, glm::vec3& yAxis, glm::vec3& zAxis) {
    // Project the X and Y axes onto the plane perpendicular to the Z axis.
    xAxis -= glm::dot(xAxis, zAxis) / glm::dot(zAxis, zAxis) * zAxis;
    yAxis -= glm::dot(yAxis, zAxis) / glm::dot(zAxis, zAxis) * zAxis;

    // If the X axis was parallel to the Z axis, choose a vector which is orthogonal to the Y and Z axes.
    if (glm::length2(xAxis) < DELTA * DELTA) {
        xAxis = glm::cross(yAxis, zAxis);
    }

    // If the Y axis was parallel to the Z axis, choose a vector which is orthogonal to the X and Z axes.
    if (glm::length2(yAxis) < DELTA * DELTA) {
        yAxis = glm::cross(zAxis, xAxis);
    }

    // Normalize the basis vectors.
    xAxis = glm::normalize(xAxis);
    yAxis = glm::normalize(yAxis);
    zAxis = glm::normalize(zAxis);
}

void ForsythHelper::CacheOptimizeIndexBuffer(std::vector<uint32_t>& indices) {
    // Count the number of vertices
    uint32_t numVertices = 0;
    for (uint32_t index = 0; index < indices.size(); index++) {
        if (indices[index] > numVertices) {
            numVertices = indices[index];
        }
    }
    numVertices += 1;

    std::vector<uint32_t> optimizedIndices(indices.size(), 0);
    uint32_t CacheSize = 32;
    Forsyth::OptimizeFaces(indices.data(), static_cast<uint32_t>(indices.size()), numVertices, optimizedIndices.data(), CacheSize);

    for (int i = 0; i < optimizedIndices.size(); i++) {
        indices[i] = optimizedIndices[i];
    }
}

void CacheOptimizeIndexBuffer(std::vector<uint32_t>& indices) { ForsythHelper::CacheOptimizeIndexBuffer(indices); }

namespace TriangleUtilities {

float ComputeTriangleArea(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC) { return glm::length(glm::cross((pointB - pointA), (pointC - pointA))) / 2.0f; }

float ComputeTriangleCornerAngle(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC) {
    glm::vec3 e1 = (pointB - pointA);
    glm::vec3 e2 = (pointC - pointA);
    // Normalize both edges (unit vector) of the triangle so we get a dotProduct result that will be a valid acos input
    // [-1, 1]
    e1 = glm::normalize(e1);
    e2 = glm::normalize(e2);
    if (!(glm::length2(e1) > SMALL_NUMBER) || !(glm::length2(e2) > SMALL_NUMBER)) {
        // Return a null ratio if the polygon is degenerate
        return 0.0f;
    }
    float dotProduct = glm::dot(e1, e2);
    return Maths::Acos(dotProduct);
}
}  // namespace TriangleUtilities

bool MeshBuildSettings::operator==(const MeshBuildSettings& Other) const {
    return bRecomputeNormals == Other.bRecomputeNormals && bRecomputeTangents == Other.bRecomputeTangents && bUseMikkTSpace == Other.bUseMikkTSpace && bComputeWeightedNormals == Other.bComputeWeightedNormals && bRemoveDegenerates == Other.bRemoveDegenerates &&
           bBuildAdjacencyBuffer == Other.bBuildAdjacencyBuffer && thresholdPosition == Other.thresholdPosition && thresholdTangentNormal == Other.thresholdTangentNormal && thresholdUV == Other.thresholdUV && morphThresholdPosition == Other.morphThresholdPosition;
}

bool MeshBuildSettings::operator!=(const MeshBuildSettings& Other) const { return !(*this == Other); }

bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, const MeshBuildSettings& overlappingThreshold) {
    const float Epsilon = overlappingThreshold.thresholdPosition;
    return Maths::Abs(v1.x - v2.x) <= Epsilon && Maths::Abs(v1.y - v2.y) <= Epsilon && Maths::Abs(v1.z - v2.z) <= Epsilon;
}

bool UVsEqual(const glm::dvec2& v1, const glm::dvec2& v2, const MeshBuildSettings& overlappingThreshold) {
    const float epsilon = overlappingThreshold.thresholdUV;
    return Maths::Abs(v1.x - v2.x) <= epsilon && Maths::Abs(v1.y - v2.y) <= epsilon;
}

bool NormalsEqual(const glm::vec3& v1, const glm::vec3& v2, const MeshBuildSettings& overlappingThreshold) {
    const float epsilon = overlappingThreshold.thresholdTangentNormal;
    return Maths::Abs(v1.x - v2.x) <= epsilon && Maths::Abs(v1.y - v2.y) <= epsilon && Maths::Abs(v1.z - v2.z) <= epsilon;
}

IMeshBuildInputData::IMeshBuildInputData(const std::vector<Importer::MeshImportData::MeshWedge>& inWedges, const std::vector<Importer::MeshImportData::MeshFace>& inFaces, const std::vector<glm::vec3>& inPoints, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences,
                                         const std::vector<int>& inPointToOriginalMap, const std::shared_ptr<Importer::SceneInfo>& inSceneInfo, const MeshBuildSettings& inBuildOptions)
    : mikkTUserData(inWedges, inFaces, inPoints, inBuildOptions.bRecomputeNormals, tangentX, tangentY, tangentZ), wedges(inWedges), faces(inFaces), points(inPoints), influences(inInfluences), pointToOriginalMap(inPointToOriginalMap), sceneInfo(inSceneInfo), buildOptions(inBuildOptions) {
    mikkTInterface.m_getNormal = MikkGetNormal;
    mikkTInterface.m_getNumFaces = MikkGetNumFaces;
    mikkTInterface.m_getNumVerticesOfFace = MikkGetNumVertsOfFace;
    mikkTInterface.m_getPosition = MikkGetPosition;
    mikkTInterface.m_getTexCoord = MikkGetTexCoord;
    mikkTInterface.m_setTSpaceBasic = MikkSetTSpaceBasic;
    mikkTInterface.m_setTSpace = nullptr;

    // Fill the NTBs information
    if (!inBuildOptions.bRecomputeNormals || !inBuildOptions.bRecomputeTangents) {
        if (!inBuildOptions.bRecomputeTangents) {
            for (int ii = 0; ii < wedges.size(); ii++) {
                tangentX.push_back(glm::vec3(0));
                tangentY.push_back(glm::vec3(0));
            }
        }

        if (!inBuildOptions.bRecomputeNormals) {
            for (int ii = 0; ii < wedges.size(); ii++) {
                tangentZ.push_back(glm::vec3(0));
            }
        }

        for (const Importer::MeshImportData::MeshFace& meshFace : faces) {
            for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
                uint32_t WedgeIndex = meshFace.iWedge[cornerIndex];
                if (!inBuildOptions.bRecomputeTangents) {
                    tangentX[WedgeIndex] = meshFace.tangentX[cornerIndex];
                    tangentY[WedgeIndex] = meshFace.tangentY[cornerIndex];
                }
                if (!inBuildOptions.bRecomputeNormals) {
                    tangentZ[WedgeIndex] = meshFace.tangentZ[cornerIndex];
                }
            }
        }
    }
}

uint32_t IMeshBuildInputData::GetWedgeIndex(uint32_t faceIndex, uint32_t triIndex) { return faces[faceIndex].iWedge[triIndex]; }

uint32_t IMeshBuildInputData::GetVertexIndex(uint32_t wedgeIndex) { return wedges[wedgeIndex].iVertex; }

uint32_t IMeshBuildInputData::GetVertexIndex(uint32_t faceIndex, uint32_t triIndex) { return wedges[faces[faceIndex].iWedge[triIndex]].iVertex; }

glm::vec3 IMeshBuildInputData::GetVertexPosition(uint32_t wedgeIndex) { return points[wedges[wedgeIndex].iVertex]; }

glm::vec3 IMeshBuildInputData::GetVertexPosition(uint32_t faceIndex, uint32_t triIndex) { return points[wedges[faces[faceIndex].iWedge[triIndex]].iVertex]; }

glm::dvec2 IMeshBuildInputData::GetVertexUV(uint32_t faceIndex, uint32_t triIndex, uint32_t UVIndex) { return wedges[faces[faceIndex].iWedge[triIndex]].uvs[UVIndex]; }

uint32_t IMeshBuildInputData::GetFaceSmoothingGroups(uint32_t faceIndex) { return faces[faceIndex].smoothingGroups; }

uint32_t IMeshBuildInputData::GetNumFaces() { return static_cast<uint32_t>(faces.size()); }

uint32_t IMeshBuildInputData::GetNumWedges() { return static_cast<uint32_t>(wedges.size()); }

SMikkTSpaceInterface* IMeshBuildInputData::GetMikkTInterface() { return &mikkTInterface; }

void* IMeshBuildInputData::GetMikkTUserData() { return (void*)&mikkTUserData; }

void IMeshBuildInputData::ValidateTangentArraySize() {
    ASSERT(tangentX.size() == wedges.size());
    ASSERT(tangentY.size() == wedges.size());
    ASSERT(tangentZ.size() == wedges.size());
}

namespace MeshHelper {
void FindOverlappingCorners(std::shared_ptr<OverlappingCorners> outOverlappingCorners, IMeshBuildInputData& buildData, float comparisonThreshold) {
    int numFaces = buildData.GetNumFaces();
    int numWedges = buildData.GetNumWedges();
    ASSERT(numFaces * 3 <= numWedges);

    // Create a list of vertex Z/index pairs
    std::vector<IndexAndZ> vertIndexAndZ;
    for (int faceIndex = 0; faceIndex < numFaces; faceIndex++) {
        for (int triIndex = 0; triIndex < 3; triIndex++) {
            uint32_t index = buildData.GetWedgeIndex(faceIndex, triIndex);
            vertIndexAndZ.push_back(IndexAndZ(index, buildData.GetVertexPosition(index)));
        }
    }

    // Sort the vertices by z value
    std::sort(vertIndexAndZ.begin(), vertIndexAndZ.end(), CompareIndexAndZ());

    outOverlappingCorners->Init(numWedges);

    // Search for duplicates, quickly!
    for (int i = 0; i < vertIndexAndZ.size(); i++) {
        // only need to search forward, since we add pairs both ways
        for (int j = i + 1; j < vertIndexAndZ.size(); j++) {
            if (Maths::Abs(vertIndexAndZ[j].z - vertIndexAndZ[i].z) > comparisonThreshold) break;  // can't be any more dups

            glm::vec3 PositionA = buildData.GetVertexPosition(vertIndexAndZ[i].index);
            glm::vec3 PositionB = buildData.GetVertexPosition(vertIndexAndZ[j].index);

            if (PointsEqual(PositionA, PositionB, comparisonThreshold)) {
                outOverlappingCorners->Add(vertIndexAndZ[i].index, vertIndexAndZ[j].index);
            }
        }
    }

    outOverlappingCorners->FinishAdding();
}

void ComputeTriangleTangents(std::vector<glm::vec3>& triangleTangentX, std::vector<glm::vec3>& triangleTangentY, std::vector<glm::vec3>& triangleTangentZ, IMeshBuildInputData& buildData, float ComparisonThreshold) {
    int numTriangles = buildData.GetNumFaces();

    // Currently GetSafeNormal do not support 0.0f threshold properly
    float realComparisonThreshold = Maths::Max(ComparisonThreshold, FLT_MIN);

    for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++) {
        const int UVIndex = 0;
        glm::vec3 p[3];

        for (int i = 0; i < 3; i++) {
            p[i] = buildData.GetVertexPosition(triangleIndex, i);
            if (buildData.sceneInfo->scaleFactor != 1.0f) {
                p[i] /= buildData.sceneInfo->scaleFactor;
            }
        }

        // get safe normal should have return a valid normalized vector or a zero vector.
        glm::vec3 normal = GetSafeNormal(glm::cross((p[1] - p[0]), (p[2] - p[1])), realComparisonThreshold);
        // Avoid doing orthonormal vector from a degenerated triangle.
        if (!Maths::IsNearlyZero(normal, FLT_MIN)) {
            glm::dvec2 T1 = buildData.GetVertexUV(triangleIndex, 0, UVIndex);
            glm::dvec2 T2 = buildData.GetVertexUV(triangleIndex, 1, UVIndex);
            glm::dvec2 T3 = buildData.GetVertexUV(triangleIndex, 2, UVIndex);
            glm::vec2 deltaUV1 = T2 - T1;
            glm::vec2 deltaUV2 = T3 - T1;
            glm::vec3 edge1 = p[1] - p[0];
            glm::vec3 edge2 = p[2] - p[0];
            float f = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
            triangleTangentX.push_back(glm::normalize(glm::vec3(f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x), f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y), f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z))));
            triangleTangentY.push_back(glm::normalize(glm::vec3(f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x), f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y), f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z))));
            if (glm::dot(normal, glm::normalize(glm::cross(triangleTangentX[triangleIndex], triangleTangentY[triangleIndex]))) < 0) {
                triangleTangentX[triangleIndex] *= -1;
            }
            triangleTangentZ.push_back(normal);
            // CreateOrthonormalBasis(triangleTangentX[triangleIndex], triangleTangentY[triangleIndex], triangleTangentZ[triangleIndex]);

            if (Maths::IsNearlyZero(triangleTangentX[triangleIndex]) || Maths::ContainsNaN(triangleTangentX[triangleIndex]) || Maths::IsNearlyZero(triangleTangentY[triangleIndex]) || Maths::ContainsNaN(triangleTangentY[triangleIndex]) || Maths::IsNearlyZero(triangleTangentZ[triangleIndex]) ||
                Maths::ContainsNaN(triangleTangentZ[triangleIndex])) {
                triangleTangentX[triangleIndex] = glm::vec3(0);
                triangleTangentY[triangleIndex] = glm::vec3(0);
                triangleTangentZ[triangleIndex] = glm::vec3(0);
            }
        } else {
            // Add zero tangents and normal for this triangle, this is like weighting it to zero when we compute the
            // vertex normal But we need the triangle to correctly connect other neighbourg triangles
            triangleTangentX.push_back(glm::vec3(0));
            triangleTangentY.push_back(glm::vec3(0));
            triangleTangentZ.push_back(glm::vec3(0));
        }
    }
}

void AddAdjacentFace(IMeshBuildInputData& buildData, std::vector<bool>& faceAdded, std::map<int, std::vector<int>>& vertexIndexToAdjacentFaces, int faceIndex, std::vector<int>& triangleQueue, const bool bConnectByEdge) {
    int numFaces = static_cast<int>(buildData.GetNumFaces());
    ASSERT(faceAdded.size() == numFaces);

    std::map<int, int> adjacentFaceCommonVertices;
    for (int corner = 0; corner < 3; corner++) {
        int vertexIndex = buildData.GetVertexIndex(faceIndex, corner);
        ASSERT(vertexIndexToAdjacentFaces.find(vertexIndex) != vertexIndexToAdjacentFaces.end());
        std::vector<int>& adjacentFaces = vertexIndexToAdjacentFaces[vertexIndex];
        for (int adjacentFaceArrayIndex = 0; adjacentFaceArrayIndex < adjacentFaces.size(); adjacentFaceArrayIndex++) {
            int adjacentFaceIndex = adjacentFaces[adjacentFaceArrayIndex];
            if (!faceAdded[adjacentFaceIndex] && adjacentFaceIndex != faceIndex) {
                bool bAddConnected = !bConnectByEdge;
                if (bConnectByEdge) {
                    int& AdjacentFaceCommonVerticeCount = adjacentFaceCommonVertices[adjacentFaceIndex];
                    AdjacentFaceCommonVerticeCount++;
                    // Is the connected triangles share 2 vertex index (one edge) not only one vertex
                    bAddConnected = AdjacentFaceCommonVerticeCount > 1;
                }

                if (bAddConnected) {
                    triangleQueue.push_back(adjacentFaceIndex);
                    // Add the face only once by marking the face has computed
                    faceAdded[adjacentFaceIndex] = true;
                }
            }
        }
    }
}

void FillPolygonPatch(IMeshBuildInputData& buildData, std::vector<int>& faceIndexToPatchIndex, const bool bConnectByEdge) {
    int numTriangles = buildData.GetNumFaces();
    ASSERT(faceIndexToPatchIndex.size() == numTriangles);

    int patchIndex = 0;

    std::map<int, std::vector<int>> vertexIndexToAdjacentFaces;
    for (int faceIndex = 0; faceIndex < numTriangles; faceIndex++) {
        int wedgeOffset = faceIndex * 3;
        for (int corner = 0; corner < 3; corner++) {
            int vertexIndex = buildData.GetVertexIndex(faceIndex, corner);
            std::vector<int>& AdjacentFaces = vertexIndexToAdjacentFaces[vertexIndex];
            if (std::find(AdjacentFaces.begin(), AdjacentFaces.end(), faceIndex) == AdjacentFaces.end()) {
                AdjacentFaces.push_back(faceIndex);
            }
        }
    }

    // Mark added face so we do not add them more then once
    std::vector<bool> faceAdded(numTriangles, false);

    std::vector<int> triangleQueue;
    for (int faceIndex = 0; faceIndex < numTriangles; faceIndex++) {
        if (faceAdded[faceIndex]) {
            continue;
        }
        triangleQueue.clear();
        triangleQueue.push_back(faceIndex);  // Use a queue to avoid recursive function
        faceAdded[faceIndex] = true;
        while (triangleQueue.size() > 0) {
            int currentTriangleIndex = triangleQueue.back();
            triangleQueue.pop_back();
            faceIndexToPatchIndex[currentTriangleIndex] = patchIndex;
            AddAdjacentFace(buildData, faceAdded, vertexIndexToAdjacentFaces, currentTriangleIndex, triangleQueue, bConnectByEdge);
        }
        patchIndex++;
    }
}

bool IsTriangleMirror(IMeshBuildInputData& buildData, const std::vector<glm::vec3>& triangleTangentZ, const uint32_t faceIdxA, const uint32_t faceIdxB) {
    if (faceIdxA == faceIdxB) {
        return false;
    }
    for (int cornerA = 0; cornerA < 3; cornerA++) {
        const glm::vec3& cornerAPosition = buildData.GetVertexPosition((faceIdxA * 3) + cornerA);
        bool bFoundMatch = false;
        for (int cornerB = 0; cornerB < 3; cornerB++) {
            const glm::vec3& cornerBPosition = buildData.GetVertexPosition((faceIdxB * 3) + cornerB);
            if (PointsEqual(cornerAPosition, cornerBPosition, buildData.buildOptions)) {
                bFoundMatch = true;
                break;
            }
        }

        if (!bFoundMatch) {
            return false;
        }
    }
    // Check if the triangles normals are opposite and parallel. Dot product equal -1.0f
    if (Maths::IsNearlyEqual(glm::dot(triangleTangentZ[faceIdxA], triangleTangentZ[faceIdxB]), -1.0f, KINDA_SMALL_NUMBER)) {
        return true;
    }
    return false;
}

void ComputeTangents(const std::string& skinnedMeshName, IMeshBuildInputData& buildData, std::shared_ptr<OverlappingCorners> OverlappingCorners) {
    bool bBlendOverlappingNormals = true;
    bool bIgnoreDegenerateTriangles = buildData.buildOptions.bRemoveDegenerates;
    bool bComputeWeightedNormals = buildData.buildOptions.bComputeWeightedNormals;
    bool bUseMikktSpace = buildData.buildOptions.bUseMikkTSpace && (buildData.buildOptions.bRecomputeNormals || buildData.buildOptions.bRecomputeTangents);

    int numFaces = buildData.GetNumFaces();
    int numWedges = buildData.GetNumWedges();
    ASSERT(numFaces * 3 <= numWedges);

    // Compute per-triangle tangents.
    std::vector<glm::vec3> triangleTangentX;
    std::vector<glm::vec3> triangleTangentY;
    std::vector<glm::vec3> triangleTangentZ;

    // 先计算每个三角形的法线向量，并根据UV信息建立一个切线空间正交坐标系，其中TriangleTangentZ的坐标轴对应法线的反向。
    ComputeTriangleTangents(triangleTangentX, triangleTangentY, triangleTangentZ, buildData, bIgnoreDegenerateTriangles ? SMALL_NUMBER : FLT_MIN);

    std::vector<int> faceIndexToPatchIndex;
    faceIndexToPatchIndex.resize(numFaces);
    // Since we use triangle normals to compute the vertex normal, we need a full edge connected (2 vertex component per
    // triangle)
    const bool bConnectByEdge = true;

    // 根据三角形面的相邻状况，把网格拆分成各自独立的连通区域。
    FillPolygonPatch(buildData, faceIndexToPatchIndex, bConnectByEdge);

    std::vector<glm::vec3>& wedgeTangentX = buildData.tangentX;
    std::vector<glm::vec3>& wedgeTangentY = buildData.tangentY;
    std::vector<glm::vec3>& wedgeTangentZ = buildData.tangentZ;

    // Declare these out here to avoid reallocations.
    std::vector<FanFace> relevantFacesForCorner[3];
    std::vector<int> adjacentFaces;

    // Allocate storage for tangents and normal if none were provided.
    if (wedgeTangentX.size() != numWedges) {
        wedgeTangentX.resize(numWedges, glm::vec3(0));
    }
    if (wedgeTangentY.size() != numWedges) {
        wedgeTangentY.resize(numWedges, glm::vec3(0));
    }
    if (wedgeTangentZ.size() != numWedges) {
        wedgeTangentZ.resize(numWedges, glm::vec3(0));
    }

    bool bIsZeroLengthNormalErrorMessageDisplayed = false;
    // we need to calculate normals for MikkTSpace
    for (int faceIndex = 0; faceIndex < numFaces; faceIndex++) {
        int patchIndex = faceIndexToPatchIndex[faceIndex];
        int wedgeOffset = faceIndex * 3;
        glm::vec3 cornerPositions[3];
        glm::vec3 cornerTangentX[3];
        glm::vec3 cornerTangentY[3];
        glm::vec3 cornerNormal[3];

        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            cornerTangentX[cornerIndex] = glm::vec3(0);
            cornerTangentY[cornerIndex] = glm::vec3(0);
            cornerNormal[cornerIndex] = glm::vec3(0);
            cornerPositions[cornerIndex] = buildData.GetVertexPosition(faceIndex, cornerIndex);
            relevantFacesForCorner[cornerIndex].clear();
        }

        // Don't process degenerate triangles.
        if (PointsEqual(cornerPositions[0], cornerPositions[1], buildData.buildOptions) || PointsEqual(cornerPositions[0], cornerPositions[2], buildData.buildOptions) || PointsEqual(cornerPositions[1], cornerPositions[2], buildData.buildOptions)) {
            continue;
        }

        // No need to process triangles if tangents already exist.
        bool bCornerHasNormal[3] = {0};
        bool bCornerHasTangents[3] = {0};
        bool bSkipNTB = true;
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            bCornerHasNormal[cornerIndex] = !Maths::IsNearlyZero(wedgeTangentZ[wedgeOffset + cornerIndex]);
            bCornerHasTangents[cornerIndex] = !Maths::IsNearlyZero(wedgeTangentX[wedgeOffset + cornerIndex]) && !Maths::IsNearlyZero(wedgeTangentY[wedgeOffset + cornerIndex]);

            // If we want to compute mikkt we dont check tangents to skip this corner
            if (!bCornerHasNormal[cornerIndex] || (!bUseMikktSpace && !bCornerHasTangents[cornerIndex])) {
                bSkipNTB = false;
            }
        }
        if (bSkipNTB) {
            continue;
        }

        // Calculate smooth vertex normals.
        // todo
        float determinant = glm::determinant(glm::mat3(triangleTangentX[faceIndex], triangleTangentY[faceIndex], triangleTangentZ[faceIndex]));

        // 首先得到当前三角形相邻的所有三角形，这里不仅要看点，还要考虑同位置的点（OverlappingCorners），而且是要同一个连通区域的同位置的点，这里还要去除镜像面的case。因为镜像面会使得混合法线权重出问题。

        // Start building a list of faces adjacent to this face.
        adjacentFaces.clear();
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            int thisCornerIndex = wedgeOffset + cornerIndex;
            const std::vector<int>& dupVerts = OverlappingCorners->FindIfOverlapping(thisCornerIndex);
            if (dupVerts.size() == 0) {
                if (std::find(adjacentFaces.begin(), adjacentFaces.end(), thisCornerIndex / 3) == adjacentFaces.end()) {
                    adjacentFaces.push_back(thisCornerIndex / 3);  // I am a "dup" of myself
                }
            }
            for (int k = 0; k < dupVerts.size(); k++) {
                int potentialTriangleIndex = dupVerts[k] / 3;
                // Do not add a triangle that was remove
                if (triangleTangentZ.size() > potentialTriangleIndex && potentialTriangleIndex >= 0) {
                    bool bDegeneratedTriangles = Maths::IsNearlyZero(triangleTangentZ[faceIndex]) || Maths::IsNearlyZero(triangleTangentZ[potentialTriangleIndex]);
                    // Do not add mirror triangle to the adjacentFaces. Also make sure adjacent triangle is in the same
                    // connected triangle patch. Accept connected degenerate triangle
                    if ((bDegeneratedTriangles || !IsTriangleMirror(buildData, triangleTangentZ, faceIndex, potentialTriangleIndex)) && patchIndex == faceIndexToPatchIndex[potentialTriangleIndex]) {
                        if (std::find(adjacentFaces.begin(), adjacentFaces.end(), potentialTriangleIndex) == adjacentFaces.end()) {
                            adjacentFaces.push_back(potentialTriangleIndex);
                        }
                    }
                }
            }
        }

        // We need to sort these here because the criteria for point equality is
        // exact, so we must ensure the exact same order for all dups.
        std::sort(adjacentFaces.begin(), adjacentFaces.end());

        // 遍历所有邻面，获取当前面中每个点在邻面中的链接情况。
        // Process adjacent faces
        for (int adjacentFaceIndex = 0; adjacentFaceIndex < adjacentFaces.size(); adjacentFaceIndex++) {
            int otherFaceIndex = adjacentFaces[adjacentFaceIndex];
            for (int ourCornerIndex = 0; ourCornerIndex < 3; ourCornerIndex++) {
                if (bCornerHasNormal[ourCornerIndex] && (bUseMikktSpace || bCornerHasTangents[ourCornerIndex])) continue;

                FanFace newFanFace;
                int commonIndexCount = 0;

                // Check for vertices in common.
                if (faceIndex == otherFaceIndex) {
                    commonIndexCount = 3;
                    newFanFace.linkedVertexIndex = ourCornerIndex;
                } else {
                    // Check matching vertices against main vertex .
                    for (int otherCornerIndex = 0; otherCornerIndex < 3; otherCornerIndex++) {
                        if (PointsEqual(cornerPositions[ourCornerIndex], buildData.GetVertexPosition(otherFaceIndex, otherCornerIndex), buildData.buildOptions)) {
                            commonIndexCount++;
                            newFanFace.linkedVertexIndex = otherCornerIndex;
                        }
                    }
                }

                // Add if connected by at least one point. Smoothing matches are considered later.
                if (commonIndexCount > 0) {
                    newFanFace.faceIndex = otherFaceIndex;
                    newFanFace.bFilled = (otherFaceIndex == faceIndex);  // Starter face for smoothing floodfill.
                    newFanFace.bBlendTangents = newFanFace.bFilled;
                    newFanFace.bBlendNormals = newFanFace.bFilled;
                    relevantFacesForCorner[ourCornerIndex].push_back(newFanFace);
                }
            }
        }

        // 根据平滑组信息去过滤出正真需要参与法线混合的邻面。
        // Find true relevance of faces for a vertex normal by traversing
        // smoothing-group-compatible connected triangle fans around common vertices.
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            if (bCornerHasNormal[cornerIndex] && (bUseMikktSpace || bCornerHasTangents[cornerIndex])) continue;

            int newConnections;
            do {
                newConnections = 0;
                for (int otherFaceIdx = 0; otherFaceIdx < relevantFacesForCorner[cornerIndex].size(); otherFaceIdx++) {
                    FanFace& otherFace = relevantFacesForCorner[cornerIndex][otherFaceIdx];
                    // The vertex' own face is initially the only face with bFilled == true.
                    if (otherFace.bFilled) {
                        for (int nextFaceIndex = 0; nextFaceIndex < relevantFacesForCorner[cornerIndex].size(); nextFaceIndex++) {
                            FanFace& nextFace = relevantFacesForCorner[cornerIndex][nextFaceIndex];
                            if (!nextFace.bFilled)  // && !NextFace.bBlendTangents)
                            {
                                if ((nextFaceIndex != otherFaceIdx) && (buildData.GetFaceSmoothingGroups(nextFace.faceIndex) & buildData.GetFaceSmoothingGroups(otherFace.faceIndex))) {
                                    int commonVertices = 0;
                                    int commonTangentVertices = 0;
                                    int commonNormalVertices = 0;
                                    for (int otherCornerIndex = 0; otherCornerIndex < 3; otherCornerIndex++) {
                                        for (int nextCornerIndex = 0; nextCornerIndex < 3; nextCornerIndex++) {
                                            int nextVertexIndex = buildData.GetVertexIndex(nextFace.faceIndex, nextCornerIndex);
                                            int otherVertexIndex = buildData.GetVertexIndex(otherFace.faceIndex, otherCornerIndex);
                                            if (PointsEqual(buildData.GetVertexPosition(nextFace.faceIndex, nextCornerIndex), buildData.GetVertexPosition(otherFace.faceIndex, otherCornerIndex), buildData.buildOptions)) {
                                                commonVertices++;
                                                if (!bUseMikktSpace && UVsEqual(buildData.GetVertexUV(nextFace.faceIndex, nextCornerIndex, 0), buildData.GetVertexUV(otherFace.faceIndex, otherCornerIndex, 0), buildData.buildOptions)) {
                                                    commonTangentVertices++;
                                                }
                                                if (bBlendOverlappingNormals || nextVertexIndex == otherVertexIndex) {
                                                    commonNormalVertices++;
                                                }
                                            }
                                        }
                                    }
                                    // Flood fill faces with more than one common vertices which must be touching edges.
                                    if (commonVertices > 1) {
                                        nextFace.bFilled = true;
                                        nextFace.bBlendNormals = (commonNormalVertices > 1);
                                        newConnections++;
                                        // Only blend tangents if there is no UV seam along the edge with this face.
                                        if (!bUseMikktSpace && otherFace.bBlendTangents && commonTangentVertices > 1) {
                                            float OtherDeterminant = glm::determinant(glm::mat3(triangleTangentX[nextFace.faceIndex], triangleTangentY[nextFace.faceIndex], triangleTangentZ[nextFace.faceIndex]));
                                            if ((determinant * OtherDeterminant) > 0.0f) {
                                                nextFace.bBlendTangents = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } while (newConnections > 0);
        }

        // Vertex normal construction.
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            bool bUseProvidedNormal = bCornerHasNormal[cornerIndex];
            bool bUseProvidedTangents = !bUseMikktSpace && bCornerHasTangents[cornerIndex];
            if (bUseProvidedNormal) {
                cornerNormal[cornerIndex] = wedgeTangentZ[wedgeOffset + cornerIndex];
            }
            if (bUseProvidedTangents) {
                cornerTangentX[cornerIndex] = wedgeTangentX[wedgeOffset + cornerIndex];
                cornerTangentY[cornerIndex] = wedgeTangentY[wedgeOffset + cornerIndex];
            }

            if (!bUseProvidedNormal || !bUseProvidedTangents) {
                for (int relevantFaceIdx = 0; relevantFaceIdx < relevantFacesForCorner[cornerIndex].size(); relevantFaceIdx++) {
                    FanFace const& relevantFace = relevantFacesForCorner[cornerIndex][relevantFaceIdx];
                    if (relevantFace.bFilled) {
                        int otherFaceIndex = relevantFace.faceIndex;
                        float cornerWeight = 1.0f;
                        if (bComputeWeightedNormals) {
                            glm::vec3 otherFacePoint[3] = {buildData.GetVertexPosition(otherFaceIndex, 0), buildData.GetVertexPosition(otherFaceIndex, 1), buildData.GetVertexPosition(otherFaceIndex, 2)};
                            float otherFaceArea = TriangleUtilities::ComputeTriangleArea(otherFacePoint[0], otherFacePoint[1], otherFacePoint[2]);
                            int otherFaceCornerIndex = relevantFace.linkedVertexIndex;
                            float otherFaceAngle = TriangleUtilities::ComputeTriangleCornerAngle(otherFacePoint[otherFaceCornerIndex], otherFacePoint[(otherFaceCornerIndex + 1) % 3], otherFacePoint[(otherFaceCornerIndex + 2) % 3]);
                            // Get the CornerWeight
                            cornerWeight = otherFaceArea * otherFaceAngle;
                        }
                        if (!bUseProvidedTangents && relevantFace.bBlendTangents) {
                            cornerTangentX[cornerIndex] += cornerWeight * triangleTangentX[otherFaceIndex];
                            cornerTangentY[cornerIndex] += cornerWeight * triangleTangentY[otherFaceIndex];
                        }
                        if (!bUseProvidedNormal && relevantFace.bBlendNormals) {
                            cornerNormal[cornerIndex] += cornerWeight * triangleTangentZ[otherFaceIndex];
                        }
                    }
                }
                if (!Maths::IsNearlyZero(wedgeTangentX[wedgeOffset + cornerIndex])) {
                    cornerTangentX[cornerIndex] = wedgeTangentX[wedgeOffset + cornerIndex];
                }
                if (!Maths::IsNearlyZero(wedgeTangentY[wedgeOffset + cornerIndex])) {
                    cornerTangentY[cornerIndex] = wedgeTangentY[wedgeOffset + cornerIndex];
                }
                if (!Maths::IsNearlyZero(wedgeTangentZ[wedgeOffset + cornerIndex])) {
                    cornerNormal[cornerIndex] = wedgeTangentZ[wedgeOffset + cornerIndex];
                }
            }
        }

        // Normalization.
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            cornerNormal[cornerIndex] = glm::normalize(cornerNormal[cornerIndex]);
            if (!bUseMikktSpace) {
                cornerTangentX[cornerIndex] = glm::normalize(cornerTangentX[cornerIndex]);
                cornerTangentY[cornerIndex] = glm::normalize(cornerTangentY[cornerIndex]);

                // Gram-Schmidt orthogonalization
                cornerTangentY[cornerIndex] -= cornerTangentX[cornerIndex] * glm::dot(cornerTangentX[cornerIndex], cornerTangentY[cornerIndex]);
                cornerTangentY[cornerIndex] = glm::normalize(cornerTangentY[cornerIndex]);

                cornerTangentX[cornerIndex] -= cornerNormal[cornerIndex] * glm::dot(cornerNormal[cornerIndex], cornerTangentX[cornerIndex]);
                cornerTangentX[cornerIndex] = glm::normalize(cornerTangentX[cornerIndex]);
                cornerTangentY[cornerIndex] -= cornerNormal[cornerIndex] * glm::dot(cornerNormal[cornerIndex], cornerTangentY[cornerIndex]);
                cornerTangentY[cornerIndex] = glm::normalize(cornerTangentY[cornerIndex]);
            }
        }

        auto VerifyTangentSpace = [&bIsZeroLengthNormalErrorMessageDisplayed, &buildData, &skinnedMeshName](glm::vec3& normalizedVector) {
            if (Maths::IsNearlyZero(normalizedVector) || Maths::ContainsNaN(normalizedVector)) {
                normalizedVector = glm::vec3(0);
                // We also notify the log that we compute a zero length normals, so the user is aware of it
                if (!bIsZeroLengthNormalErrorMessageDisplayed) {
                    bIsZeroLengthNormalErrorMessageDisplayed = true;
                    LOG_WARN("Mesh contains zero normal vector.");
                }
            }
        };

        // Copy back to the mesh.
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
            VerifyTangentSpace(cornerNormal[cornerIndex]);
            wedgeTangentZ[wedgeOffset + cornerIndex] = cornerNormal[cornerIndex];

            if (!bUseMikktSpace) {
                VerifyTangentSpace(cornerTangentX[cornerIndex]);
                wedgeTangentX[wedgeOffset + cornerIndex] = cornerTangentX[cornerIndex];

                VerifyTangentSpace(cornerTangentY[cornerIndex]);
                wedgeTangentY[wedgeOffset + cornerIndex] = cornerTangentY[cornerIndex];
            }
        }
    }

    if (bUseMikktSpace) {
        // we can use mikktspace to calculate the tangents
        SMikkTSpaceContext MikkTContext;
        MikkTContext.m_pInterface = buildData.GetMikkTInterface();
        MikkTContext.m_pUserData = buildData.GetMikkTUserData();
        // MikkTContext.m_bIgnoreDegenerates = bIgnoreDegenerateTriangles;

        genTangSpaceDefault(&MikkTContext);
    }

    ASSERT(wedgeTangentX.size() == numWedges);
    ASSERT(wedgeTangentY.size() == numWedges);
    ASSERT(wedgeTangentZ.size() == numWedges);
}

}  // namespace MeshHelper

bool AreMeshVerticesEqual(const SubMeshVertexWithWedgeIdx& v1, const SubMeshVertexWithWedgeIdx& v2, const MeshBuildSettings& buildOptions) {
    if (!PointsEqual(v1.position, v2.position, buildOptions)) {
        return false;
    }

    for (int UVIdx = 0; UVIdx < Configuration::MaxTexCoord; UVIdx++) {
        if (!UVsEqual(v1.uvs[UVIdx], v2.uvs[UVIdx], buildOptions)) {
            return false;
        }
    }

    if (!NormalsEqual(v1.tangentX, v2.tangentX, buildOptions)) {
        return false;
    }

    if (!NormalsEqual(v1.tangentY, v2.tangentY, buildOptions)) {
        return false;
    }

    if (!NormalsEqual(v1.tangentZ, v2.tangentZ, buildOptions)) {
        return false;
    }

    if (v1.influenceBones.size() != v2.influenceBones.size()) {
        return false;
    }

    bool influencesMatch = 1;
    for (uint32_t influenceIndex = 0; influenceIndex < v1.influenceBones.size(); influenceIndex++) {
        if (v1.influenceBones[influenceIndex] != v2.influenceBones[influenceIndex] || v1.influenceWeights[influenceIndex] != v2.influenceWeights[influenceIndex]) {
            influencesMatch = 0;
            break;
        }
    }

    if (v1.color != v2.color) {
        return false;
    }

    if (!influencesMatch) {
        return false;
    }

    return true;
}

MeshBuildInputData::MeshBuildInputData(MeshAsset<>& inModel, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences, const std::vector<Importer::MeshImportData::MeshWedge>& inWedges, const std::vector<Importer::MeshImportData::MeshFace>& inFaces,
                                       const std::vector<glm::vec3>& inPoints, const std::vector<int>& inPointToOriginalMap, const std::shared_ptr<Importer::SceneInfo>& inSceneInfo, const MeshBuildSettings& inBuildOptions)
    : IMeshBuildInputData(inWedges, inFaces, inPoints, inInfluences, inPointToOriginalMap, inSceneInfo, inBuildOptions), model(inModel) {}

bool MeshBuilder::Build(std::shared_ptr<Mesh<>> mesh, std::shared_ptr<Importer::MeshImportData> meshImportData, std::shared_ptr<Importer::SceneInfo> sceneInfo, const MeshBuildSettings& options) {
    std::vector<glm::vec3> points;
    std::vector<Importer::MeshImportData::MeshWedge> wedges;
    std::vector<Importer::MeshImportData::MeshFace> faces;
    std::vector<Importer::MeshImportData::VertexInfluence> influences;
    std::vector<int> pointToRawMap;
    int numTextCoord = 1;
    // Copy vertex data.
    points.resize(meshImportData->points.size());
    for (int p = 0; p < meshImportData->points.size(); p++) {
        points[p] = meshImportData->points[p];
    }

    // Copy wedge information to static LOD level.
    wedges.resize(meshImportData->wedges.size());
    for (int w = 0; w < meshImportData->wedges.size(); w++) {
        wedges[w].iVertex = meshImportData->wedges[w].vertexIndex;
        // Copy all texture coordinates
        for (int i = 0; i < Configuration::MaxTexCoord; i++) {
            wedges[w].uvs[i] = meshImportData->wedges[w].uvs[i];
        }
        wedges[w].color = meshImportData->wedges[w].color;
    }

    // Copy triangle/ face data to static LOD level.
    faces.resize(meshImportData->faces.size());
    for (int f = 0; f < meshImportData->faces.size(); f++) {
        Importer::MeshImportData::MeshFace face;
        face.iWedge[0] = meshImportData->faces[f].wedgeIndex[0];
        face.iWedge[1] = meshImportData->faces[f].wedgeIndex[1];
        face.iWedge[2] = meshImportData->faces[f].wedgeIndex[2];
        face.meshMaterialIndex = meshImportData->faces[f].materialIndex;

        face.tangentX[0] = meshImportData->faces[f].tangentX[0];
        face.tangentX[1] = meshImportData->faces[f].tangentX[1];
        face.tangentX[2] = meshImportData->faces[f].tangentX[2];

        face.tangentY[0] = meshImportData->faces[f].tangentY[0];
        face.tangentY[1] = meshImportData->faces[f].tangentY[1];
        face.tangentY[2] = meshImportData->faces[f].tangentY[2];

        face.tangentZ[0] = meshImportData->faces[f].tangentZ[0];
        face.tangentZ[1] = meshImportData->faces[f].tangentZ[1];
        face.tangentZ[2] = meshImportData->faces[f].tangentZ[2];

        face.smoothingGroups = meshImportData->faces[f].smoothingGroups;

        faces[f] = face;
    }

    // Copy weights/ influences to static LOD level.
    influences.resize(meshImportData->influences.size());
    for (int i = 0; i < meshImportData->influences.size(); i++) {
        influences[i].weight = meshImportData->influences[i].weight;
        influences[i].vertexIndex = meshImportData->influences[i].vertexIndex;
        influences[i].boneIndex = meshImportData->influences[i].boneIndex;
    }

    // Copy mapping
    pointToRawMap = meshImportData->pointToRawMap;

    // Use the max because we need to have at least one texture coordinnate
    numTextCoord = Maths::Max<int>(numTextCoord, meshImportData->numTexCoords);

    std::shared_ptr<MeshAsset<>> model = std::make_shared<MeshAsset<>>();

    BuildMesh(*model, mesh->name, influences, wedges, faces, points, pointToRawMap, sceneInfo, options);

    mesh->asset = model;

    return true;
}

void ChunkVertices(std::vector<std::shared_ptr<OriginSubMesh>>& chunks) {
    // Copy over the old chunks (this is just copying pointers).
    std::vector<std::shared_ptr<OriginSubMesh>> srcChunks = chunks;
    chunks.clear();

    // Sort the chunks by material index.
    struct CompareSkinnedMeshChunk {
        bool operator()(const std::shared_ptr<OriginSubMesh>& A, const std::shared_ptr<OriginSubMesh>& B) const { return A->materialIndex < B->materialIndex; }
    };
    std::sort(srcChunks.begin(), srcChunks.end(), CompareSkinnedMeshChunk());

    chunks = srcChunks;
}

bool MeshBuilder::GenerateRenderableMesh(MeshBuildInputData& buildData) {
    std::vector<VertIndexAndZ> vertIndexAndZ;
    std::vector<SubMeshVertexWithWedgeIdx> rawVertices;

    for (int faceIndex = 0; faceIndex < buildData.faces.size(); faceIndex++) {
        // Only update the status progress bar if we are in the game thread and every thousand faces.
        // Updating status is extremely slow
        if (faceIndex % 5000 == 0) {
            LOG_INFO(fmt::format("Processing triangles: {:d}/{:d} .", faceIndex + 1, buildData.faces.size()));
        }

        const Importer::MeshImportData::MeshFace& Face = buildData.faces[faceIndex];

        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
            SubMeshVertexWithWedgeIdx vertex;
            const uint32_t WedgeIndex = buildData.GetWedgeIndex(faceIndex, vertexIndex);
            const Importer::MeshImportData::MeshWedge& wedge = buildData.wedges[WedgeIndex];

            vertex.position = buildData.GetVertexPosition(faceIndex, vertexIndex);

            glm::vec3 tangentX, tangentY, tangentZ;
            tangentX = GetSafeNormal(buildData.tangentX[WedgeIndex]);
            tangentY = GetSafeNormal(buildData.tangentY[WedgeIndex]);
            tangentZ = GetSafeNormal(buildData.tangentZ[WedgeIndex]);

            // Normalize overridden tangents.  Its possible for them to import un-normalized.
            tangentX = glm::normalize(tangentX);
            tangentY = glm::normalize(tangentY);
            tangentZ = glm::normalize(tangentZ);

            vertex.tangentX = tangentX;
            vertex.tangentY = tangentY;
            vertex.tangentZ = tangentZ;

            for (int ii = 0; ii < Configuration::MaxTexCoord; ii++) {
                vertex.uvs[ii] = wedge.uvs[ii];
            }
            vertex.color = wedge.color;

            // Add the vertex as well as its original index in the points array
            vertex.pointWedgeIdx = wedge.iVertex;

            int qawIndex = static_cast<int>(rawVertices.size());
            rawVertices.push_back(vertex);

            // Add an efficient way to find dupes of this vertex later for fast combining of vertices
            VertIndexAndZ indexAndZ;
            indexAndZ.index = qawIndex;
            indexAndZ.z = vertex.position.z;

            vertIndexAndZ.push_back(indexAndZ);
        }
    }

    // Z轴排序网格点，按照material index分组，分组后去除重复节点，并构建index buffer。
    // Generate chunks and their vertices and indices
    BuildMeshChunks(buildData.faces, rawVertices, vertIndexAndZ, buildData.buildOptions, buildData.chunks);

    ChunkVertices(buildData.chunks);
    return true;
}

/**
 * Builds a renderable skeletal mesh LOD model. Note that the array of chunks
 * will be destroyed during this process!
 * @param LODModel				Upon return contains a renderable skeletal mesh LOD model.
 * @param RefSkeleton			The reference skeleton associated with the model.
 * @param Chunks				Skinned mesh chunks from which to build the renderable model.
 * @param PointToOriginalMap	Maps a vertex's RawPointIdx to its index at import time.
 */
void BuildMeshModelFromChunks(MeshAsset<>& model, std::vector<std::shared_ptr<OriginSubMesh>>& chunks, const std::vector<int>& pointToOriginalMap) {
    // Clear out any data currently held in the LOD model.
    model.sections.clear();
    model.numVertices = 0;
    model.indexBuffer.clear();

    // Setup the section and chunk arrays on the model.
    for (int chunkIndex = 0; chunkIndex < chunks.size(); chunkIndex++) {
        std::shared_ptr<OriginSubMesh> srcChunk = chunks[chunkIndex];
        model.sections.push_back(SubMeshAsset());
        SubMeshAsset& section = model.sections.back();
        section.materialIndex = srcChunk->materialIndex;

        section.originalDataSectionIndex = srcChunk->originalSectionIndex;
        section.chunkedParentSectionIndex = srcChunk->parentChunkSectionIndex;
    }

    // Reset 'final vertex to import vertex' map info
    model.meshToImportVertexMap.clear();
    model.maxImportVertex = 0;

    // Keep track of index mapping to chunk vertex offsets
    std::vector<std::vector<uint32_t>> vertexIndexRemap;
    // Pack the chunk vertices into a single vertex buffer.
    std::vector<uint32_t> rawPointIndices;
    model.numVertices = 0;

    int prevMaterialIndex = -1;
    int currentChunkBaseVertexIndex = -1;  // base vertex index for all chunks of the same material
    int currentChunkVertexCount = -1;      // total vertex count for all chunks of the same material
    int currentVertexIndex = 0;            // current vertex index added to the index buffer for all chunks of the same material

    // rearrange the vert order to minimize the data fetched by the GPU
    for (int sectionIndex = 0; sectionIndex < model.sections.size(); sectionIndex++) {
        LOG_INFO(fmt::format("Processing sections {}/{} .", sectionIndex + 1, model.sections.size()));

        std::shared_ptr<OriginSubMesh> srcChunk = chunks[sectionIndex];
        SubMeshAsset& section = model.sections[sectionIndex];
        std::vector<SubMeshVertexWithWedgeIdx>& chunkVertices = srcChunk->vertices;
        std::vector<uint32_t>& chunkIndices = srcChunk->indices;

        // Reorder the section index buffer for better vertex cache efficiency.
        CacheOptimizeIndexBuffer(chunkIndices);

        // Calculate the number of triangles in the section.  Note that CacheOptimize may change the number of triangles
        // in the index buffer!
        section.numTriangles = static_cast<uint32_t>(chunkIndices.size() / 3);
        std::vector<SubMeshVertexWithWedgeIdx> originalVertices = chunkVertices;
        chunkVertices.clear();
        chunkVertices.resize(originalVertices.size());

        std::vector<int> indexCache;
        indexCache.resize(chunkVertices.size(), -1);
        int nextAvailableIndex = 0;
        // Go through the indices and assign them new values that are coherent where possible
        for (int index = 0; index < chunkIndices.size(); index++) {
            const int originalIndex = chunkIndices[index];
            const int cachedIndex = indexCache[originalIndex];

            if (cachedIndex == -1) {
                // No new index has been allocated for this existing index, assign one
                chunkIndices[index] = nextAvailableIndex;
                // Mark what this index has been assigned to
                indexCache[originalIndex] = nextAvailableIndex;
                nextAvailableIndex++;
            } else {
                // Reuse an existing index assignment
                chunkIndices[index] = cachedIndex;
            }
            // Reorder the vertices based on the new index assignment
            chunkVertices[chunkIndices[index]] = originalVertices[originalIndex];
        }
    }

    // Build the arrays of rigid and soft vertices on the model's chunks.
    for (int sectionIndex = 0; sectionIndex < model.sections.size(); sectionIndex++) {
        SubMeshAsset& section = model.sections[sectionIndex];
        std::vector<SubMeshVertexWithWedgeIdx>& chunkVertices = chunks[sectionIndex]->vertices;
        LOG_INFO(fmt::format("Building sections {}/{} .", sectionIndex + 1, model.sections.size()));

        currentVertexIndex = 0;
        currentChunkVertexCount = 0;
        prevMaterialIndex = section.materialIndex;

        // Calculate the offset to this chunk's vertices in the vertex buffer.
        section.baseVertexIndex = currentChunkBaseVertexIndex = model.numVertices;

        // Update the size of the vertex buffer.
        model.numVertices += static_cast<uint32_t>(chunkVertices.size());

        // Separate the section's vertices into rigid and soft vertices.
        vertexIndexRemap.push_back(std::vector<uint32_t>());
        std::vector<uint32_t>& chunkVertexIndexRemap = vertexIndexRemap.back();
        chunkVertexIndexRemap.resize(chunkVertices.size());

        for (int vertexIndex = 0; vertexIndex < chunkVertices.size(); vertexIndex++) {
            const SubMeshVertexWithWedgeIdx& softVertex = chunkVertices[vertexIndex];

            SubMeshVertex newVertex;
            newVertex.position = softVertex.position;
            newVertex.tangentX = softVertex.tangentX;
            newVertex.tangentY = softVertex.tangentY;
            newVertex.tangentZ = glm::vec4(softVertex.tangentZ.x, softVertex.tangentZ.y, softVertex.tangentZ.z, 1);
            for (int ii = 0; ii < Configuration::MaxTexCoord; ii++) {
                newVertex.uvs[ii] = softVertex.uvs[ii];
            }
            newVertex.color = softVertex.color;
            section.vertices.push_back(newVertex);
            chunkVertexIndexRemap[vertexIndex] = static_cast<uint32_t>(section.baseVertexIndex + currentVertexIndex);
            currentVertexIndex++;
            // add the index to the original wedge point source of this vertex
            rawPointIndices.push_back(softVertex.pointWedgeIdx);
            // Also remember import index
            const int rawVertIndex = pointToOriginalMap[softVertex.pointWedgeIdx];
            model.meshToImportVertexMap.push_back(rawVertIndex);
            model.maxImportVertex = Maths::Max<int>(model.maxImportVertex, rawVertIndex);
        }

        // update NumVertices
        section.numVertices = static_cast<int>(section.vertices.size());

        // Log info about the chunk.
        LOG_INFO(fmt::format("Section {:d} has {:d} vertcies.", sectionIndex + 1, section.numVertices));
    }

    // Copy raw point indices to LOD model.
    model.rawPointIndices.clear();
    if (rawPointIndices.size()) {
        model.rawPointIndices = rawPointIndices;
    }

    // Finish building the sections.
    for (int sectionIndex = 0; sectionIndex < model.sections.size(); sectionIndex++) {
        SubMeshAsset& section = model.sections[sectionIndex];

        const std::vector<uint32_t>& sectionIndices = chunks[sectionIndex]->indices;

        section.baseIndex = static_cast<uint32_t>(model.indexBuffer.size());
        const int numIndices = static_cast<int>(sectionIndices.size());
        const std::vector<uint32_t>& sectionVertexIndexRemap = vertexIndexRemap[sectionIndex];
        for (int index = 0; index < numIndices; index++) {
            uint32_t vertexIndex = sectionVertexIndexRemap[sectionIndices[index]];
            model.indexBuffer.push_back(vertexIndex);
        }
    }

    // Free the skinned mesh chunks which are no longer needed.
    for (int i = 0; i < chunks.size(); i++) {
        chunks[i] = nullptr;
    }
    chunks.clear();
}

bool PrepareSourceMesh(const std::string& skinnedMeshName, IMeshBuildInputData& buildData, std::vector<std::shared_ptr<OverlappingCorners>>& overlappingCorners) {
    std::shared_ptr<OverlappingCorners> corners = std::make_shared<OverlappingCorners>();
    overlappingCorners.push_back(corners);
    float comparisonThreshold = THRESH_POINTS_ARE_SAME;  // GetComparisonThreshold(LODBuildSettings[LODIndex]);
    int numWedges = buildData.GetNumWedges();

    // Find overlapping corners to accelerate adjacency.
    MeshHelper::FindOverlappingCorners(corners, buildData, comparisonThreshold);

    // Figure out if we should recompute normals and tangents.
    bool bRecomputeNormals = buildData.buildOptions.bRecomputeNormals;
    bool bRecomputeTangents = buildData.buildOptions.bRecomputeTangents;

    // Dump normals and tangents if we are recomputing them.
    if (bRecomputeTangents) {
        std::vector<glm::vec3>& tangentX = buildData.tangentX;
        std::vector<glm::vec3>& tangentY = buildData.tangentY;

        tangentX.resize(numWedges, glm::vec3(0));
        tangentY.resize(numWedges, glm::vec3(0));
    }
    if (bRecomputeNormals) {
        std::vector<glm::vec3>& tangentZ = buildData.tangentZ;
        tangentZ.resize(numWedges, glm::vec3(0));
    }

    // Compute any missing tangents. MikkTSpace should be use only when the user want to recompute the normals or
    // tangents otherwise should always fallback on builtin tangent
    MeshHelper::ComputeTangents(skinnedMeshName, buildData, corners);

    // At this point the mesh will have valid tangents.
    buildData.ValidateTangentArraySize();
    ASSERT(overlappingCorners.size() == 1);
    return true;
}

bool MeshBuilder::BuildMesh(MeshAsset<>& model, const std::string& meshName, const std::vector<Importer::MeshImportData::VertexInfluence>& influences, const std::vector<Importer::MeshImportData::MeshWedge>& wedges, const std::vector<Importer::MeshImportData::MeshFace>& faces,
                            const std::vector<glm::vec3>& points, const std::vector<int>& pointToOriginalMap, std::shared_ptr<Importer::SceneInfo> sceneInfo, const MeshBuildSettings& buildOptions) {
    MeshBuildInputData buildData(model, influences, wedges, faces, points, pointToOriginalMap, sceneInfo, buildOptions);

    if (!PrepareSourceMesh(meshName, buildData, overlappingCorners)) {
        return false;
    }

    if (!GenerateRenderableMesh(buildData)) {
        return false;
    }

    // Build the skeletal model from chunks.
    // Builder.BeginSlowTask();
    BuildMeshModelFromChunks(buildData.model, buildData.chunks, buildData.pointToOriginalMap);
    // UpdateOverlappingVertices(buildData.model);
    // Builder.EndSlowTask();

    // Only show these warnings if in the game thread.  When importing morph targets, this function can run in another
    // thread and these warnings dont prevent the mesh from importing

    bool bHasBadSections = buildData.model.sections.size() != 0;
    for (int sectionIndex = 0; sectionIndex < buildData.model.sections.size(); sectionIndex++) {
        SubMeshAsset& section = buildData.model.sections[sectionIndex];
        bHasBadSections |= (section.numTriangles == 0);

        // Log info about the section.
        LOG_INFO(fmt::format("Section {:d} use material index {:d}, has {:d} triangles.", sectionIndex + 1, section.materialIndex, section.numTriangles));
    }
    if (bHasBadSections) {
        LOG_ERROR("Meah has section without any triangles.");
    }

    return true;
}

std::shared_ptr<Assets::MeshAsset> ConvertToMesh(std::shared_ptr<Mesh<>> mesh) {
    std::shared_ptr<Assets::MeshAsset> result = std::make_shared<Assets::MeshAsset>();
    result->name = mesh->name;
    result->uniqueID = std::to_string(mesh->fbxMesh->GetUniqueID());
    result->bHasNormals = mesh->bHasNormals;
    result->bHasTangents = mesh->bHasTangents;
    result->bHasVertexColors = mesh->bHasVertexColors;
    result->indexBuffer = mesh->asset->indexBuffer;
    result->indiceFormat = mesh->asset->indexBuffer.size() > 65535 ? Assets::EMeshIndiceFormat::Bit32 : Assets::EMeshIndiceFormat::Bit16;
    result->numTexCoords = mesh->numTexCoords;
    for (SubMeshAsset& rawSubMesh : mesh->asset->sections) {
        std::shared_ptr<Assets::MeshAsset::SubMesh> subMesh = std::make_shared<Assets::MeshAsset::SubMesh>();
        subMesh->baseIndex = rawSubMesh.baseIndex;
        for (auto& rawVetex : rawSubMesh.vertices) {
            std::shared_ptr<Assets::MeshAsset::Vertex> vertex = std::make_shared<Assets::MeshAsset::Vertex>();
            Types::ConvertFromGLM(vertex->position, rawVetex.position);
            if (mesh->bHasVertexColors) {
                Types::ConvertFromGLM(vertex->color, rawVetex.color);
            }
            if (mesh->bHasNormals) {
                Types::ConvertFromGLM(vertex->normal, rawVetex.tangentZ);
            }
            if (mesh->numTexCoords > 0) {
                for (int i = 0; i < mesh->numTexCoords; i++) {
                    vertex->uv.emplace_back(static_cast<float>(rawVetex.uvs[i].x), static_cast<float>(rawVetex.uvs[i].y));
                }
            }
            if (mesh->bHasTangents) {
                vertex->tangent.x = rawVetex.tangentX.x;
                vertex->tangent.y = rawVetex.tangentX.y;
                vertex->tangent.z = rawVetex.tangentX.z;
                vertex->tangent.w = 1.0;
            }
            subMesh->vertices.push_back(vertex);
        }
        subMesh->material = mesh->materials[rawSubMesh.materialIndex];
        subMesh->numTriangles = rawSubMesh.numTriangles;
        result->subMeshes.push_back(subMesh);
    }
    result->boundingSphere = std::make_shared<Assets::MeshAsset::BoundingSphere>();
    Types::ConvertFromGLM(result->boundingSphere->center, mesh->boundingSphere->origin);
    result->boundingSphere->radius = mesh->boundingSphere->sphereRadius;
    result->boundingBox = std::make_shared<Assets::MeshAsset::BoundingBox>();
    glm::vec3 center, extents;
    mesh->boundingBox->GetCenterAndExtents(center, extents);
    Types::ConvertFromGLM(result->boundingBox->center, center);
    Types::ConvertFromGLM(result->boundingBox->extents, extents);
    result->bForSkinnedMesh = false;
    return result;
}

}}  // namespace Fbx::Builder