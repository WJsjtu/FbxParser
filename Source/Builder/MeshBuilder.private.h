#pragma once
#include <map>
#include "Importer/Mesh.h"
#include "FbxParser.private.h"
#include "forsythtriangleorderoptimizer.h"
#include "mikktspace.h"
namespace Fbx { namespace Builder {

/**
 * Smoothing group interpretation helper structure.
 */
struct FanFace {
    int faceIndex;
    int linkedVertexIndex;
    bool bFilled;
    bool bBlendTangents;
    bool bBlendNormals;
};

int MikkGetNumFaces(const SMikkTSpaceContext* context);

int MikkGetNumVertsOfFace(const SMikkTSpaceContext* context, const int faceIdx);

void MikkGetPosition(const SMikkTSpaceContext* context, float position[3], const int faceIdx, const int VertIdx);

void MikkGetNormal(const SMikkTSpaceContext* context, float normal[3], const int faceIdx, const int vertIdx);

void MikkSetTSpaceBasic(const SMikkTSpaceContext* context, const float tangent[3], const float bitangentSign, const int faceIdx, const int vertIdx);

void MikkGetTexCoord(const SMikkTSpaceContext* context, float UV[2], const int faceIdx, const int vertIdx);

/** Helper struct for building acceleration structures. */
struct IndexAndZ {
    float z;
    int index;

    /** Default constructor. */
    IndexAndZ() {}

    /** Initialization constructor. */
    IndexAndZ(int inIndex, glm::vec3 v) {
        z = 0.30f * v.x + 0.33f * v.y + 0.37f * v.z;
        index = inIndex;
    }
};

// this is used for a sub-quadratic routine to find "equal" verts
struct VertIndexAndZ {
    int index;
    float z;
};

/** Sorting function for vertex Z/index pairs. */
struct CompareIndexAndZ {
    bool operator()(IndexAndZ const& a, IndexAndZ const& b) const { return a.z < b.z; }
};

/**
 * Returns true if the specified points are about equal
 */
bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, float comparisonThreshold);

/**
 * Returns true if the specified points are about equal
 */
bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, bool bUseEpsilonCompare = true);

bool UVsEqual(const glm::dvec2& v1, const glm::dvec2& v2, float epsilon);

bool NormalsEqual(const glm::vec3& v1, const glm::vec3& v2);
bool NormalsEqual(const glm::vec3& v1, const glm::vec3& v2);

glm::vec3 GetSafeNormal(const glm::vec3& vec, const float tolerance = SMALL_NUMBER);

void CreateOrthonormalBasis(glm::vec3& xAxis, glm::vec3& yAxis, glm::vec3& zAxis);

class ForsythHelper {
public:
    static void CacheOptimizeIndexBuffer(std::vector<uint32_t>& indices);
};

void CacheOptimizeIndexBuffer(std::vector<uint32_t>& indices);

namespace TriangleUtilities {

/*
 * This function compute the area of a triangle, it will return zero if the triangle is degenerated
 */
float ComputeTriangleArea(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC);

/*
 * This function compute the angle of a triangle corner, it will return zero if the triangle is degenerated
 */
float ComputeTriangleCornerAngle(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC);
// namespace TriangleUtilities
}  // namespace TriangleUtilities

class MeshBuildSettings;

bool PointsEqual(const glm::vec3& v1, const glm::vec3& v2, const MeshBuildSettings& overlappingThreshold);

bool UVsEqual(const glm::dvec2& v1, const glm::dvec2& v2, const MeshBuildSettings& overlappingThreshold);

bool NormalsEqual(const glm::vec3& v1, const glm::vec3& v2, const MeshBuildSettings& overlappingThreshold);

namespace MeshHelper {

void FindOverlappingCorners(std::shared_ptr<OverlappingCorners> outOverlappingCorners, IMeshBuildInputData& buildData, float comparisonThreshold);

void ComputeTriangleTangents(std::vector<glm::vec3>& triangleTangentX, std::vector<glm::vec3>& triangleTangentY, std::vector<glm::vec3>& triangleTangentZ, IMeshBuildInputData& buildData, float ComparisonThreshold);

// This function add every triangles connected to the triangle queue.
// A connected triangle pair must share at least 1 vertex between the two triangles.
// If bConnectByEdge is true, the connected triangle must share at least one edge (two vertex index)
void AddAdjacentFace(IMeshBuildInputData& buildData, std::vector<bool>& faceAdded, std::map<int, std::vector<int>>& vertexIndexToAdjacentFaces, int faceIndex, std::vector<int>& triangleQueue, const bool bConnectByEdge);

// Fill FaceIndexToPatchIndex so every triangle know is unique island patch index.
// We need to respect the island when we use the smooth group to compute the normals.
// Each island patch have its own smoothgroup data, there is no triangle connectivity possible between island patch.
//@Param bConnectByEdge: If true we need at least 2 vertex index (one edge) to connect 2 triangle. If false we just need
// one vertex index (bowtie)
void FillPolygonPatch(IMeshBuildInputData& buildData, std::vector<int>& faceIndexToPatchIndex, const bool bConnectByEdge);

bool IsTriangleMirror(IMeshBuildInputData& buildData, const std::vector<glm::vec3>& triangleTangentZ, const uint32_t faceIdxA, const uint32_t faceIdxB);

void ComputeTangents(const std::string& skinnedMeshName, IMeshBuildInputData& buildData, std::shared_ptr<OverlappingCorners> OverlappingCorners);
}  // namespace MeshHelper

bool AreMeshVerticesEqual(const SubMeshVertexWithWedgeIdx& v1, const SubMeshVertexWithWedgeIdx& v2, const MeshBuildSettings& buildOptions);

// Z轴排序网格点，按照material index分组，分组后去除重复节点，并构建index buffer。
template <class T>
void BuildMeshChunks(const std::vector<Importer::MeshImportData::MeshFace>& faces, const std::vector<SubMeshVertexWithWedgeIdx>& rawVertices, std::vector<VertIndexAndZ>& rawVertIndexAndZ, const MeshBuildSettings& buildOptions, std::vector<std::shared_ptr<T>>& outChunks) {
    static_assert(std::is_base_of<OriginSubMesh, T>::value, "Out chunks must derive from OriginSubMesh");
    std::vector<int> dupVerts;

    std::multimap<int, int> rawVerts2Dupes;
    {
        // Sorting function for vertex Z/index pairs
        struct CompareFSkinnedMeshVertIndexAndZ {
            bool operator()(const VertIndexAndZ& A, const VertIndexAndZ& B) const { return A.z < B.z; }
        };

        // Sort the vertices by z value
        std::sort(rawVertIndexAndZ.begin(), rawVertIndexAndZ.end(), CompareFSkinnedMeshVertIndexAndZ());

        // Search for duplicates, quickly!
        for (int i = 0; i < rawVertIndexAndZ.size(); i++) {
            // only need to search forward, since we add pairs both ways
            for (int j = i + 1; j < rawVertIndexAndZ.size(); j++) {
                if (Maths::Abs(rawVertIndexAndZ[j].z - rawVertIndexAndZ[i].z) > buildOptions.thresholdPosition) {
                    // our list is sorted, so there can't be any more dupes
                    break;
                }

                // check to see if the points are really overlapping
                if (PointsEqual(rawVertices[rawVertIndexAndZ[i].index].position, rawVertices[rawVertIndexAndZ[j].index].position, buildOptions)) {
                    rawVerts2Dupes.emplace(rawVertIndexAndZ[i].index, rawVertIndexAndZ[j].index);
                    rawVerts2Dupes.emplace(rawVertIndexAndZ[j].index, rawVertIndexAndZ[i].index);
                }
            }
        }
    }

    std::map<std::shared_ptr<T>, std::map<int, int>> chunkToFinalVerts;

    uint32_t triangleIndices[3];
    for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++) {
        const Importer::MeshImportData::MeshFace& Face = faces[faceIndex];

        // Find a chunk which matches this triangle.
        std::shared_ptr<T> chunk = nullptr;
        for (int i = 0; i < outChunks.size(); ++i) {
            if (outChunks[i]->materialIndex == Face.meshMaterialIndex) {
                chunk = outChunks[i];
                break;
            }
        }
        if (chunk == nullptr) {
            chunk = std::make_shared<T>();
            chunk->materialIndex = Face.meshMaterialIndex;
            chunk->originalSectionIndex = static_cast<int>(outChunks.size());
            outChunks.push_back(chunk);
        }

        std::map<int, int>& finalVerts = chunkToFinalVerts[chunk];

        for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            int wedgeIndex = faceIndex * 3 + vertexIndex;
            const SubMeshVertexWithWedgeIdx& vertex = rawVertices[wedgeIndex];

            int finalVertIndex = -1;
            dupVerts.clear();
            {
                auto Range = rawVerts2Dupes.equal_range(wedgeIndex);
                if (Range.first != rawVerts2Dupes.end()) {
                    for (auto Iter = Range.first; Iter != Range.second; ++Iter) {
                        dupVerts.push_back(Iter->second);
                    }
                }
            }
            std::sort(dupVerts.begin(), dupVerts.end());

            for (int k = 0; k < dupVerts.size(); k++) {
                if (dupVerts[k] >= wedgeIndex) {
                    // the verts beyond me haven't been placed yet, so these duplicates are not relevant
                    break;
                }

                auto location = finalVerts.find(dupVerts[k]);
                if (location != finalVerts.end()) {
                    if (AreMeshVerticesEqual(vertex, chunk->vertices[location->second], buildOptions)) {
                        finalVertIndex = location->second;
                        break;
                    }
                }
            }
            if (finalVertIndex == -1) {
                finalVertIndex = static_cast<int>(chunk->vertices.size());
                chunk->vertices.push_back(vertex);
                finalVerts.emplace(wedgeIndex, finalVertIndex);
            }

            // set the index entry for the newly added vertex
            // TArray internally has int32 for capacity, so no need to test for uint32 as it's larger than int32
            triangleIndices[vertexIndex] = static_cast<uint32_t>(finalVertIndex);
        }

        if (triangleIndices[0] != triangleIndices[1] && triangleIndices[0] != triangleIndices[2] && triangleIndices[1] != triangleIndices[2]) {
            for (uint32_t VertexIndex = 0; VertexIndex < 3; VertexIndex++) {
                chunk->indices.push_back(triangleIndices[VertexIndex]);
            }
        }
    }
}

bool PrepareSourceMesh(const std::string& skinnedMeshName, IMeshBuildInputData& buildData, std::vector<std::shared_ptr<OverlappingCorners>>& overlappingCorners);

}}  // namespace Fbx::Builder