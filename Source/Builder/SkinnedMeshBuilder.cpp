#include "SkinnedMeshBuilder.h"
#include "MeshBuilder.private.h"
#include "Importer/Scene.h"
#include "FbxParser.private.h"

namespace Fbx { namespace Builder {

SkinnedSubMeshAsset::SkinnedSubMeshAsset() : SubMeshAsset(), maxBoneInfluences(4) {}

/**
 * Calculate max # of bone influences used by this skel mesh chunk
 */
void SkinnedSubMeshAsset::CalcMaxBoneInfluences() {
    // if we only have rigid verts then there is only one bone
    maxBoneInfluences = 1;
    // iterate over all the soft vertices for this chunk and find max # of bones used
    for (int vertIdx = 0; vertIdx < vertices.size(); vertIdx++) {
        SubMeshVertex& softVert = vertices[vertIdx];

        // calc # of bones used by this soft skinned vertex
        int bonesUsed = 0;
        for (int influenceIdx = 0; influenceIdx < MAX_TOTAL_INFLUENCES; influenceIdx++) {
            if (softVert.influenceWeights[influenceIdx] > 0) {
                bonesUsed++;
            }
        }
        // reorder bones so that there aren't any unused influence entries within the [0,BonesUsed] range
        for (int influenceIdx = 0; influenceIdx < bonesUsed; influenceIdx++) {
            if (softVert.influenceWeights[influenceIdx] == 0) {
                for (int exchangeIdx = influenceIdx + 1; exchangeIdx < MAX_TOTAL_INFLUENCES; exchangeIdx++) {
                    if (softVert.influenceWeights[exchangeIdx] != 0) {
                        float TempWeight = softVert.influenceWeights[influenceIdx];
                        softVert.influenceWeights[influenceIdx] = softVert.influenceWeights[exchangeIdx];
                        softVert.influenceWeights[exchangeIdx] = TempWeight;

                        uint16_t TempIndex = softVert.influenceBones[influenceIdx];
                        softVert.influenceBones[influenceIdx] = softVert.influenceBones[exchangeIdx];
                        softVert.influenceBones[exchangeIdx] = TempIndex;
                        break;
                    }
                }
            }
        }

        // maintain max bones used
        maxBoneInfluences = Maths::Max(maxBoneInfluences, bonesUsed);
    }
}

SkinnedMeshBuildInputData::SkinnedMeshBuildInputData(SkinnedMeshAsset& inModel, const SkinnedMeshSkeleton& inSkeleton, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences, const std::vector<Importer::MeshImportData::MeshWedge>& inWedges,
                                                     const std::vector<Importer::MeshImportData::MeshFace>& inFaces, const std::vector<glm::vec3>& inPoints, const std::vector<int>& inPointToOriginalMap, const MeshBuildSettings& inBuildOptions)
    : IMeshBuildInputData(inWedges, inFaces, inPoints, inInfluences, inPointToOriginalMap, inBuildOptions), model(inModel), skeleton(inSkeleton) {}

bool SkinnedMeshBuilder::Build(std::shared_ptr<SkinnedMesh> skinnedMesh, std::shared_ptr<Importer::MeshImportData> meshImportData, const MeshBuildSettings& options) {
    const SkinnedMeshSkeleton& skeleton = skinnedMesh->skeleton;

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
        for (int i = 0; i < MAX_TEXCOORDS; i++) {
            wedges[w].UVs[i] = meshImportData->wedges[w].UVs[i];
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

    std::shared_ptr<SkinnedMeshAsset> model = std::make_shared<SkinnedMeshAsset>();

    BuildSkinnedMesh(*model, skinnedMesh->name, skeleton, influences, wedges, faces, points, pointToRawMap, options);

    skinnedMesh->asset = model;

    return true;
}

// Fill FaceIndexToPatchIndex so every triangle knows its unique island patch index.
// Each island patch have is fill with connected vertexinstance where position, NTBs. UVs and colors are nearly equal.
//@Param bConnectByEdge: If true we need at least 2 vertex index (one edge) to connect 2 triangles. If false we just
// need one vertex index (bowtie)
void PolygonShellsHelper::FillPolygonPatch(const std::vector<uint32_t>& indices, const std::vector<SubMeshVertexWithWedgeIdx>& vertices, const std::map<uint32_t, std::vector<uint16_t>>& alternateBoneIDs, std::vector<PatchAndBoneInfluence>& patchData,
                                           std::vector<std::vector<uint32_t>>& patchIndexToIndices, std::map<int, std::vector<uint16_t>>& bonesPerFace, const int maxBonesPerChunk, const bool bConnectByEdge) {
    const int numIndice = static_cast<int>(indices.size());
    const int numFace = numIndice / 3;
    int patchIndex = 0;

    // Store a map containing connected faces for each vertex index
    std::map<int, std::vector<int>> vertexIndexToAdjacentFaces;
    // Store a map to retrieve bones use per face
    for (int faceIndex = 0; faceIndex < numFace; ++faceIndex) {
        const int indiceOffset = faceIndex * 3;
        std::vector<uint16_t>& faceInfluenceBones = bonesPerFace[faceIndex];
        for (int corner = 0; corner < 3; corner++) {
            const int indiceIndex = indiceOffset + corner;
            ASSERT(indices.size() > indiceIndex && indiceIndex >= 0);
            int vertexIndex = indices[indiceIndex];
            std::vector<int>& adjacentFaces = vertexIndexToAdjacentFaces[vertexIndex];
            if (std::find(adjacentFaces.begin(), adjacentFaces.end(), faceIndex) == adjacentFaces.end()) {
                adjacentFaces.push_back(faceIndex);
            }
            const SubMeshVertexWithWedgeIdx& softSkinVertex = vertices[vertexIndex];
            for (int boneIndex = 0; boneIndex < MAX_TOTAL_INFLUENCES; ++boneIndex) {
                if (softSkinVertex.influenceWeights[boneIndex] > 0) {
                    if (std::find(faceInfluenceBones.begin(), faceInfluenceBones.end(), softSkinVertex.influenceBones[boneIndex]) == faceInfluenceBones.end()) {
                        faceInfluenceBones.push_back(softSkinVertex.influenceBones[boneIndex]);
                    }
                }
            }
            // Add the alternate bones
            const auto& alternateBones = alternateBoneIDs.find(softSkinVertex.pointWedgeIdx);
            if (alternateBones != alternateBoneIDs.end()) {
                for (int influenceIndex = 0; influenceIndex < alternateBones->second.size(); influenceIndex++) {
                    if (std::find(faceInfluenceBones.begin(), faceInfluenceBones.end(), alternateBones->second[influenceIndex]) == faceInfluenceBones.end()) {
                        faceInfluenceBones.push_back(alternateBones->second[influenceIndex]);
                    }
                }
            }
        }
    }

    // Mark added face so we do not add them more then once
    std::vector<bool> faceAdded(numFace, false);

    std::vector<int> triangleQueue;
    // Allocate an array and use it to retrieve the data, we do not know the number of indices per patch so it prevent
    // us doing a huge reserve per patch Simply copy the result in PatchIndexToIndices when we finish gathering the
    // patch data.
    std::vector<uint32_t> allocatedPatchIndexToIndices;
    for (int faceIndex = 0; faceIndex < numFace; ++faceIndex) {
        // Skip already added faces
        if (faceAdded[faceIndex]) {
            continue;
        }
        allocatedPatchIndexToIndices.clear();

        // Add all the faces connected to the current face index
        triangleQueue.clear();
        triangleQueue.push_back(faceIndex);  // Use a queue to avoid recursive function
        faceAdded[faceIndex] = true;
        while (triangleQueue.size() > 0) {
            int currentTriangleIndex = triangleQueue.back();
            triangleQueue.pop_back();
            std::vector<uint16_t> BonesToAdd = bonesPerFace[currentTriangleIndex];
            for (const uint16_t boneIndex : BonesToAdd) {
                if (!(patchData.size() > patchIndex && patchIndex >= 0)) {
                    size_t FillSize = patchData.size() - patchIndex + 1;
                    for (int ii = 0; ii < FillSize; ii++) {
                        patchData.push_back(PatchAndBoneInfluence());
                    }
                }
                if (std::find(patchData[patchIndex].uniqueBones.begin(), patchData[patchIndex].uniqueBones.end(), boneIndex) == patchData[patchIndex].uniqueBones.end()) {
                    patchData[patchIndex].uniqueBones.push_back(boneIndex);
                }
            }
            int indiceOffset = currentTriangleIndex * 3;
            for (int corner = 0; corner < 3; corner++) {
                const int indiceIndex = indiceOffset + corner;
                allocatedPatchIndexToIndices.push_back(indiceIndex);
            }
            // The patch should exist at this time
            ASSERT(patchData.size() > patchIndex && patchIndex >= 0);

            AddAdjacentFace(indices, vertices, faceAdded, vertexIndexToAdjacentFaces, currentTriangleIndex, triangleQueue, bConnectByEdge);
        }

        // This is a new patch create the data and append the patch result remap
        ASSERT(!(patchIndexToIndices.size() > patchIndex && patchIndex >= 0));
        patchIndexToIndices.push_back(std::vector<uint32_t>());
        ASSERT(patchIndexToIndices.size() > patchIndex && patchIndex >= 0);
        for (const auto& Item : allocatedPatchIndexToIndices) {
            patchIndexToIndices[patchIndex].push_back(Item);
        }
        patchIndex++;
    }
}

// This function add every triangles connected to the triangle queue.
// A connected triangle pair must share at least 1 vertex between the two triangles.
// If bConnectByEdge is true, the connected triangle must share at least one edge (two vertex index)
// To have a connected vertex instance pair, the position, NTBs, UVs(channel 0) and color must match.
void PolygonShellsHelper::AddAdjacentFace(const std::vector<uint32_t>& indices, const std::vector<SubMeshVertexWithWedgeIdx>& vertices, std::vector<bool>& faceAdded, const std::map<int, std::vector<int>>& vertexIndexToAdjacentFaces, const int FaceIndex, std::vector<int>& triangleQueue,
                                          const bool bConnectByEdge) {
    int numFaces = static_cast<int>(indices.size() / 3);
    ASSERT(faceAdded.size() == numFaces);

    std::map<int, int> adjacentFaceCommonVertices;
    for (int corner = 0; corner < 3; corner++) {
        int indiceIndex = FaceIndex * 3 + corner;
        ASSERT(indices.size() > indiceIndex && indiceIndex >= 0);
        int vertexIndex = indices[indiceIndex];
        ASSERT(vertices.size() > vertexIndex && vertexIndex >= 0);
        const SubMeshVertexWithWedgeIdx& softSkinVertRef = vertices[vertexIndex];
        const glm::vec3& positionRef = softSkinVertRef.position;
        const glm::vec3& tangentXRef = softSkinVertRef.tangentX;
        const glm::vec3& tangentYRef = softSkinVertRef.tangentY;
        const glm::vec3& tangentZRef = softSkinVertRef.tangentZ;
        const glm::dvec2& UVRef = softSkinVertRef.UVs[0];
        const glm::vec4& colorRef = softSkinVertRef.color;
        ASSERT(vertexIndexToAdjacentFaces.find(vertexIndex) != vertexIndexToAdjacentFaces.end());
        const std::vector<int>& adjacentFaces = vertexIndexToAdjacentFaces.find(vertexIndex)->second;
        for (int adjacentFaceArrayIndex = 0; adjacentFaceArrayIndex < adjacentFaces.size(); ++adjacentFaceArrayIndex) {
            const int adjacentFaceIndex = adjacentFaces[adjacentFaceArrayIndex];
            if (!faceAdded[adjacentFaceIndex] && adjacentFaceIndex != FaceIndex) {
                // Ensure we have position, NTBs, uv and color match to allow a connection.
                bool bRealConnection = false;
                for (int adjCorner = 0; adjCorner < 3; adjCorner++) {
                    const int indiceIndexAdj = adjacentFaceIndex * 3 + adjCorner;
                    ASSERT(indices.size() > indiceIndexAdj && indiceIndexAdj >= 0);
                    const int vertexIndexAdj = indices[indiceIndexAdj];
                    ASSERT(vertices.size() > vertexIndexAdj && vertexIndexAdj >= 0);
                    const SubMeshVertexWithWedgeIdx& softSkinVertAdj = vertices[vertexIndexAdj];

                    if (PointsEqual(positionRef, softSkinVertAdj.position, SMALL_NUMBER) && PointsEqual(tangentXRef, softSkinVertAdj.tangentX, SMALL_NUMBER) && PointsEqual(tangentYRef, softSkinVertAdj.tangentY, SMALL_NUMBER) && PointsEqual(tangentZRef, softSkinVertAdj.tangentZ, SMALL_NUMBER) &&
                        UVsEqual(UVRef, softSkinVertAdj.UVs[0], KINDA_SMALL_NUMBER) && colorRef == softSkinVertAdj.color) {
                        bRealConnection = true;
                        break;
                    }
                }
                if (!bRealConnection) {
                    continue;
                }

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

// Sort the shells to a setup that use the less section possible
void PolygonShellsHelper::GatherShellUsingSameBones(const int parentPatchIndex, std::vector<PatchAndBoneInfluence>& patchData, std::vector<bool>& patchConsumed, const int maxBonesPerChunk) {
    ASSERT(patchData.size() > parentPatchIndex && parentPatchIndex >= 0);
    std::vector<uint16_t> UniqueBones = patchData[parentPatchIndex].uniqueBones;
    patchData[parentPatchIndex].bIsParent = true;
    if (UniqueBones.size() > maxBonesPerChunk) {
        return;
    }
    for (int PatchIndex = parentPatchIndex + 1; PatchIndex < patchData.size(); ++PatchIndex) {
        if (patchConsumed[PatchIndex]) {
            continue;
        }

        std::vector<uint16_t> AddedBones;
        for (int BoneIndex = 0; BoneIndex < patchData[PatchIndex].uniqueBones.size(); ++BoneIndex) {
            uint16_t BoneIndexType = patchData[PatchIndex].uniqueBones[BoneIndex];
            if (std::find(UniqueBones.begin(), UniqueBones.end(), BoneIndexType) == UniqueBones.end()) {
                AddedBones.push_back(BoneIndexType);
            }
        }
        if (AddedBones.size() + UniqueBones.size() <= maxBonesPerChunk) {
            for (const auto& Item : AddedBones) {
                UniqueBones.push_back(Item);
            }
            patchConsumed[PatchIndex] = true;
            // We only support one parent layer, the assumption is we have a hierarchy depth of max 2 (parents, childs)
            ASSERT(!patchData[PatchIndex].bIsParent);
            patchData[parentPatchIndex].patchToChunkWith.push_back(PatchIndex);
        }
    }
}

void PolygonShellsHelper::RecursiveFillRemapIndices(const std::vector<PatchAndBoneInfluence>& patchData, const int patchIndex, const std::vector<std::vector<uint32_t>>& patchIndexToIndices, std::vector<uint32_t>& srcChunkRemapIndicesIndex) {
    for (const auto& item : patchIndexToIndices[patchIndex]) {
        srcChunkRemapIndicesIndex.push_back(item);
    }
    ASSERT(patchData.size() > patchIndex && patchIndex >= 0);
    // Do the child patch to chunk with
    for (int SubPatchIndex = 0; SubPatchIndex < patchData[patchIndex].patchToChunkWith.size(); ++SubPatchIndex) {
        RecursiveFillRemapIndices(patchData, patchData[patchIndex].patchToChunkWith[SubPatchIndex], patchIndexToIndices, srcChunkRemapIndicesIndex);
    }
}

void ChunkSkinnedVertices(std::vector<std::shared_ptr<SkinnedSubMesh>>& chunks, std::map<uint32_t, std::vector<uint16_t>>& alternateBoneIDs, int maxBonesPerChunk) {
    // Copy over the old chunks (this is just copying pointers).
    std::vector<std::shared_ptr<SkinnedSubMesh>> srcChunks = chunks;
    chunks.clear();

    // Sort the chunks by material index.
    struct CompareSkinnedMeshChunk {
        bool operator()(const std::shared_ptr<SkinnedSubMesh>& A, const std::shared_ptr<SkinnedSubMesh>& B) const { return A->materialIndex < B->materialIndex; }
    };
    std::sort(srcChunks.begin(), srcChunks.end(), CompareSkinnedMeshChunk());

    std::map<int, std::vector<PolygonShellsHelper::PatchAndBoneInfluence>> patchDataPerSrcChunk;
    std::map<int, std::vector<std::vector<uint32_t>>> patchIndexToIndicesPerSrcChunk;
    std::map<int, std::map<int, std::vector<uint16_t>>> patchIndexToBonesPerFace;

    //  对每个chunk（per material index）拆分连通域。
    // Find the shells inside chunks
    for (int chunkIndex = 0; chunkIndex < srcChunks.size(); ++chunkIndex) {
        std::shared_ptr<SkinnedSubMesh> chunkToShell = srcChunks[chunkIndex];
        std::vector<uint32_t>& indices = chunkToShell->indices;
        std::vector<SubMeshVertexWithWedgeIdx>& vertices = chunkToShell->vertices;
        std::vector<PolygonShellsHelper::PatchAndBoneInfluence>& patchData = patchDataPerSrcChunk[chunkIndex];
        std::vector<std::vector<uint32_t>>& patchIndexToIndices = patchIndexToIndicesPerSrcChunk[chunkIndex];
        std::map<int, std::vector<uint16_t>>& bonesPerFace = patchIndexToBonesPerFace[chunkIndex];
        // We need edge connection (2 similar vertex )
        const bool bConnectByEdge = true;
        PolygonShellsHelper::FillPolygonPatch(indices, vertices, alternateBoneIDs, patchData, patchIndexToIndices, bonesPerFace, maxBonesPerChunk, bConnectByEdge);
    }

    //  对每个chunk（per material index）对已拆分的连通域进行骨骼贪心合并分析。
    for (int srcChunkIndex = 0; srcChunkIndex < srcChunks.size(); ++srcChunkIndex) {
        std::vector<PolygonShellsHelper::PatchAndBoneInfluence>& patchData = patchDataPerSrcChunk[srcChunkIndex];
        std::vector<bool> patchConsumed(patchData.size(), false);

        for (int patchIndex = 0; patchIndex < patchData.size(); ++patchIndex) {
            if (patchConsumed[patchIndex]) {
                continue;
            }
            patchConsumed[patchIndex] = true;
            PolygonShellsHelper::GatherShellUsingSameBones(patchIndex, patchData, patchConsumed, maxBonesPerChunk);
        }
    }

    // Now split chunks to respect the desired bone limit.
    std::vector<std::vector<int>> indexMaps;
    for (int srcChunkIndex = 0; srcChunkIndex < srcChunks.size(); ++srcChunkIndex) {
        std::shared_ptr<SkinnedSubMesh> srcChunk = srcChunks[srcChunkIndex];
        srcChunk->originalSectionIndex = srcChunkIndex;
        int firstChunkIndex = static_cast<int>(chunks.size());
        // Iterate Indice in the order of the shell patch
        std::vector<uint32_t> srcChunkRemapIndicesIndex;
        std::vector<PolygonShellsHelper::PatchAndBoneInfluence>& patchData = patchDataPerSrcChunk[srcChunkIndex];
        const std::vector<std::vector<uint32_t>>& patchIndexToIndices = patchIndexToIndicesPerSrcChunk[srcChunkIndex];
        const std::map<int, std::vector<uint16_t>>& nonesPerFace = patchIndexToBonesPerFace[srcChunkIndex];

        for (int patchIndex = 0; patchIndex < patchData.size(); ++patchIndex) {
            if (!patchData[patchIndex].bIsParent) {
                continue;
            }
            srcChunkRemapIndicesIndex.clear();
            PolygonShellsHelper::RecursiveFillRemapIndices(patchData, patchIndex, patchIndexToIndices, srcChunkRemapIndicesIndex);

            // Force adding a chunk since we want to control where we cut the model
            int lastCreatedChunkIndex = firstChunkIndex;
            const int patchInitialChunkIndex = static_cast<int>(chunks.size());

            auto CreateChunk = [&srcChunk, &firstChunkIndex, &lastCreatedChunkIndex, &chunks, &indexMaps](std::shared_ptr<SkinnedSubMesh>& destinationChunk) {
                destinationChunk = std::make_shared<SkinnedSubMesh>();
                lastCreatedChunkIndex = static_cast<int>(chunks.size());
                chunks.push_back(destinationChunk);
                destinationChunk->materialIndex = srcChunk->materialIndex;
                destinationChunk->originalSectionIndex = srcChunk->originalSectionIndex;
                destinationChunk->parentChunkSectionIndex = lastCreatedChunkIndex == firstChunkIndex ? -1 : firstChunkIndex;
                indexMaps.push_back(std::vector<int>());
                std::vector<int>& indexMap = indexMaps.back();
                indexMap.resize(srcChunk->vertices.size(), -1);
            };

            // Create a chunk
            {
                std::shared_ptr<SkinnedSubMesh> destChunk = nullptr;
                CreateChunk(destChunk);
            }

            // Add Indices to the chunk and add extra chunk only in case the patch use more bone then the maximum
            // specified
            for (int i = 0; i < srcChunkRemapIndicesIndex.size(); i += 3) {
                // We remap the iteration order to avoid cutting polygon shell
                int indiceIndex = srcChunkRemapIndicesIndex[i];
                // Find all bones needed by this triangle.
                const int FaceIndex = (indiceIndex / 3);
                ASSERT(nonesPerFace.find(FaceIndex) != nonesPerFace.end());
                const std::vector<uint16_t>& uniqueBones = nonesPerFace.find(FaceIndex)->second;

                // Now find a chunk for them.
                std::shared_ptr<SkinnedSubMesh> destChunk = nullptr;
                int destChunkIndex = /* bUseExperimentalChunking ? PatchInitialChunkIndex : */ lastCreatedChunkIndex;
                int smallestNumBoneToAdd = std::numeric_limits<int>::max();
                for (int chunkIndex = destChunkIndex; chunkIndex < chunks.size(); ++chunkIndex) {
                    const std::vector<uint16_t>& boneMap = chunks[chunkIndex]->boneMap;
                    int numUniqueBones = 0;
                    for (int j = 0; j < uniqueBones.size(); ++j) {
                        numUniqueBones += ((std::find(boneMap.begin(), boneMap.end(), uniqueBones[j]) != boneMap.end()) ? 0 : 1);
                        if (numUniqueBones == smallestNumBoneToAdd) {
                            // Another previous chunk use less or equal unique bone, avoid searching more
                            break;
                        }
                    }
                    if (numUniqueBones + boneMap.size() <= maxBonesPerChunk && numUniqueBones < smallestNumBoneToAdd) {
                        // Add the vertex to the chunk that can contain it with the less addition.
                        smallestNumBoneToAdd = numUniqueBones;
                        destChunkIndex = chunkIndex;
                        destChunk = chunks[chunkIndex];
                        if (smallestNumBoneToAdd == 0) {
                            // This is the best candidate
                            break;
                        }
                    }
                }

                // If no chunk was found, create one!
                if (destChunk == nullptr) {
                    CreateChunk(destChunk);
                    // Set back the DestChunkIndex. CreateChunk set the LastCreatedChunkIndex, so we need to update
                    // DestChunkIndex to pick The right IndexMaps that match the new chunk.
                    destChunkIndex = lastCreatedChunkIndex;
                }
                std::vector<int>& indexMap = indexMaps[destChunkIndex];

                // Add the unique bones to this chunk's bone map.
                for (int j = 0; j < uniqueBones.size(); ++j) {
                    if (std::find(destChunk->boneMap.begin(), destChunk->boneMap.end(), uniqueBones[j]) == destChunk->boneMap.end()) {
                        destChunk->boneMap.push_back(uniqueBones[j]);
                    }
                }

                // For each vertex, add it to the chunk's arrays of vertices and indices.
                for (int corner = 0; corner < 3; corner++) {
                    int vertexIndex = srcChunk->indices[indiceIndex + corner];
                    int destIndex = indexMap[vertexIndex];
                    if (destIndex == -1) {
                        destIndex = static_cast<int>(destChunk->vertices.size());
                        destChunk->vertices.push_back(srcChunk->vertices[vertexIndex]);
                        SubMeshVertexWithWedgeIdx& v = destChunk->vertices[destIndex];
                        for (int influenceIndex = 0; influenceIndex < MAX_TOTAL_INFLUENCES; influenceIndex++) {
                            if (v.influenceWeights[influenceIndex] > 0) {
                                auto iter = std::find(destChunk->boneMap.begin(), destChunk->boneMap.end(), v.influenceBones[influenceIndex]);
                                int mappedIndex = iter == destChunk->boneMap.end() ? -1 : static_cast<int>(iter - destChunk->boneMap.begin());
                                ASSERT(destChunk->boneMap.size() > mappedIndex && mappedIndex >= 0);
                                v.influenceBones[influenceIndex] = mappedIndex;
                            }
                        }
                        indexMap[vertexIndex] = destIndex;
                    }
                    destChunk->indices.push_back(destIndex);
                }
            }
        }

        // Source chunks are no longer needed.
        srcChunks[srcChunkIndex] = nullptr;
        // 至此，每个dest
        // chunk中：顶点信息除了bone已完成记录在Vertices，index信息记录在indices，BoneMap记录了用到的骨骼。
        // 外部的IndexMap记录了每个chunk中的顶点 => 新index的索引
    }
}

bool SkinnedMeshBuilder::GenerateRenderableSkinnedMesh(SkinnedMeshBuildInputData& buildData) {
    // Find wedge influences.
    std::vector<int> wedgeInfluenceIndices;

    // Influences中第一次出现VertIndex的位置（InfluenceIndex）
    std::map<uint32_t, uint32_t> vertexIndexToInfluenceIndexMap;

    for (uint32_t LookIdx = 0; LookIdx < static_cast<uint32_t>(buildData.influences.size()); LookIdx++) {
        // Order matters do not allow the map to overwrite an existing value.
        if (vertexIndexToInfluenceIndexMap.find(buildData.influences[LookIdx].vertexIndex) == vertexIndexToInfluenceIndexMap.end()) {
            vertexIndexToInfluenceIndexMap.emplace(buildData.influences[LookIdx].vertexIndex, LookIdx);
        }
    }

    for (int wedgeIndex = 0; wedgeIndex < buildData.wedges.size(); wedgeIndex++) {
        auto InflunenceFound = vertexIndexToInfluenceIndexMap.find(buildData.wedges[wedgeIndex].iVertex);

        if (InflunenceFound != vertexIndexToInfluenceIndexMap.end()) {
            wedgeInfluenceIndices.push_back(InflunenceFound->second);
        } else {
            // we have missing influence vert, we weight to root
            wedgeInfluenceIndices.push_back(-1);
            LOG_WARN("顶点" + std::to_string(buildData.wedges[wedgeIndex].iVertex) + "缺少蒙皮权重信息，锁定到根骨骼。");
        }
    }

    ASSERT(buildData.wedges.size() == wedgeInfluenceIndices.size());

    std::vector<VertIndexAndZ> vertIndexAndZ;
    std::vector<SubMeshVertexWithWedgeIdx> rawVertices;

    for (int faceIndex = 0; faceIndex < buildData.faces.size(); faceIndex++) {
        // Only update the status progress bar if we are in the game thread and every thousand faces.
        // Updating status is extremely slow
        if (faceIndex % 5000 == 0) {
            LOG_INFO("正在处理" + std::to_string(faceIndex) + "/" + std::to_string(buildData.faces.size()) + "个三角面。");
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

            for (int ii = 0; ii < MAX_TEXCOORDS; ii++) {
                vertex.UVs[ii] = wedge.UVs[ii];
            }
            vertex.color = wedge.color;

            {
                // Count the influences.
                int infIdx = wedgeInfluenceIndices[Face.iWedge[vertexIndex]];
                if (infIdx == -1) {
                    for (uint32_t i = 0; i < MAX_TOTAL_INFLUENCES; i++) {
                        vertex.influenceBones[i] = 0;
                        vertex.influenceWeights[i] = 0;
                    }
                } else {
                    int lookIdx = infIdx;

                    uint32_t influenceCount = 0;
                    while (buildData.influences.size() > lookIdx && lookIdx >= 0 && (buildData.influences[lookIdx].vertexIndex == wedge.iVertex)) {
                        influenceCount++;
                        lookIdx++;
                    }

                    // Setup the vertex influences.
                    vertex.influenceBones[0] = 0;
                    vertex.influenceWeights[0] = 1.0f;
                    for (uint32_t i = 1; i < MAX_TOTAL_INFLUENCES; i++) {
                        vertex.influenceBones[i] = 0;
                        vertex.influenceWeights[i] = 0.0f;
                    }

                    for (uint32_t i = 0; i < influenceCount; i++) {
                        uint16_t BoneIndex = buildData.influences[infIdx + i].boneIndex;
                        if (BoneIndex >= buildData.skeleton.GetBoneNum()) continue;
                        vertex.influenceBones[i] = BoneIndex;
                        vertex.influenceWeights[i] = buildData.influences[infIdx + i].weight;
                    }

                    influenceCount = Maths::Min<uint32_t>(influenceCount, MAX_TOTAL_INFLUENCES);
                    if (influenceCount > EXTRA_BONE_INFLUENCES) {
                        int max = EXTRA_BONE_INFLUENCES;
                        LOG_WARN("蒙皮顶点" + std::to_string(wedge.iVertex) + "包含" + std::to_string(influenceCount) + "个骨骼权重信息，截断到支持上限" + std::to_string(max) + "。");
                        influenceCount = EXTRA_BONE_INFLUENCES;
                    }

                    float totalInfluenceWeight = 0;
                    for (uint32_t i = 0; i < influenceCount; i++) {
                        uint16_t BoneIndex = buildData.influences[infIdx + i].boneIndex;
                        totalInfluenceWeight += vertex.influenceWeights[i];
                    }

                    vertex.influenceWeights[0] += 1.0f - totalInfluenceWeight;
                }
            }

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

    // Get alternate skinning weights map to retrieve easily the data
    std::map<uint32_t, std::vector<uint16_t>> alternateBoneIDs;

    // Chunk vertices to satisfy the requested limit.
    const uint32_t maxGPUSkinBones = Importer::Importer::GetInstance()->options->maxBones;
    ChunkSkinnedVertices(buildData.chunks, alternateBoneIDs, maxGPUSkinBones);
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
void BuildSkinnedMeshModelFromChunks(SkinnedMeshAsset& model, const SkinnedMeshSkeleton& skeleton, std::vector<std::shared_ptr<SkinnedSubMesh>>& chunks, const std::vector<int>& pointToOriginalMap) {
    // Clear out any data currently held in the LOD model.
    model.sections.clear();
    model.numVertices = 0;
    model.indexBuffer.clear();

    // Setup the section and chunk arrays on the model.
    for (int chunkIndex = 0; chunkIndex < chunks.size(); ++chunkIndex) {
        std::shared_ptr<SkinnedSubMesh> srcChunk = chunks[chunkIndex];
        model.sections.push_back(SkinnedSubMeshAsset());
        SkinnedSubMeshAsset& section = model.sections.back();
        section.materialIndex = srcChunk->materialIndex;
        section.boneMap = std::move(srcChunk->boneMap);
        srcChunk->boneMap.clear();

        section.originalDataSectionIndex = srcChunk->originalSectionIndex;
        section.chunkedParentSectionIndex = srcChunk->parentChunkSectionIndex;

        // Update the active bone indices on the LOD model.
        for (int boneIndex = 0; boneIndex < section.boneMap.size(); ++boneIndex) {
            if (std::find(model.activeBoneIndices.begin(), model.activeBoneIndices.end(), section.boneMap[boneIndex]) == model.activeBoneIndices.end()) {
                model.activeBoneIndices.push_back(section.boneMap[boneIndex]);
            }
        }
    }

    // ensure parent exists with incoming active bone indices, and the result should be sorted
    skeleton.EnsureParentsExistAndSort(model.activeBoneIndices);

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
        LOG_INFO("正在处理网格分块" + std::to_string(sectionIndex) + "/" + std::to_string(model.sections.size()) + "。");

        std::shared_ptr<SkinnedSubMesh> srcChunk = chunks[sectionIndex];
        SkinnedSubMeshAsset& section = model.sections[sectionIndex];
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
                // No new index has been allocated for this existing index, assign a new one
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
        SkinnedSubMeshAsset& section = model.sections[sectionIndex];
        std::vector<SubMeshVertexWithWedgeIdx>& chunkVertices = chunks[sectionIndex]->vertices;
        LOG_INFO("正在处理分块。");

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
            for (int ii = 0; ii < MAX_TEXCOORDS; ii++) {
                newVertex.UVs[ii] = softVertex.UVs[ii];
            }
            newVertex.color = softVertex.color;
            for (int i = 0; i < MAX_TOTAL_INFLUENCES; ++i) {
                // it only adds to the bone map if it has weight on it
                // BoneMap contains only the bones that has influence with weight of >0.f
                // so here, just make sure it is included before setting the data
                if (section.boneMap.size() > softVertex.influenceBones[i] && softVertex.influenceBones[i] >= 0) {
                    newVertex.influenceBones[i] = softVertex.influenceBones[i];
                    newVertex.influenceWeights[i] = softVertex.influenceWeights[i];
                }
            }
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

        // update max bone influences
        section.CalcMaxBoneInfluences();

        // Log info about the chunk.
        LOG_INFO("分块" + std::to_string(sectionIndex) + "：" + std::to_string(section.numVertices) + "个顶点，" + std::to_string(section.boneMap.size()) + "根骨骼。");
    }

    // Copy raw point indices to LOD model.
    model.rawPointIndices.clear();
    if (rawPointIndices.size()) {
        model.rawPointIndices = rawPointIndices;
    }

    // Finish building the sections.
    for (int sectionIndex = 0; sectionIndex < model.sections.size(); sectionIndex++) {
        SkinnedSubMeshAsset& section = model.sections[sectionIndex];

        const std::vector<uint32_t>& sectionIndices = chunks[sectionIndex]->indices;

        section.baseIndex = static_cast<uint32_t>(model.indexBuffer.size());
        const int numIndices = static_cast<int>(sectionIndices.size());
        const std::vector<uint32_t>& sectionVertexIndexRemap = vertexIndexRemap[sectionIndex];
        for (int index = 0; index < numIndices; index++) {
            uint32_t VertexIndex = sectionVertexIndexRemap[sectionIndices[index]];
            model.indexBuffer.push_back(VertexIndex);
        }
    }

    // Free the skinned mesh chunks which are no longer needed.
    for (int i = 0; i < chunks.size(); ++i) {
        chunks[i] = nullptr;
    }
    chunks.clear();

    // Compute the required bones for this model.
    {
        // RequiredBones for base model includes all raw bones.
        int requiredBoneCount = skeleton.GetBoneNum();
        model.requiredBones.clear();
        for (int i = 0; i < requiredBoneCount; i++) {
            model.requiredBones.push_back(i);
        }
    }
}

bool SkinnedMeshBuilder::BuildSkinnedMesh(SkinnedMeshAsset& model, const std::string& skinnedMeshName, const SkinnedMeshSkeleton& skeleton, const std::vector<Importer::MeshImportData::VertexInfluence>& influences, const std::vector<Importer::MeshImportData::MeshWedge>& wedges,
                                          const std::vector<Importer::MeshImportData::MeshFace>& faces, const std::vector<glm::vec3>& points, const std::vector<int>& pointToOriginalMap, const MeshBuildSettings& buildOptions) {
    SkinnedMeshBuildInputData buildData(model, skeleton, influences, wedges, faces, points, pointToOriginalMap, buildOptions);

    if (!PrepareSourceMesh(skinnedMeshName, buildData, overlappingCorners)) {
        return false;
    }

    if (!GenerateRenderableSkinnedMesh(buildData)) {
        return false;
    }

    // Build the skeletal model from chunks.
    // Builder.BeginSlowTask();
    BuildSkinnedMeshModelFromChunks(buildData.model, buildData.skeleton, buildData.chunks, buildData.pointToOriginalMap);
    // UpdateOverlappingVertices(buildData.model);
    // Builder.EndSlowTask();

    // Only show these warnings if in the game thread.  When importing morph targets, this function can run in another
    // thread and these warnings dont prevent the mesh from importing

    bool bHasBadSections = buildData.model.sections.size() != 0;
    for (int sectionIndex = 0; sectionIndex < buildData.model.sections.size(); sectionIndex++) {
        SkinnedSubMeshAsset& section = buildData.model.sections[sectionIndex];
        bHasBadSections |= (section.numTriangles == 0);

        // Log info about the section.
        LOG_INFO("分块" + std::to_string(sectionIndex) + "：" + std::to_string(section.materialIndex) + "个材质索引，" + std::to_string(section.numTriangles) + "个三角面。");
    }
    if (bHasBadSections) {
        LOG_ERROR("分块不包含任何三角面。");
    }

    return true;
}

std::shared_ptr<Assets::MeshAsset> ConvertToMesh(std::shared_ptr<SkinnedMesh> mesh) {
    std::shared_ptr<Assets::MeshAsset> result = std::make_shared<Assets::MeshAsset>();
    result->name = mesh->name;
    result->uniqueID = std::to_string(mesh->fbxMesh->GetUniqueID());
    result->bHasNormals = mesh->bHasNormals;
    result->bHasTangents = mesh->bHasTangents;
    result->bHasVertexColors = mesh->bHasVertexColors;
    result->indexBuffer = mesh->asset->indexBuffer;
    result->indiceFormat = mesh->asset->indexBuffer.size() > 65535 ? Assets::EMeshIndiceFormat::Bit32 : Assets::EMeshIndiceFormat::Bit16;
    result->numTexCoords = mesh->numTexCoords;
    for (SkinnedSubMeshAsset& rawSubMesh : mesh->asset->sections) {
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
                    vertex->uv.emplace_back(static_cast<float>(rawVetex.UVs[i].x), static_cast<float>(rawVetex.UVs[i].y));
                }
            }
            if (mesh->bHasTangents) {
                vertex->tangent.x = rawVetex.tangentX.x;
                vertex->tangent.y = rawVetex.tangentX.y;
                vertex->tangent.z = rawVetex.tangentX.z;
                vertex->tangent.w = 1.0;
            }
            for (int i = 0; i < EXTRA_BONE_INFLUENCES; i++) {
                vertex->boneWeight.push_back(rawVetex.influenceWeights[i]);
                vertex->boneIndex.push_back(rawSubMesh.boneMap[rawVetex.influenceBones[i]]);
            }
            subMesh->numTriangles = rawSubMesh.numTriangles;
            subMesh->vertices.push_back(vertex);
        }
        subMesh->material = mesh->materials[rawSubMesh.materialIndex];
        result->subMeshes.push_back(subMesh);
    }
    for (auto& rawMat : mesh->boneInverseMatrices) {
        Types::Matrix4 mat;
        Types::ConvertFromGLM(mat, rawMat);
        result->boneInverseMatrices.push_back(std::move(mat));
    }
    result->boundingSphere = std::make_shared<Assets::MeshAsset::BoundingSphere>();
    Types::ConvertFromGLM(result->boundingSphere->center, mesh->boundingSphere->origin);
    result->boundingSphere->radius = mesh->boundingSphere->sphereRadius;
    result->boundingBox = std::make_shared<Assets::MeshAsset::BoundingBox>();
    glm::vec3 center, extents;
    mesh->boundingBox->GetCenterAndExtents(center, extents);
    Types::ConvertFromGLM(result->boundingBox->center, center);
    Types::ConvertFromGLM(result->boundingBox->extents, extents);
    result->bForSkinnedMesh = true;
    result->activeBoneIndices = mesh->asset->activeBoneIndices;
    return result;
}

template <class T>
struct SortBySecondPair {
    bool operator()(const T& lhs, const T& rhs) { return lhs.second < rhs.second; }
};

std::shared_ptr<Assets::SkeletonAsset> ConvertToSkeleton(const Builder::SkinnedMeshSkeleton& skeleton, const MeshBoneInfo& root, bool bForSkinnedMesh) {
    std::shared_ptr<Assets::SkeletonAsset> result = std::make_shared<Assets::SkeletonAsset>();
    result->name = root.name;
    int boneNum = skeleton.GetBoneNum();
    int rootIndex = -1;
    for (int i = 0; i < boneNum; i++) {
        std::shared_ptr<Assets::SkeletonAsset::Bone> bone = std::make_shared<Assets::SkeletonAsset::Bone>();
        auto& boneInfo = skeleton.GetBoneInfo()[i];
        if (boneInfo == root) {
            rootIndex = i;
        }
        bone->name = boneInfo.name;
        bone->uniqueID = std::to_string(boneInfo.source->GetUniqueID());
        auto& inTransform = skeleton.GetBonePose()[i];
        Types::ConvertFromGLM(bone->transform.translation, inTransform.translation);
        Types::ConvertFromGLM(bone->transform.scale, inTransform.scale);
        Types::ConvertFromGLM(bone->transform.quaternion, inTransform.rotation);
        result->bones.push_back(bone);
    }
    for (int i = 0; i < boneNum; i++) {
        auto& boneInfo = skeleton.GetBoneInfo()[i];
        if (boneInfo.parentIndex != -1) {
            ASSERT(boneInfo.parentIndex >= 0 && boneInfo.parentIndex < boneNum);
            result->bones[boneInfo.parentIndex]->children.push_back(result->bones[i]);
            result->bones[i]->parent = result->bones[boneInfo.parentIndex];
        }
    }
    for (auto bone : result->bones) {
        std::vector<std::string> parents;
        std::shared_ptr<Assets::SkeletonAsset::Bone> current = bone;
        while (current) {
            parents.push_back(current->name);
            current = current->parent;
        }
        std::reverse(parents.begin(), parents.end());
        bone->path = "/" + Utils::Join(parents, "/");
        result->names.push_back(bone->name);
        result->paths.push_back(bone->path);
    }
    if (rootIndex >= 0) {
        result->root = result->bones[rootIndex];
    }
    result->bForSkinnedMesh = bForSkinnedMesh;

    auto IsChildOrSameNode = [&](std::shared_ptr<Assets::SkeletonAsset::Bone> child, std::shared_ptr<Assets::SkeletonAsset::Bone> inParent) -> bool {
        while (child) {
            if (child == inParent) return true;
            child = child->parent;
        }
        return false;
    };

    auto FindRootIndex = [&](std::shared_ptr<Assets::SkeletonAsset::Bone> node, std::vector<std::pair<std::shared_ptr<Assets::SkeletonAsset::Bone>, int>>& roots) -> int {
        for (int i = 0; i < roots.size(); i++) {
            if (IsChildOrSameNode(node, roots[i].first)) return i;
        }
        return -1;
    };

    std::multimap<int, std::shared_ptr<Assets::SkeletonAsset::Bone>> rootsSortedByDepth;
    for (auto& bone : result->bones) {
        int depth = 0;
        {
            std::shared_ptr<Assets::SkeletonAsset::Bone> parent = bone->parent;
            while (parent) {
                depth++;
                parent = parent->parent;
            }
        }
        rootsSortedByDepth.emplace(depth, bone);
    }

    std::vector<std::pair<std::shared_ptr<Assets::SkeletonAsset::Bone>, int>> rootsAndNumberOfChildBones;
    for (auto kvp : rootsSortedByDepth) {
        std::shared_ptr<Assets::SkeletonAsset::Bone> bone = kvp.second;
        int found = FindRootIndex(bone, rootsAndNumberOfChildBones);
        if (found == -1) {
            rootsAndNumberOfChildBones.push_back(std::make_pair(bone, 1));
        } else {
            rootsAndNumberOfChildBones[found].second++;
        }
    }
    std::vector<std::shared_ptr<Assets::SkeletonAsset::Bone>> rootBones;
    std::stable_sort(rootsAndNumberOfChildBones.begin(), rootsAndNumberOfChildBones.end(), SortBySecondPair<std::pair<std::shared_ptr<Assets::SkeletonAsset::Bone>, int>>());
    std::vector<std::shared_ptr<Assets::SkeletonAsset::Bone>> rootsSortedByNumberOfChildren;
    for (int i = 0; i < rootsAndNumberOfChildBones.size(); i++) {
        rootsSortedByNumberOfChildren.push_back(rootsAndNumberOfChildBones[i].first);
    }
    std::shared_ptr<Assets::SkeletonAsset::Bone> mostUsedRoot = rootsSortedByNumberOfChildren.empty() ? nullptr : rootsSortedByNumberOfChildren.back();
    rootBones.push_back(mostUsedRoot);
    result->rootBone = (rootBones.size() && rootBones.front() != nullptr) ? rootBones.front() : nullptr;
    return result;
}

/**
 * Process and fill in the mesh Materials using the raw binary import data
 *
 * @param Materials - [out] array of materials to update
 * @param ImportData - raw binary import data to process
 */
void SkinnedMeshHelper::ProcessImportMeshMaterials(std::vector<std::shared_ptr<Assets::MaterialAsset>>& materials, std::shared_ptr<Importer::MeshImportData> importData) {
    std::vector<Importer::MeshImportData::RawMaterial>& importedMaterials = importData->materials;

    // If direct linkup of materials is requested, try to find them here - to get a texture name from a
    // material name, cut off anything in front of the dot (beyond are special flags).
    materials.clear();
    int skinOffset = -1;
    for (int matIndex = 0; matIndex < importedMaterials.size(); ++matIndex) {
        const Importer::MeshImportData::RawMaterial& importedMaterial = importedMaterials[matIndex];

        std::shared_ptr<Assets::MaterialAsset> material = nullptr;
        std::string materialNameNoSkin = importedMaterial.materialImportName;
        if (importedMaterial.material != nullptr) {
            material = importedMaterial.material;
        }
        if (material == nullptr) {
            material = std::make_shared<Assets::MaterialAsset>();
            material->name = "Default Material";
        }
        materials.push_back(material);
    }

    int numMaterialsToAdd = Maths::Max<int>(static_cast<int>(importedMaterials.size()), importData->maxMaterialIndex + 1);

    // Pad the material pointers
    while (numMaterialsToAdd > materials.size()) {
        FbxString lMaterialName = "Default Material";
        FbxSurfaceMaterial* fbxMat = FbxSurfaceLambert::Create(Importer::Importer::GetInstance()->sdkManager, lMaterialName.Buffer());
        std::shared_ptr<Assets::MaterialAsset> material = std::make_shared<Assets::MaterialAsset>();
        material->name = "Default Material";
        materials.push_back(material);
    }
}

/**
 * Process and update the vertex Influences using the predefined wedges
 *
 * @param WedgeCount - The number of wedges in the corresponding mesh.
 * @param Influences - BoneWeights and Ids for the corresponding vertices.
 */
void SkinnedMeshHelper::ProcessImportMeshInfluences(std::shared_ptr<Importer::MeshImportData> importData, const std::string& meshName) {
    std::vector<Importer::MeshImportData::RawBoneInfluence>& influences = importData->influences;
    int WedgeCount = static_cast<int>(importData->wedges.size());
    // Sort influences by vertex index.
    struct FCompareVertexIndex {
        bool operator()(const Importer::MeshImportData::RawBoneInfluence& A, const Importer::MeshImportData::RawBoneInfluence& B) const {
            if (A.vertexIndex > B.vertexIndex)
                return false;
            else if (A.vertexIndex < B.vertexIndex)
                return true;
            else if (A.weight < B.weight)
                return false;
            else if (A.weight > B.weight)
                return true;
            else if (A.boneIndex > B.boneIndex)
                return false;
            else if (A.boneIndex < B.boneIndex)
                return true;
            else
                return false;
        }
    };
    std::sort(influences.begin(), influences.end(), FCompareVertexIndex());

    std::vector<Importer::MeshImportData::RawBoneInfluence> newInfluences;
    int lastNewInfluenceIndex = 0;
    int lastVertexIndex = -1;
    int influenceCount = 0;

    float totalWeight = 0.f;
    const float MINWEIGHT = 0.01f;

    int maxVertexInfluence = 0;
    float maxIgnoredWeight = 0.0f;

    // We have to normalize the data before filtering influences
    // Because influence filtering is base on the normalize value.
    // Some DCC like Daz studio don't have normalized weight
    for (int i = 0; i < influences.size(); i++) {
        // if less than min weight, or it's more than 8, then we clear it to use weight
        influenceCount++;
        totalWeight += influences[i].weight;
        // we have all influence for the same vertex, normalize it now
        if (i + 1 >= influences.size() || influences[i].vertexIndex != influences[i + 1].vertexIndex) {
            // Normalize the last set of influences.
            if (influenceCount && (totalWeight != 1.0f)) {
                float oneOverTotalWeight = 1.f / totalWeight;
                for (int r = 0; r < influenceCount; r++) {
                    influences[i - r].weight *= oneOverTotalWeight;
                }
            }

            if (maxVertexInfluence < influenceCount) {
                maxVertexInfluence = influenceCount;
            }

            // clear to count next one
            influenceCount = 0;
            totalWeight = 0.f;
        }

        if (influenceCount > MAX_TOTAL_INFLUENCES && influences[i].weight > maxIgnoredWeight) {
            maxIgnoredWeight = influences[i].weight;
        }
    }

    // warn about too many influences
    if (maxVertexInfluence > MAX_TOTAL_INFLUENCES) {
        // UE_LOG(LogLODUtilities, Display, TEXT("Skeletal mesh (%s) influence count of %d exceeds max count of %d.
        // Influence truncation will occur. Maximum Ignored Weight %f"), *MeshName, MaxVertexInfluence,
        // MAX_TOTAL_INFLUENCES, MaxIgnoredWeight);
    }

    for (int i = 0; i < influences.size(); i++) {
        // we found next verts, normalize it now
        if (lastVertexIndex != influences[i].vertexIndex) {
            // Normalize the last set of influences.
            if (influenceCount && (totalWeight != 1.0f)) {
                float oneOverTotalWeight = 1.f / totalWeight;
                for (int r = 0; r < influenceCount; r++) {
                    newInfluences[lastNewInfluenceIndex - r].weight *= oneOverTotalWeight;
                }
            }

            // now we insert missing verts
            if (lastVertexIndex != -1) {
                int CurrentVertexIndex = influences[i].vertexIndex;
                for (int j = lastVertexIndex + 1; j < CurrentVertexIndex; j++) {
                    // Add a 0-bone weight if none other present (known to happen with certain MAX skeletal setups).
                    newInfluences.push_back(Importer::MeshImportData::RawBoneInfluence());
                    lastNewInfluenceIndex = static_cast<int>(newInfluences.size() - 1);
                    newInfluences[lastNewInfluenceIndex].vertexIndex = j;
                    newInfluences[lastNewInfluenceIndex].boneIndex = 0;
                    newInfluences[lastNewInfluenceIndex].weight = 1.f;
                }
            }

            // clear to count next one
            influenceCount = 0;
            totalWeight = 0.f;
            lastVertexIndex = influences[i].vertexIndex;
        }

        // if less than min weight, or it's more than 8, then we clear it to use weight
        if (influences[i].weight > MINWEIGHT && influenceCount < MAX_TOTAL_INFLUENCES) {
            newInfluences.push_back(influences[i]);
            lastNewInfluenceIndex = static_cast<int>(newInfluences.size() - 1);
            influenceCount++;
            totalWeight += influences[i].weight;
        }
    }

    influences = newInfluences;

    // Ensure that each vertex has at least one influence as e.g. CreateSkinningStream relies on it.
    // The below code relies on influences being sorted by vertex index.
    if (influences.size() == 0) {
        // warn about no influences
        // UE_LOG(LogLODUtilities, Warning, TEXT("Warning skeletal mesh (%s) has no vertex influences"), *MeshName);
        // add one for each wedge entry
        influences.resize(WedgeCount);
        for (int wedgeIdx = 0; wedgeIdx < WedgeCount; wedgeIdx++) {
            influences[wedgeIdx].vertexIndex = wedgeIdx;
            influences[wedgeIdx].boneIndex = 0;
            influences[wedgeIdx].weight = 1.0f;
        }
        for (int i = 0; i < influences.size(); i++) {
            int currentVertexIndex = influences[i].vertexIndex;

            if (lastVertexIndex != currentVertexIndex) {
                for (int j = lastVertexIndex + 1; j < currentVertexIndex; j++) {
                    // Add a 0-bone weight if none other present (known to happen with certain MAX skeletal setups).
                    // Influences.InsertUninitialized(i, 1);
                    influences[i].vertexIndex = j;
                    influences[i].boneIndex = 0;
                    influences[i].weight = 1.f;
                }
                lastVertexIndex = currentVertexIndex;
            }
        }
    }
}

/**
 * Process and fill in the mesh ref skeleton bone hierarchy using the raw binary import data
 *
 * @param RefSkeleton - [out] reference skeleton hierarchy to update
 * @param SkeletalDepth - [out] depth of the reference skeleton hierarchy
 * @param ImportData - raw binary import data to process
 * @return true if the operation completed successfully
 */
bool SkinnedMeshHelper::ProcessImportSkinnedMeshSkeleton(SkinnedMeshSkeleton& skeleton, int& skeletalDepth, std::shared_ptr<Importer::MeshImportData> importData) {
    std::vector<Importer::MeshImportData::Bone>& bones = importData->bones;

    // Setup skeletal hierarchy + names structure.
    skeleton.Empty();

    // Digest bones to the serializable format.
    for (int b = 0; b < bones.size(); b++) {
        const Importer::MeshImportData::Bone& bone = bones[b];
        const MeshBoneInfo boneInfo(bone.source, bone.name, bone.parentIndex);
        const Maths::Transform boneTransform(bone.bonePosition.transform);

        if (skeleton.FindRawBoneIndex(boneInfo.name) != -1) {
            // FFbxImporter->AddTokenizedErrorMessage(FTokenizedMessage::Create(EMessageSeverity::Error,
            // FText::Format(LOCTEXT("SkeletonHasDuplicateBones", "Skeleton has non-unique bone names.\nBone named '{0}'
            // encountered more than once."), FText::FromName(BoneInfo.Name))),
            // FFbxErrors::SkeletalMesh_DuplicateBones);
            return false;
        }

        skeleton.Add(boneInfo, boneTransform);
    }

    // Add hierarchy index to each bone and detect max depth.
    skeletalDepth = 0;

    std::vector<int> skeletalDepths;
    skeletalDepths.resize(bones.size(), 0);
    for (int b = 0; b < skeleton.GetBoneNum(); b++) {
        int parent = skeleton.GetParentIndex(b);
        int depth = 1;

        skeletalDepths[b] = 1;
        if (parent != -1) {
            depth += skeletalDepths[parent];
        }
        if (skeletalDepth < depth) {
            skeletalDepth = depth;
        }
        skeletalDepths[b] = depth;
    }

    skeleton.RebuildRefSkeleton(true);

    return true;
}
}}  // namespace Fbx::Builder