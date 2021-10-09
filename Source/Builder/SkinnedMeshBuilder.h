#pragma once
#include <map>
#include <algorithm>
#include "FbxParser.h"
#include "FbxParser.private.h"
#include "Builder/MeshBuilder.h"
#include "Importer/Mesh.h"
#include "mikktspace.h"
namespace Fbx { namespace Builder {

// This contains Reference-skeleton related info
// Bone transform is saved as FTransform array
struct MeshBoneInfo {
    // Bone's name.
    std::string name;

    FbxNode* source;

    // 0/NULL if this is the root bone.
    int parentIndex;

    MeshBoneInfo(FbxNode* sourceNode) : source(sourceNode), name(""), parentIndex(-1) {}

    MeshBoneInfo(FbxNode* sourceNode, const std::string& inName, int inParentIndex) : source(sourceNode), name(inName), parentIndex(inParentIndex) {}

    MeshBoneInfo(const MeshBoneInfo& other) : name(other.name), parentIndex(other.parentIndex), source(other.source) {}

    bool operator==(const MeshBoneInfo& other) const { return (name == other.name); }
};

class SkinnedMeshSkeleton {
private:
    /** Reference bone related info to be serialized **/
    std::vector<MeshBoneInfo> boneInfo;
    /** Reference bone transform **/
    std::vector<Maths::Transform> bonePose;
    /** TMap to look up bone index from bone name. */
    std::map<std::string, int> nameToIndexMap;

public:
    /** Accessor to private data. Raw relates to original asset. Const so it can't be changed recklessly. */
    const std::vector<Maths::Transform>& GetBonePose() const { return bonePose; }

    const std::vector<MeshBoneInfo>& GetBoneInfo() const { return boneInfo; }

    void RebuildNameToIndexMap() {
        // Start by clearing the current map.
        nameToIndexMap.clear();

        // Then iterate over each bone, adding the name and bone index.
        const int numBones = static_cast<int>(boneInfo.size());
        for (int boneIndex = 0; boneIndex < numBones; boneIndex++) {
            const std::string& boneName = boneInfo[boneIndex].name;
            if (boneName != "") {
                nameToIndexMap.emplace(boneName, boneIndex);
            } else {
                // UE_LOG(LogAnimation, Warning, TEXT("RebuildNameToIndexMap: Bone with no name detected for index:
                // %d"), BoneIndex);
            }
        }

        // Make sure we don't have duplicate bone names. This would be very bad.
        ASSERT(nameToIndexMap.size() == numBones);
    }

    void RebuildRefSkeleton(bool bRebuildNameMap) {
        if (bRebuildNameMap) {
            // On loading FinalRefBone data wont exist but NameToIndexMap will and will be valid
            RebuildNameToIndexMap();
        }
    }

    /** Add a new bone.
     * BoneName must not already exist! ParentIndex must be valid. */
    void Add(const MeshBoneInfo& inBoneInfo, const Maths::Transform& inBonePose) {
        // Adding a bone that already exists is illegal
        ASSERT(FindRawBoneIndex(inBoneInfo.name) == -1);

        // Make sure our arrays are in sync.
        ASSERT((boneInfo.size() == bonePose.size()) && (boneInfo.size() == nameToIndexMap.size()));

        const int boneIndex = static_cast<int>(boneInfo.size());
        boneInfo.push_back(inBoneInfo);
        bonePose.push_back(inBonePose);
        nameToIndexMap.emplace(inBoneInfo.name, boneIndex);

        // Normalize Quaternion to be safe.
        bonePose[boneIndex].rotation = glm::normalize(bonePose[boneIndex].rotation);

        // Parent must be valid. Either INDEX_NONE for Root, or before children for non root bones.
        ASSERT((((boneIndex == 0) && (inBoneInfo.parentIndex == -1)) || ((boneIndex > 0) && boneInfo.size() > inBoneInfo.parentIndex && inBoneInfo.parentIndex >= 0 && (inBoneInfo.parentIndex < boneIndex))));
    }

    /** Returns number of raw bones in Skeleton. These are the original bones of the asset */
    int GetBoneNum() const { return static_cast<int>(boneInfo.size()); }

    void Empty() {
        boneInfo.clear();
        bonePose.clear();
        nameToIndexMap.clear();
    }

    /** Find Bone Index from BoneName. Precache as much as possible in speed critical sections! */
    int FindRawBoneIndex(const std::string& boneName) const {
        ASSERT(boneInfo.size() == nameToIndexMap.size());
        int BoneIndex = -1;
        if (boneName != "") {
            auto indexPtr = nameToIndexMap.find(boneName);
            if (indexPtr != nameToIndexMap.end()) {
                BoneIndex = indexPtr->second;
            }
        }
        return BoneIndex;
    }

    int GetParentIndex(const int boneIndex) const {
        const int parentIndex = boneInfo[boneIndex].parentIndex;

        // Parent must be valid. Either INDEX_NONE for Root, or before children for non root bones.
        ASSERT((((boneIndex == 0) && (parentIndex == -1)) || ((boneIndex > 0) && boneInfo.size() > parentIndex && parentIndex >= 0 && (parentIndex < boneIndex))));

        return parentIndex;
    }

    void EnsureParentsExistAndSort(std::vector<uint16_t>& inOutBoneUnsortedArray) const {
        std::sort(inOutBoneUnsortedArray.begin(), inOutBoneUnsortedArray.end());

        EnsureParentsExist(inOutBoneUnsortedArray);

        std::sort(inOutBoneUnsortedArray.begin(), inOutBoneUnsortedArray.end());
    }

private:
    void EnsureParentsExist(std::vector<uint16_t>& InOutBoneSortedArray) const {
        const int NumBones = GetBoneNum();
        // Iterate through existing array.
        int i = 0;

        std::vector<bool> BoneExists(NumBones, 0);

        while (i < InOutBoneSortedArray.size()) {
            const int BoneIndex = InOutBoneSortedArray[i];

            // For the root bone, just move on.
            if (BoneIndex > 0) {
                BoneExists[BoneIndex] = true;

                const int ParentIndex = GetParentIndex(BoneIndex);

                // If we do not have this parent in the array, we add it in this location, and leave 'i' where it is.
                // This can happen if somebody removes bones in the physics asset, then it will try add back in, and in
                // the process, parent can be missing
                if (!BoneExists[ParentIndex]) {
                    InOutBoneSortedArray.insert(InOutBoneSortedArray.begin() + i, std::numeric_limits<uint16_t>::max());
                    InOutBoneSortedArray[i] = ParentIndex;
                    BoneExists[ParentIndex] = true;
                }
                // If parent was in array, just move on.
                else {
                    i++;
                }
            } else {
                BoneExists[0] = true;
                i++;
            }
        }
    }
};

class SkinnedMeshAsset;

class SkinnedSubMeshAsset : public SubMeshAsset {
public:
    /** The bones which are used by the vertices of this section. Indices of bones in the USkeletalMesh::RefSkeleton
     * array */
    std::vector<uint16_t> boneMap;

    /** max # of bones used to skin the vertices in this section */
    int maxBoneInfluences;

    SkinnedSubMeshAsset();

    /**
     * Calculate max # of bone influences used by this skel mesh section
     */
    void CalcMaxBoneInfluences();
};

class SkinnedMeshAsset : public MeshAsset<SkinnedSubMeshAsset> {
public:
    /**
     * Bone hierarchy subset active for this LOD.
     * This is a map between the bones index of this LOD (as used by the vertex structs) and the bone index in the
     * reference skeleton of this SkeletalMesh.
     */
    std::vector<uint16_t> activeBoneIndices;

    /**
     * Bones that should be updated when rendering this LOD. This may include bones that are not required for rendering.
     * All parents for bones in this array should be present as well - that is, a complete path from the root to each
     * bone. For bone LOD code to work, this array must be in strictly increasing order, to allow easy merging of other
     * required bones.
     */
    std::vector<uint16_t> requiredBones;
};

class SkinnedMesh : public Mesh<SkinnedSubMeshAsset, SkinnedMeshAsset> {
public:
    SkinnedMeshSkeleton skeleton;
    std::vector<glm::mat4> boneInverseMatrices;

private:
    /** Cached matrices from GetComposedRefPoseMatrix */
    std::vector<glm::mat4> CachedComposedRefPoseMatrices;

public:
    glm::mat4 GetRefPoseMatrix(int BoneIndex) const {
        ASSERT(BoneIndex >= 0 && BoneIndex < skeleton.GetBoneNum());
        Maths::Transform BoneTransform = skeleton.GetBonePose()[BoneIndex];
        // Make sure quaternion is normalized!
        BoneTransform.NormalizeRotation();
        return BoneTransform.ToMatrixWithScale();
    }

    // Pre-calculate refpose-to-local transforms
    void CalculateInvRefMatrices() {
        const int numRealBones = skeleton.GetBoneNum();

        if (boneInverseMatrices.size() != numRealBones) {
            boneInverseMatrices.resize(numRealBones, glm::mat4(0));

            // Reset cached mesh-space ref pose
            CachedComposedRefPoseMatrices.resize(numRealBones, glm::mat4(0));

            // Precompute the Mesh.RefBasesInverse.
            for (int b = 0; b < numRealBones; b++) {
                // Render the default pose.
                CachedComposedRefPoseMatrices[b] = GetRefPoseMatrix(b);

                // Construct mesh-space skeletal hierarchy.
                if (b > 0) {
                    int Parent = skeleton.GetParentIndex(b);
                    CachedComposedRefPoseMatrices[b] = CachedComposedRefPoseMatrices[Parent] * CachedComposedRefPoseMatrices[b];
                }

                glm::vec3 XAxis, YAxis, ZAxis;

                Maths::GetMatrixScaledAxes(CachedComposedRefPoseMatrices[b], XAxis, YAxis, ZAxis);
                if (Maths::IsNearlyZero(XAxis, SMALL_NUMBER) && Maths::IsNearlyZero(YAxis, SMALL_NUMBER) && Maths::IsNearlyZero(ZAxis, SMALL_NUMBER)) {
                    // this is not allowed, warn them
                    LOG_WARN("Reference Pose for asset " + name + " for joint (" + skeleton.GetBoneInfo()[b].name + ") includes NIL matrix. Zero scale isn't allowed on ref pose. ");
                }

                // Precompute inverse so we can use from-refpose-skin vertices.
                boneInverseMatrices[b] = glm::inverse(CachedComposedRefPoseMatrices[b]);
            }
        }
    }
};

struct SkinnedSubMesh : public OriginSubMesh {
    /** If not empty, contains a map from bones referenced in this chunk to the skeleton. */
    std::vector<uint16_t> boneMap;
};

class SkinnedMeshBuildInputData : public IMeshBuildInputData {
public:
    SkinnedMeshBuildInputData(SkinnedMeshAsset& inModel, const SkinnedMeshSkeleton& inSkeleton, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences, const std::vector<Importer::MeshImportData::MeshWedge>& inWedges,
                              const std::vector<Importer::MeshImportData::MeshFace>& inFaces, const std::vector<glm::vec3>& inPoints, const std::vector<int>& inPointToOriginalMap, const MeshBuildSettings& inBuildOptions);

    std::vector<std::shared_ptr<SkinnedSubMesh>> chunks;

    SkinnedMeshAsset& model;
    const SkinnedMeshSkeleton& skeleton;
};

class SkinnedMeshBuilder {
public:
    bool Build(std::shared_ptr<SkinnedMesh> skeletalMesh, std::shared_ptr<Importer::MeshImportData> meshImportData, const MeshBuildSettings& options);

    bool BuildSkinnedMesh(SkinnedMeshAsset& model, const std::string& skeletalMeshName, const SkinnedMeshSkeleton& skeleton, const std::vector<Importer::MeshImportData::VertexInfluence>& influences, const std::vector<Importer::MeshImportData::MeshWedge>& wedges,
                          const std::vector<Importer::MeshImportData::MeshFace>& faces, const std::vector<glm::vec3>& points, const std::vector<int>& pointToOriginalMap, const MeshBuildSettings& buildOptions);

private:
    bool GenerateRenderableSkinnedMesh(SkinnedMeshBuildInputData& inBuildData);

    std::vector<std::shared_ptr<OverlappingCorners>> overlappingCorners;
};

std::shared_ptr<Assets::MeshAsset> ConvertToMesh(std::shared_ptr<SkinnedMesh> mesh);

std::shared_ptr<Assets::SkeletonAsset> ConvertToSkeleton(const Builder::SkinnedMeshSkeleton& skeleton, const MeshBoneInfo& root, bool bForSkinnedMesh);

class PolygonShellsHelper {
public:
    struct PatchAndBoneInfluence {
        std::vector<uint16_t> uniqueBones;
        std::vector<int> patchToChunkWith;
        bool bIsParent = false;
    };

    static void FillPolygonPatch(const std::vector<uint32_t>& indices, const std::vector<SubMeshVertexWithWedgeIdx>& vertices, const std::map<uint32_t, std::vector<uint16_t>>& alternateBoneIDs, std::vector<PatchAndBoneInfluence>& patchData, std::vector<std::vector<uint32_t>>& patchIndexToIndices,
                                 std::map<int, std::vector<uint16_t>>& bonesPerFace, const int maxBonesPerChunk, const bool bConnectByEdge);

    static void AddAdjacentFace(const std::vector<uint32_t>& Indices, const std::vector<SubMeshVertexWithWedgeIdx>& vertices, std::vector<bool>& faceAdded, const std::map<int, std::vector<int>>& vertexIndexToAdjacentFaces, const int faceIndex, std::vector<int>& triangleQueue,
                                const bool bConnectByEdge);

    static void GatherShellUsingSameBones(const int parentPatchIndex, std::vector<PatchAndBoneInfluence>& patchData, std::vector<bool>& patchConsumed, const int maxBonesPerChunk);

    static void RecursiveFillRemapIndices(const std::vector<PatchAndBoneInfluence>& patchData, const int patchIndex, const std::vector<std::vector<uint32_t>>& patchIndexToIndices, std::vector<uint32_t>& srcChunkRemapIndicesIndex);
};

namespace SkinnedMeshHelper {
void ProcessImportMeshInfluences(std::shared_ptr<Importer::MeshImportData> importData, const std::string& skeletalMeshName);
void ProcessImportMeshMaterials(std::vector<std::shared_ptr<Assets::MaterialAsset>>& materials, std::shared_ptr<Importer::MeshImportData> importData);
bool ProcessImportSkinnedMeshSkeleton(SkinnedMeshSkeleton& refSkeleton, int& skeletalDepth, std::shared_ptr<Importer::MeshImportData> importData);
}  // namespace SkinnedMeshHelper
}}  // namespace Fbx::Builder