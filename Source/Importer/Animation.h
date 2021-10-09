#pragma once
#include <fbxsdk.h>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#undef snprintf
#include "FbxParser.private.h"

namespace Fbx {
namespace Builder {
class SkinnedMeshSkeleton;
}

namespace Importer {

struct FRawAnimSequenceTrack {
    /** Position keys. */
    std::vector<glm::vec3> posKeys;

    /** Rotation keys. */
    std::vector<glm::quat> rotKeys;

    /** Scale keys. */
    std::vector<glm::vec3> scaleKeys;

    std::vector<int> posIndices;
    std::vector<int> rotIndices;
    std::vector<int> scaleIndices;

    std::vector<glm::vec3> posInTangents;
    std::vector<glm::quat> rotInTangents;
    std::vector<glm::vec3> scaleInTangents;

    std::vector<glm::vec3> posOutTangents;
    std::vector<glm::quat> rotOutTangents;
    std::vector<glm::vec3> scaleOutTangents;
};

struct FTrackToSkeletonMap {
    int boneTreeIndex;

    FTrackToSkeletonMap() : boneTreeIndex(0) {}

    FTrackToSkeletonMap(int InBoneTreeIndex) : boneTreeIndex(InBoneTreeIndex) {}
};

/**
 * Container and importer for skinned mesh (FBX file) data
 **/
class AnimationImportData {
public:
    std::string sequenceName;
    float importFileFramerate;
    float importResampleFramerate;
    float sequenceLength = 0;

    const std::vector<FbxNode*>& sortedLinks;
    const std::vector<std::string>& fbxRawBoneNames;
    const std::vector<std::string>& fbxRawBonePaths;
    const FbxNode* rootNode;
    const FbxTimeSpan& animTimeSpan;
    const Builder::SkinnedMeshSkeleton& skeleton;
    const std::vector<FbxNode*>& ignoredSortedLinks;

    AnimationImportData(const Builder::SkinnedMeshSkeleton& inSkeleton, const std::vector<FbxNode*>& inSortedLinks, const std::vector<FbxNode*>& inIgnoredSortedLinks, const std::vector<std::string>& inFbxRawBoneNames, const std::vector<std::string>& inFbxRawBonePaths, const FbxNode* inRootNode,
                        const FbxTimeSpan& inAnimTimeSpan)
        : skeleton(inSkeleton), sortedLinks(inSortedLinks), ignoredSortedLinks(inIgnoredSortedLinks), fbxRawBoneNames(inFbxRawBoneNames), fbxRawBonePaths(inFbxRawBonePaths), rootNode(inRootNode), animTimeSpan(inAnimTimeSpan) {}

    int numFrames;

    std::vector<std::string> animationTrackNames;
    std::vector<FRawAnimSequenceTrack> rawAnimationData;
    std::vector<FTrackToSkeletonMap> trackToSkeletonMapTable;
};

}  // namespace Importer
}  // namespace Fbx