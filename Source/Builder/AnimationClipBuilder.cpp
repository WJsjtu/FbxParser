#include "AnimationClipBuilder.h"
namespace Fbx { namespace Builder {

bool AnimationClipBuilder::Build(std::shared_ptr<Assets::AnimationClipAsset> asset, std::shared_ptr<Importer::AnimationImportData> animationImportData, const AnimationBuildSettings& options) {
    float t = 0.5;
    float delta = 2.0 / animationImportData->importResampleFramerate;
    float ttt = t * t * t;
    float tt = t * t;
    float a = 2 * ttt - 3 * tt + 1;
    float b = -2 * ttt + 3 * tt;
    float c = ttt - 2 * tt + t;
    float d = ttt - tt;

    float da = 6 * tt - 6 * t;
    float db = -6 * tt + 6 * t;
    float dc = 3 * tt - 4 * t + 1;
    float dd = 3 * tt - 2 * t;

    for (auto& track : animationImportData->rawAnimationData) {
        {
            bool compressed = true;
            while (track.posKeys.size() > 3 && compressed) {
                compressed = false;
                for (int i = 1; i < track.posKeys.size() - 1; i++) {
                    if (!Maths::IsNearlyEqual(track.posOutTangents[i], track.posOutTangents[i], 1e-4f)) {
                        continue;
                    }
                    glm::vec3 pos = track.posKeys[i - 1] * a + track.posKeys[i + 1] * b + track.posOutTangents[i - 1] * c * delta + track.posInTangents[i + 1] * d * delta;
                    glm::vec3 dPos = pos - track.posKeys[i];
                    if (abs(dPos.x) <= abs(track.posKeys[i].x) * options.posReductionThreshold && abs(dPos.y) <= abs(track.posKeys[i].y) * options.posReductionThreshold && abs(dPos.z) <= abs(track.posKeys[i].z) * options.posReductionThreshold) {
                        glm::vec3 derivative = track.posKeys[i - 1] * da / delta + track.posKeys[i + 1] * db / delta + track.posOutTangents[i - 1] * dc + track.posInTangents[i + 1] * dd;
                        glm::vec3 dd = glm::vec3(abs(atan(derivative.x) - atan(track.posInTangents[i].x)), abs(atan(derivative.y) - atan(track.posInTangents[i].y)), abs(atan(derivative.z) - atan(track.posInTangents[i].z)));
                        dd = dd / 3.1415926535f * 180.0f;
                        if (dd.x <= 0.5 && dd.y <= 0.5 && dd.z <= 0.5) {
                            track.posKeys.erase(track.posKeys.begin() + i);
                            track.posInTangents.erase(track.posInTangents.begin() + i);
                            track.posOutTangents.erase(track.posOutTangents.begin() + i);
                            track.posIndices.erase(track.posIndices.begin() + i);
                            compressed = true;
                            break;
                        }
                    }
                }
            }
        }

        {
            bool compressed = true;
            while (track.scaleKeys.size() > 3 && compressed) {
                compressed = false;
                for (int i = 1; i < track.scaleKeys.size() - 1; i++) {
                    if (!Maths::IsNearlyEqual(track.scaleOutTangents[i], track.scaleOutTangents[i], 1e-4)) {
                        continue;
                    }
                    glm::vec3 scale = track.scaleKeys[i - 1] * a + track.scaleKeys[i + 1] * b + track.scaleOutTangents[i - 1] * c * delta + track.scaleInTangents[i + 1] * d * delta;
                    glm::vec3 dScale = scale - track.scaleKeys[i];
                    if (abs(dScale.x) <= abs(track.scaleKeys[i].x) * options.scaleReductionThreshold && abs(dScale.y) <= abs(track.scaleKeys[i].y) * options.scaleReductionThreshold && abs(dScale.z) <= abs(track.scaleKeys[i].z) * options.scaleReductionThreshold) {
                        glm::vec3 derivative = track.scaleKeys[i - 1] * da / delta + track.scaleKeys[i + 1] * db / delta + track.scaleOutTangents[i - 1] * dc + track.scaleInTangents[i + 1] * dd;
                        glm::vec3 dd = glm::vec3(abs(atan(derivative.x) - atan(track.scaleInTangents[i].x)), abs(atan(derivative.y) - atan(track.scaleInTangents[i].y)), abs(atan(derivative.z) - atan(track.scaleInTangents[i].z)));
                        dd = dd / 3.1415926535f * 180.0f;
                        if (dd.x <= 0.5 && dd.y <= 0.5 && dd.z <= 0.5) {
                            track.scaleKeys.erase(track.scaleKeys.begin() + i);
                            track.scaleInTangents.erase(track.scaleInTangents.begin() + i);
                            track.scaleOutTangents.erase(track.scaleOutTangents.begin() + i);
                            track.scaleIndices.erase(track.scaleIndices.begin() + i);
                            compressed = true;
                            break;
                        }
                    }
                }
            }
        }

        {
            bool compressed = true;
            while (track.rotKeys.size() > 3 && compressed) {
                compressed = false;
                for (int i = 1; i < track.rotKeys.size() - 1; i++) {
                    if (!Maths::IsNearlyEqual(track.rotOutTangents[i], track.rotOutTangents[i], 1e-4f)) {
                        continue;
                    }
                    glm::quat rot = track.rotKeys[i - 1] * a + track.rotKeys[i + 1] * b + track.rotOutTangents[i - 1] * c * delta + track.rotInTangents[i + 1] * d * delta;
                    glm::quat dRot = rot - track.rotKeys[i];
                    if (abs(dRot.x) <= abs(track.rotKeys[i].x) * options.rotReductionThreshold && abs(dRot.y) <= abs(track.rotKeys[i].y) * options.rotReductionThreshold && abs(dRot.z) <= abs(track.rotKeys[i].z) * options.rotReductionThreshold &&
                        abs(dRot.w) <= abs(track.rotKeys[i].w) * options.rotReductionThreshold) {
                        glm::quat derivative = track.rotKeys[i - 1] * da / delta + track.rotKeys[i + 1] * db / delta + track.rotOutTangents[i - 1] * dc + track.rotInTangents[i + 1] * dd;
                        glm::vec4 dd = glm::vec4(abs(atan(derivative.x) - atan(track.rotInTangents[i].x)), abs(atan(derivative.y) - atan(track.rotInTangents[i].y)), abs(atan(derivative.z) - atan(track.rotInTangents[i].z)), abs(atan(derivative.w) - atan(track.rotInTangents[i].w)));
                        dd = dd / 3.1415926535f * 180.0f;
                        if (dd.x <= 0.5 && dd.y <= 0.5 && dd.z <= 0.5 && dd.w <= 0.5) {
                            track.rotKeys.erase(track.rotKeys.begin() + i);
                            track.rotInTangents.erase(track.rotInTangents.begin() + i);
                            track.rotOutTangents.erase(track.rotOutTangents.begin() + i);
                            track.rotIndices.erase(track.rotIndices.begin() + i);
                            compressed = true;
                        }
                    }
                }
            }
        }
    }

    asset->name = animationImportData->sequenceName;
    asset->uniqueID = std::to_string(animationImportData->rootNode->GetUniqueID());
    asset->length = animationImportData->sequenceLength;
    asset->frameRate = animationImportData->importResampleFramerate;
    asset->frameCount = animationImportData->numFrames;
    if (animationImportData->rawAnimationData.size()) {
        for (int i = 0; i < animationImportData->rawAnimationData.size(); i++) {
            auto& track = animationImportData->rawAnimationData[i];
            if (track.posKeys.size()) {
                for (int j = 0; j < track.posKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.posIndices[j];
                    frameInfo->value = track.posKeys[j].x;
                    frameInfo->inTangent = track.posInTangents[j].x;
                    frameInfo->outTangent = track.posOutTangents[j].x;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.posKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.posIndices[j];
                    frameInfo->value = track.posKeys[j].y;
                    frameInfo->inTangent = track.posInTangents[j].y;
                    frameInfo->outTangent = track.posOutTangents[j].y;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.posKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.posIndices[j];
                    frameInfo->value = track.posKeys[j].z;
                    frameInfo->inTangent = track.posInTangents[j].z;
                    frameInfo->outTangent = track.posOutTangents[j].z;
                    asset->frames.push_back(frameInfo);
                }

                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::PositionX;
                    curveInfo->frameCount = static_cast<int>(track.posKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::PositionY;
                    curveInfo->frameCount = static_cast<int>(track.posKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::PositionZ;
                    curveInfo->frameCount = static_cast<int>(track.posKeys.size());
                    asset->curves.push_back(curveInfo);
                }
            }
            if (track.scaleKeys.size()) {
                for (int j = 0; j < track.scaleKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.scaleIndices[j];
                    frameInfo->value = track.scaleKeys[j].x;
                    frameInfo->inTangent = track.scaleInTangents[j].x;
                    frameInfo->outTangent = track.scaleOutTangents[j].x;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.scaleKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.scaleIndices[j];
                    frameInfo->value = track.scaleKeys[j].y;
                    frameInfo->inTangent = track.scaleInTangents[j].y;
                    frameInfo->outTangent = track.scaleOutTangents[j].y;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.scaleKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.scaleIndices[j];
                    frameInfo->value = track.scaleKeys[j].z;
                    frameInfo->inTangent = track.scaleInTangents[j].z;
                    frameInfo->outTangent = track.scaleOutTangents[j].z;
                    asset->frames.push_back(frameInfo);
                }

                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::ScaleX;
                    curveInfo->frameCount = static_cast<int>(track.scaleKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::ScaleY;
                    curveInfo->frameCount = static_cast<int>(track.scaleKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::ScaleZ;
                    curveInfo->frameCount = static_cast<int>(track.scaleKeys.size());
                    asset->curves.push_back(curveInfo);
                }
            }

            if (track.rotKeys.size()) {
                for (int j = 0; j < track.rotKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.rotIndices[j];
                    frameInfo->value = track.rotKeys[j].x;
                    frameInfo->inTangent = track.rotInTangents[j].x;
                    frameInfo->outTangent = track.rotOutTangents[j].x;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.rotKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.rotIndices[j];
                    frameInfo->value = track.rotKeys[j].y;
                    frameInfo->inTangent = track.rotInTangents[j].y;
                    frameInfo->outTangent = track.rotOutTangents[j].y;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.rotKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.rotIndices[j];
                    frameInfo->value = track.rotKeys[j].z;
                    frameInfo->inTangent = track.rotInTangents[j].z;
                    frameInfo->outTangent = track.rotOutTangents[j].z;
                    asset->frames.push_back(frameInfo);
                }
                for (int j = 0; j < track.rotKeys.size(); j++) {
                    std::shared_ptr<Assets::AnimationClipAsset::FrameInfo> frameInfo = std::make_shared<Assets::AnimationClipAsset::FrameInfo>();
                    frameInfo->frameIndex = track.rotIndices[j];
                    frameInfo->value = track.rotKeys[j].w;
                    frameInfo->inTangent = track.rotInTangents[j].w;
                    frameInfo->outTangent = track.rotOutTangents[j].w;
                    asset->frames.push_back(frameInfo);
                }

                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::RotationX;
                    curveInfo->frameCount = static_cast<int>(track.rotKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::RotationY;
                    curveInfo->frameCount = static_cast<int>(track.rotKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::RotationZ;
                    curveInfo->frameCount = static_cast<int>(track.rotKeys.size());
                    asset->curves.push_back(curveInfo);
                }
                {
                    std::shared_ptr<Assets::AnimationClipAsset::CurveInfo> curveInfo = std::make_shared<Assets::AnimationClipAsset::CurveInfo>();
                    curveInfo->index = i;
                    curveInfo->type = Assets::AnimationClipAsset::EAnimationCurveType::RotationW;
                    curveInfo->frameCount = static_cast<int>(track.rotKeys.size());
                    asset->curves.push_back(curveInfo);
                }
            }
            asset->paths.push_back(animationImportData->fbxRawBonePaths[animationImportData->trackToSkeletonMapTable[i].boneTreeIndex]);
        }
    }
    return true;
}
}}  // namespace Fbx::Builder