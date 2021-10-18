#include <filesystem>
#include "Animation.h"
#include "Builder/SkinnedMeshBuilder.h"
#include "Scene.h"

namespace Fbx { namespace Importer {

FbxTimeSpan Scene::GetAnimationTimeSpan(FbxNode* rootNode, FbxAnimStack* animStack) {
    auto options = Importer::GetInstance()->options;
    FbxTimeSpan animTimeSpan(FBXSDK_TIME_INFINITE, FBXSDK_TIME_MINUS_INFINITE);
    bool bUseDefault = options->animationLengthParseType == Options::EFBXAnimationLengthType::FBXALIT_ExportedTime || Maths::IsNearlyZero(sceneInfo->originalFbxFramerate, 1.e-4);
    if (bUseDefault) {
        animTimeSpan = animStack->GetLocalTimeSpan();
    } else {
        rootNode->GetAnimationInterval(animTimeSpan, animStack);
    }
    return animTimeSpan;
}

bool Scene::IsValidAnimationData(std::vector<FbxNode*>& sortedLinks, FbxNode* node, int& validTakeCount) {
    // If there are no valid links, then we cannot import the anim set
    if (sortedLinks.size() == 0) {
        return false;
    }

    validTakeCount = 0;

    int animStackCount = sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>();

    for (int animStackIndex = 0; animStackIndex < animStackCount; animStackIndex++) {
        FbxAnimStack* curAnimStack = sceneInfo->scene->GetSrcObject<FbxAnimStack>(animStackIndex);
        // set current anim stack
        sceneInfo->scene->SetCurrentAnimationStack(curAnimStack);

        // The animation timespan must use the original fbx framerate so the frame number match the DCC frame number
        FbxTimeSpan animTimeSpan = GetAnimationTimeSpan(sortedLinks[0], curAnimStack);
        if (animTimeSpan.GetDuration() <= 0) {
            LOG_ERROR(fmt::format("Animation {} doesn't have any keyframes.", ImporterHelper::UTF8ToNative(curAnimStack->GetName())));
            continue;
        }

        validTakeCount++;
        {
            bool bBlendCurveFound = false;

            // consider blendshape animation curve
            FbxGeometry* geometry = (FbxGeometry*)node->GetNodeAttribute();
            if (geometry) {
                int blendShapeDeformerCount = geometry->GetDeformerCount(FbxDeformer::eBlendShape);
                for (int blendShapeIndex = 0; blendShapeIndex < blendShapeDeformerCount; blendShapeIndex++) {
                    FbxBlendShape* blendShape = (FbxBlendShape*)geometry->GetDeformer(blendShapeIndex, FbxDeformer::eBlendShape);

                    int blendShapeChannelCount = blendShape->GetBlendShapeChannelCount();
                    for (int channelIndex = 0; channelIndex < blendShapeChannelCount; channelIndex++) {
                        FbxBlendShapeChannel* channel = blendShape->GetBlendShapeChannel(channelIndex);

                        if (channel) {
                            // Get the percentage of influence of the shape.
                            FbxAnimCurve* curve = geometry->GetShapeChannel(blendShapeIndex, channelIndex, (FbxAnimLayer*)curAnimStack->GetMember(0));
                            if (curve && curve->KeyGetCount() > 0) {
                                bBlendCurveFound = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return (validTakeCount != 0);
}

int Scene::GetMaxSampleRate(std::vector<FbxNode*>& sortedLinks, FbxNode* rootNode) {
    int maxStackResampleRate = 0;
    std::vector<int> curveAnimSampleRates;
    auto options = Importer::GetInstance()->options;
    int animStackCount = sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>();
    for (int animStackIndex = 0; animStackIndex < animStackCount; animStackIndex++) {
        FbxAnimStack* curAnimStack = sceneInfo->scene->GetSrcObject<FbxAnimStack>(animStackIndex);

        FbxTimeSpan animStackTimeSpan = GetAnimationTimeSpan(sortedLinks[0], curAnimStack);

        double animStackStart = animStackTimeSpan.GetStart().GetSecondDouble();
        double animStackStop = animStackTimeSpan.GetStop().GetSecondDouble();

        int animStackLayerCount = curAnimStack->GetMemberCount();
        for (int layerIndex = 0; layerIndex < animStackLayerCount; layerIndex++) {
            FbxAnimLayer* animLayer = (FbxAnimLayer*)curAnimStack->GetMember(layerIndex);
            for (int linkIndex = 0; linkIndex < sortedLinks.size(); linkIndex++) {
                FbxNode* currentLink = sortedLinks[linkIndex];
                ImporterHelper::GetNodeSampleRate(currentLink, animLayer, curveAnimSampleRates, true, false);
            }

            // it doens't matter whether you choose to import morphtarget or not
            // blendshape are always imported. Import morphtarget is only used for morphtarget for mesh
            {
                // consider blendshape animation curve
                ImporterHelper::GetNodeSampleRate(rootNode, animLayer, curveAnimSampleRates, false, true);
            }
        }
    }

    maxStackResampleRate = curveAnimSampleRates.size() > 0 ? 1 : maxStackResampleRate;
    // Find the lowest sample rate that will pass by all the keys from all curves
    for (int curveSampleRate : curveAnimSampleRates) {
        if (curveSampleRate >= MaxReferenceRate && maxStackResampleRate < curveSampleRate) {
            maxStackResampleRate = curveSampleRate;
        } else if (maxStackResampleRate < MaxReferenceRate) {
            int LeastCommonMultiplier = Maths::LeastCommonMultiplier(maxStackResampleRate, curveSampleRate);
            maxStackResampleRate = LeastCommonMultiplier != 0 ? LeastCommonMultiplier : Maths::Max(DEFAULT_SAMPLERATE, Maths::Max(maxStackResampleRate, curveSampleRate));
            if (maxStackResampleRate >= MaxReferenceRate) {
                maxStackResampleRate = MaxReferenceRate;
            }
        }
    }

    // Make sure we're not hitting 0 for samplerate
    if (maxStackResampleRate != 0) {
        // Make sure the resample rate is positive
        if (!(maxStackResampleRate >= 0)) {
            maxStackResampleRate *= -1;
        }
        return maxStackResampleRate;
    }

    return DEFAULT_SAMPLERATE;
}

bool Scene::ValidateAnimStack(std::vector<FbxNode*>& sortedLinks, FbxNode* rootNode, FbxAnimStack* curAnimStack, int resampleRate, bool bImportMorph, FbxTimeSpan& animTimeSpan) {
    // set current anim stack
    sceneInfo->scene->SetCurrentAnimationStack(curAnimStack);

    LOG_INFO(fmt::format("Processing animation {}.", ImporterHelper::UTF8ToNative(curAnimStack->GetName())));

    bool bValidAnimStack = true;

    animTimeSpan = GetAnimationTimeSpan(sortedLinks[0], curAnimStack);

    // if no duration is found, return false
    if (animTimeSpan.GetDuration() <= 0) {
        return false;
    }

    auto options = Importer::GetInstance()->options;
    // only add morph time if not setrange. If Set Range there is no reason to override time
    if (bImportMorph) {
        // consider blendshape animation curve
        FbxGeometry* geometry = (FbxGeometry*)rootNode->GetNodeAttribute();
        if (geometry) {
            int blendShapeDeformerCount = geometry->GetDeformerCount(FbxDeformer::eBlendShape);
            for (int blendShapeIndex = 0; blendShapeIndex < blendShapeDeformerCount; blendShapeIndex++) {
                FbxBlendShape* blendShape = (FbxBlendShape*)geometry->GetDeformer(blendShapeIndex, FbxDeformer::eBlendShape);

                int blendShapeChannelCount = blendShape->GetBlendShapeChannelCount();
                for (int channelIndex = 0; channelIndex < blendShapeChannelCount; channelIndex++) {
                    FbxBlendShapeChannel* channel = blendShape->GetBlendShapeChannel(channelIndex);

                    if (channel) {
                        // Get the percentage of influence of the shape.
                        FbxAnimCurve* curve = geometry->GetShapeChannel(blendShapeIndex, channelIndex, (FbxAnimLayer*)curAnimStack->GetMember(0));
                        if (curve && curve->KeyGetCount() > 0) {
                            FbxTimeSpan tmpAnimSpan;

                            if (curve->GetTimeInterval(tmpAnimSpan)) {
                                bValidAnimStack = true;
                                // update animation interval to include morph target range
                                animTimeSpan.UnionAssignment(tmpAnimSpan);
                            }
                        }
                    }
                }
            }
        }
    }

    return bValidAnimStack;
}

void Scene::ImportBlendShapeCurves(std::shared_ptr<AnimationImportData> animImportSettings, FbxAnimStack* curAnimStack, int& outKeyCount) {
    outKeyCount = 0;
    auto skeleton = animImportSettings->skeleton;
    auto options = Importer::GetInstance()->options;

    // consider blendshape animation curve
    FbxGeometry* geometry = (FbxGeometry*)(animImportSettings->rootNode)->GetNodeAttribute();
    if (geometry) {
        int blendShapeDeformerCount = geometry->GetDeformerCount(FbxDeformer::eBlendShape);

        for (int blendShapeIndex = 0; blendShapeIndex < blendShapeDeformerCount; blendShapeIndex++) {
            FbxBlendShape* blendShape = (FbxBlendShape*)geometry->GetDeformer(blendShapeIndex, FbxDeformer::eBlendShape);

            const int blendShapeChannelCount = blendShape->GetBlendShapeChannelCount();

            std::string blendShapeName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(blendShape->GetName()));

            // see below where this is used for explanation...
            const bool bMightBeBadMAXFile = blendShapeName == "Morpher";
            for (int channelIndex = 0; channelIndex < blendShapeChannelCount; channelIndex++) {
                FbxBlendShapeChannel* channel = blendShape->GetBlendShapeChannel(channelIndex);

                if (channel) {
                    std::string channelName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(channel->GetName()));
                    // Maya adds the name of the blendshape and an underscore or point to the front of the channel name, so remove it
                    // Also avoid to endup with a empty name, we prefer having the Blendshapename instead of nothing
                    if (channelName.find(blendShapeName) == 0 && channelName.size() > blendShapeName.size()) {
                        channelName = channelName.substr(channelName.size() - (blendShapeName.size() + 1));
                    }

                    if (bMightBeBadMAXFile) {
                        FbxShape* targetShape = channel->GetTargetShapeCount() > 0 ? channel->GetTargetShape(0) : nullptr;
                        if (targetShape) {
                            std::string targetShapeName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(targetShape->GetName()));
                            channelName = targetShapeName.empty() ? channelName : targetShapeName;
                        }
                    }

                    FbxAnimCurve* curve = geometry->GetShapeChannel(blendShapeIndex, channelIndex, (FbxAnimLayer*)curAnimStack->GetMember(0));
                    if (ImporterHelper::ShouldImportCurve(curve, options->bDoNotParseCurveWithZero)) {
                        // now see if we have one already exists. If so, just overwrite that. if not, add new one.
                        // TODO
                        // if (ImportCurveToAnimSequence(animImportSettings.DestSeq, *channelName, Curve, 0, animImportSettings->animTimeSpan, 0.01f /** for some reason blend shape values are coming as 100 scaled **/)) {
                        //    outKeyCount = std::max(outKeyCount, Curve->KeyGetCount());
                        //    // this one doesn't reset Material curve to false, it just accumulate if true.
                        //    MySkeleton->AccumulateCurveMetaData(*channelName, false, true);
                        //}

                    } else {
                        LOG_WARN(fmt::format("Animation {} contains invalid value for channel {}.", ImporterHelper::UTF8ToNative(curAnimStack->GetName()), channelName));
                    }
                }
            }
        }
    }
}

void Scene::ImportBoneTracks(Builder::SkinnedMeshSkeleton skeleton, std::shared_ptr<AnimationImportData> importData, FbxNode* rootNode, FbxAnimStack* animStack, const int resampleRate, int& outTotalNumKeys) {
    LOG_INFO("Parsing bone tracks ...");
    outTotalNumKeys = 0;
    auto options = Importer::GetInstance()->options;
    const bool bPreserveLocalTransform = options->bPreserveLocalTransform;

    const FbxTimeSpan& animTimeSpan = importData->animTimeSpan;

    bool bIsRigidMeshAnimation = false;
    if (importData->sortedLinks.size() > 0) {
        for (int boneIdx = 0; boneIdx < importData->sortedLinks.size(); boneIdx++) {
            FbxNode* link = importData->sortedLinks[boneIdx];
            if (link->GetMesh() && link->GetMesh()->GetDeformerCount(FbxDeformer::eSkin) == 0) {
                bIsRigidMeshAnimation = true;
                break;
            }
        }
    }

    const int numSamplingFrame = Maths::RoundToInt((animTimeSpan.GetDuration().GetSecondDouble() * resampleRate));
    // Set the time increment from the re-sample rate
    FbxTime timeIncrement = 0;
    timeIncrement.SetSecondDouble(1.0 / ((double)(resampleRate)));

    // Add a threshold when we compare if we have reach the end of the animation
    const FbxTime timeComparisonThreshold = (KINDA_SMALL_NUMBER * static_cast<float>(FBXSDK_TC_SECOND));

    const FbxTime timeDeltaForDerivative = (0.01 * static_cast<float>(FBXSDK_TC_MILLISECOND));

    for (int sourceTrackIdx = 0; sourceTrackIdx < importData->fbxRawBoneNames.size(); sourceTrackIdx++) {
        int numKeysForTrack = 0;

        // see if it's found in Skeleton
        std::string boneName = importData->fbxRawBoneNames[sourceTrackIdx];
        int boneTreeIndex = skeleton.FindRawBoneIndex(boneName);

        LOG_INFO(fmt::format("Processing track {}, progress {}/{}, current track contains {} keyframes.", boneName, sourceTrackIdx + 1, importData->fbxRawBoneNames.size(), numSamplingFrame + 1));

        if (boneTreeIndex != -1) {
            bool bSuccess = true;

            FRawAnimSequenceTrack rawTrack;
            rawTrack.posKeys.clear();
            rawTrack.rotKeys.clear();
            rawTrack.scaleKeys.clear();

            FbxNode* link = importData->sortedLinks[sourceTrackIdx];
            bool bHasPosCurve = !!link->LclTranslation.GetCurveNode();
            bool bHasScaleCurve = !!link->LclScaling.GetCurveNode();
            bool bHasRotCurve = !!link->LclRotation.GetCurveNode();
            if (!bHasPosCurve && !bHasScaleCurve && !bHasRotCurve) {
                continue;
            }

            FbxNode* linkParent = link->GetParent();
            int frameIndex = 0;
            for (FbxTime curTime = animTimeSpan.GetStart(), curTime1 = curTime - timeDeltaForDerivative, curTime2 = curTime + timeDeltaForDerivative; curTime < (animTimeSpan.GetStop() + timeComparisonThreshold);
                 curTime += timeIncrement, curTime1 += timeIncrement, curTime2 += timeIncrement, frameIndex++) {
                auto Sample = [this, &link, &bSuccess, &boneName, &bPreserveLocalTransform, &linkParent, &boneTreeIndex, &rawTrack](FbxTime curTime, Maths::Transform& localTransform) -> bool {
                    // save global trasnform
                    FbxAMatrix globalFbxMatrix = link->EvaluateGlobalTransform(curTime);
                    // we'd like to verify this before going to Transform.
                    // currently transform has tons of NaN check, so it will crash there
                    glm::mat4 globalMatrix = FbxDataConverter::ConvertMatrix(globalFbxMatrix);
                    /*if (glm::isnan(globalMatrix)) {
                        bSuccess = false;
                        LOG_INFO(fmt::format("Track {} contains invalid value.", boneName));
                        break;
                    }*/

                    Maths::Transform globalTransform = FbxDataConverter::ConvertTransform(globalFbxMatrix);
                    if (glm::all(glm::isnan(globalTransform.translation)) || glm::all(glm::isnan(globalTransform.scale)) || glm::all(glm::isnan(globalTransform.rotation))) {
                        bSuccess = false;
                        LOG_INFO(fmt::format("Track {} contains invalid value.", boneName));
                        return false;
                    }

                    if (!bPreserveLocalTransform && linkParent) {
                        // I can't rely on LocalMatrix. I need to recalculate quaternion/scale based on global transform if Parent exists
                        FbxAMatrix ParentGlobalMatrix = link->GetParent()->EvaluateGlobalTransform(curTime);
                        if (boneTreeIndex != 0) {
                            ParentGlobalMatrix = ParentGlobalMatrix;
                        }
                        Maths::Transform parentGlobalTransform = FbxDataConverter::ConvertTransform(ParentGlobalMatrix);
                        // In case we do a scene import we need to add the skeletal mesh root node matrix to the parent link.
                        // if (boneTreeIndex == 0 && importData->rootNode != nullptr) {
                        //    // In the case of a rigidmesh animation we have to use the skeletalMeshRootNode position at zero since the mesh can be animate.
                        //    FbxAMatrix globalSkeletalNodeFbx = bIsRigidMeshAnimation ? rootNode->EvaluateGlobalTransform(0) : rootNode->EvaluateGlobalTransform(curTime);
                        //    FTransform globalSkeletalNode = FbxDataConverter::ConvertTransform(globalSkeletalNodeFbx);
                        //    parentGlobalTransform.SetFromMatrix(parentGlobalTransform.ToMatrixWithScale() * globalSkeletalNode.ToMatrixWithScale());
                        //}
                        localTransform.SetFromMatrix(glm::inverse(parentGlobalTransform.ToMatrixWithScale()) * globalTransform.ToMatrixWithScale());
                    } else {
                        FbxAMatrix& localMatrix = link->EvaluateLocalTransform(curTime);
                        FbxVector4 newLocalT = localMatrix.GetT();
                        FbxVector4 newLocalS = localMatrix.GetS();
                        FbxQuaternion newLocalQ = localMatrix.GetQ();

                        localTransform.translation = FbxDataConverter::ConvertPos(newLocalT);
                        localTransform.scale = FbxDataConverter::ConvertScale(newLocalS);
                        localTransform.rotation = FbxDataConverter::ConvertRotToQuat(newLocalQ);
                    }

                    if (sceneInfo->scaleFactor != 1.0) {
                        localTransform.translation *= sceneInfo->scaleFactor;
                    }

                    if (glm::all(glm::isnan(localTransform.translation)) || glm::all(glm::isnan(localTransform.scale)) || glm::all(glm::isnan(localTransform.rotation))) {
                        bSuccess = false;
                        LOG_INFO(fmt::format("Track {} contains invalid value.", boneName));
                        return false;
                    }

                    return true;
                };

                Maths::Transform transform;

                if (!Sample(curTime, transform)) {
                    break;
                }
                if (bHasScaleCurve) {
                    rawTrack.scaleKeys.push_back(transform.scale);
                    rawTrack.scaleInTangents.push_back(glm::vec3(0, 0, 0));
                    rawTrack.scaleOutTangents.push_back(glm::vec3(0, 0, 0));
                }
                if (bHasPosCurve) {
                    rawTrack.posKeys.push_back(transform.translation);
                    rawTrack.posInTangents.push_back(glm::vec3(0, 0, 0));
                    rawTrack.posOutTangents.push_back(glm::vec3(0, 0, 0));
                }
                if (bHasRotCurve) {
                    if (rawTrack.rotKeys.size()) {
                        if (glm::dot(rawTrack.rotKeys.back(), transform.rotation) < 0) {
                            transform.rotation = -transform.rotation;
                        }
                    }
                    rawTrack.rotKeys.push_back(transform.rotation);
                    rawTrack.rotInTangents.push_back(glm::vec3(0, 0, 0));
                    rawTrack.rotOutTangents.push_back(glm::vec3(0, 0, 0));
                }

                Maths::Transform transform1, transform2;
                if (frameIndex != 0 && !Sample(curTime1, transform1)) {
                    break;
                }
                if (!Sample(curTime2, transform2)) {
                    break;
                }

                if (frameIndex != 0) {
                    if (bHasPosCurve) {
                        rawTrack.posInTangents.back() = (rawTrack.posKeys.back() - transform1.translation) / (float)(timeDeltaForDerivative.GetSecondDouble());
                    }
                    if (bHasScaleCurve) {
                        rawTrack.scaleInTangents.back() = (rawTrack.scaleKeys.back() - transform1.scale) / (float)(timeDeltaForDerivative.GetSecondDouble());
                    }
                    if (bHasRotCurve) {
                        if (rawTrack.rotKeys.size()) {
                            if (glm::dot(transform.rotation, transform1.rotation) < 0) {
                                transform1.rotation = -transform1.rotation;
                            }
                        }
                        rawTrack.rotInTangents.back() = (rawTrack.rotKeys.back() - transform1.rotation) / (float)(timeDeltaForDerivative.GetSecondDouble());
                    }
                }

                if (bHasPosCurve) {
                    rawTrack.posOutTangents.back() = (transform2.translation - rawTrack.posKeys.back()) / (float)(timeDeltaForDerivative.GetSecondDouble());
                }
                if (bHasScaleCurve) {
                    rawTrack.scaleOutTangents.back() = (transform2.scale - rawTrack.scaleKeys.back()) / (float)(timeDeltaForDerivative.GetSecondDouble());
                }
                if (bHasRotCurve) {
                    if (rawTrack.rotKeys.size()) {
                        if (glm::dot(transform.rotation, transform2.rotation) < 0) {
                            transform2.rotation = -transform2.rotation;
                        }
                    }
                    rawTrack.rotOutTangents.back() = (transform2.rotation - rawTrack.rotKeys.back()) / (float)(timeDeltaForDerivative.GetSecondDouble());
                }

                ++numKeysForTrack;
            }

            if (bSuccess) {
                // add new track
                int newTrackIdx = -1;
                {
                    const int skeletonIndex = skeleton.FindRawBoneIndex(boneName);

                    if (skeletonIndex != -1) {
                        auto found = std::find(importData->animationTrackNames.begin(), importData->animationTrackNames.end(), boneName);
                        int trackIndex = found == importData->animationTrackNames.end() ? -1 : std::distance(found, importData->animationTrackNames.begin());
                        if (trackIndex != -1) {
                            if (importData->rawAnimationData.size() <= trackIndex) {
                                importData->rawAnimationData.resize(trackIndex + 1);
                            }
                            importData->rawAnimationData[trackIndex] = rawTrack;
                            newTrackIdx = trackIndex;
                        } else {
                            ASSERT(importData->animationTrackNames.size() == importData->rawAnimationData.size());
                            importData->animationTrackNames.push_back(boneName);
                            trackIndex = importData->animationTrackNames.size() - 1;
                            importData->trackToSkeletonMapTable.push_back(FTrackToSkeletonMap(skeletonIndex));
                            importData->rawAnimationData.push_back(rawTrack);
                            newTrackIdx = trackIndex;
                        }
                    } else {
                        newTrackIdx = -1;
                    }
                }
            }
        }

        outTotalNumKeys = Maths::Max(outTotalNumKeys, numKeysForTrack);
    }

    importData->numFrames = outTotalNumKeys;
}

std::vector<std::shared_ptr<AnimationImportData>> Scene::ImportAnimations(FbxNode* rootNode, std::vector<FbxNode*>& sortedLinks, const std::vector<std::string>& fbxRawBoneNames, const std::vector<std::string>& fbxRawBonePaths, Builder::SkinnedMeshSkeleton& skeleton) {
    std::vector<std::shared_ptr<AnimationImportData>> result;
    int validTakeCount = 0;
    if (IsValidAnimationData(sortedLinks, rootNode, validTakeCount) == false) {
        return result;
    }
    auto options = Importer::GetInstance()->options;
    int resampleRate = DEFAULT_SAMPLERATE;
    if (options->bResample) {
        if (options->resampleRate > 0) {
            resampleRate = options->resampleRate;
        } else {
            // For FBX data, "Frame Rate" is just the speed at which the animation is played back.  It can change
            // arbitrarily, and the underlying data can stay the same.  What we really want here is the Sampling Rate,
            // ie: the number of animation keys per second.  These are the individual animation curve keys
            // on the FBX nodes of the skeleton.  So we loop through the nodes of the skeleton and find the maximum number
            // of keys that any node has, then divide this by the total length (in seconds) of the animation to find the
            // sampling rate of this set of data

            // we want the maximum resample rate, so that we don't lose any precision of fast anims,
            // and don't mind creating lerped frames for slow anims
            int bestResampleRate = GetMaxSampleRate(sortedLinks, rootNode);

            if (bestResampleRate > 0) {
                resampleRate = bestResampleRate;
            }
        }
    }

    int animStackCount = sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>();
    for (int animStackIndex = 0; animStackIndex < animStackCount; animStackIndex++) {
        FbxAnimStack* curAnimStack = sceneInfo->scene->GetSrcObject<FbxAnimStack>(animStackIndex);

        FbxTimeSpan animTimeSpan = GetAnimationTimeSpan(sortedLinks[0], curAnimStack);
        bool bValidAnimStack = ValidateAnimStack(sortedLinks, rootNode, curAnimStack, resampleRate, options->bParseMorphTargets, animTimeSpan);
        // no animation
        if (!bValidAnimStack) {
            continue;
        }

        std::string sequenceName = ImporterHelper::UTF8ToNative(curAnimStack->GetName());
        sequenceName = Utils::SanitizeObjectName(sequenceName);

        // do import

        {
            FbxTime sequenceLength = animTimeSpan.GetDuration();
            std::vector<FbxNode*> ignoredSortedLinks;

            if (animations.find(curAnimStack) == animations.end()) {
                animations.emplace(curAnimStack, std::vector<FbxNode*>());
            }
            for (auto link : sortedLinks) {
                if (std::find(animations[curAnimStack].begin(), animations[curAnimStack].end(), link) == animations[curAnimStack].end()) {
                    animations[curAnimStack].push_back(link);
                }
            }

            auto animationImportData = std::make_shared<AnimationImportData>(skeleton, sortedLinks, ignoredSortedLinks, fbxRawBoneNames, fbxRawBonePaths, rootNode, animTimeSpan);
            animationImportData->sequenceLength = Maths::Max(sequenceLength.GetSecondDouble(), 1.0 / DEFAULT_SAMPLERATE);
            animationImportData->importFileFramerate = sceneInfo->originalFbxFramerate;
            animationImportData->importResampleFramerate = resampleRate;
            animationImportData->sequenceName = sequenceName;

            //
            // import blend shape curves
            //
            int curveAttributeKeyCount = 0;
            ImportBlendShapeCurves(animationImportData, curAnimStack, curveAttributeKeyCount);

            std::vector<std::string> curvesNotFound;
            if (options->bParseCustomAttribute) {
                int customAttributeKeyCount = 0;

                // TODO
                // ImportAnimationCustomAttribute(animationImportData, customAttributeKeyCount, curvesNotFound);

                curveAttributeKeyCount = Maths::Max(curveAttributeKeyCount, customAttributeKeyCount);
            }
            // importing custom attribute END
            int totalNumKeys = 0;
            // import animation
            if (options->bParseBoneTracks) {
                ImportBoneTracks(skeleton, animationImportData, rootNode, curAnimStack, resampleRate, totalNumKeys);
            } else if (curveAttributeKeyCount > 0) {
                animationImportData->numFrames = curveAttributeKeyCount;
            }
            result.push_back(animationImportData);
        }
    }

    for (auto& animation : result) {
        for (auto& rawAnim : animation->rawAnimationData) {
            for (auto& scale3D : rawAnim.scaleKeys) {
                if (Maths::IsNearlyZero(scale3D.x)) {
                    scale3D.x = 0.f;
                }
                if (Maths::IsNearlyZero(scale3D.y)) {
                    scale3D.y = 0.f;
                }
                if (Maths::IsNearlyZero(scale3D.z)) {
                    scale3D.z = 0.f;
                }
            }

            // make sure Rotation part is normalized before compress
            for (auto& rotation : rawAnim.rotKeys) {
                rotation = glm::normalize(rotation);
            }
        }

        if (animation->rawAnimationData.size()) {
            ImporterHelper::CompressRawAnimData(animation->rawAnimationData, animation->numFrames, animation->sequenceName, 0.0001f, 0.0003f);
        }
    }

    std::vector<std::shared_ptr<AnimationImportData>> nonEmptyResult;
    for (auto& animation : result) {
        if (animation->rawAnimationData.size()) {
            nonEmptyResult.push_back(animation);
        }
    }
    return nonEmptyResult;
}

bool ImporterHelper::ShouldImportCurve(FbxAnimCurve* curve, bool bDoNotImportWithZeroValues) {
    if (curve && curve->KeyGetCount() > 0) {
        if (bDoNotImportWithZeroValues) {
            for (int keyIndex = 0; keyIndex < curve->KeyGetCount(); keyIndex++) {
                if (!Maths::IsNearlyZero(curve->KeyGetValue(keyIndex))) {
                    return true;
                }
            }
        } else {
            return true;
        }
    }

    return false;
}
}}  // namespace Fbx::Importer