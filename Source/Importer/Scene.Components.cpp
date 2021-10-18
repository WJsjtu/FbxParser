#include "Builder/AnimationClipBuilder.h"
#include "Builder/SkinnedMeshBuilder.h"
#include "Mesh.h"
#include "Scene.h"

namespace Fbx { namespace Importer {

void Scene::ProcessMesh() {
    for (const auto& kvp : sceneInfo->meshInfo) {
        if (kvp.second.bIsSkelMesh) {
            continue;
        }
        FbxNode* fbxNode = kvp.first;
        // find the mesh by its name
        FbxMesh* fbxMesh = fbxNode->GetMesh();
        if (!fbxMesh) {
            continue;
        }
        auto sceneNodeKV = nodeMap.find(fbxNode);
        if (sceneNodeKV == nodeMap.end()) {
            continue;
        }
        std::shared_ptr<Objects::Entity> sceneNode = sceneNodeKV->second;

        std::shared_ptr<MeshImportData> importData = ImportMesh(fbxNode, fbxMesh, false);
        if (!importData) {
            continue;
        }

        Builder::MeshBuildSettings buildOptions;
        auto options = Importer::GetInstance()->options;
        if (options->computeNormal == Options::NormalOptions::ImportOrComputeNormal) {
            buildOptions.bRecomputeNormals = !importData->bHasNormals;
        } else if (options->computeNormal == Options::NormalOptions::ForceComputeNormal) {
            buildOptions.bRecomputeNormals = true;
        } else {
            buildOptions.bRecomputeNormals = false;
        }
        if (options->computeTangent == Options::TangentOptions::ImportOrComputeTangent) {
            buildOptions.bRecomputeTangents = !importData->bHasTangents;
        } else if (options->computeTangent == Options::TangentOptions::ForceComputeTangent) {
            buildOptions.bRecomputeTangents = true;
        } else {
            buildOptions.bRecomputeTangents = false;
        }

        std::shared_ptr<Builder::Mesh<>> meshAsset = std::make_shared<Builder::Mesh<>>();
        meshAsset->fbxMesh = fbxMesh;
        meshAsset->name = importData->name;
        // Store whether or not this mesh has vertex colors
        meshAsset->bHasVertexColors = importData->bHasVertexColors;
        meshAsset->numTexCoords = importData->numTexCoords;
        meshAsset->boundingSphere = importData->boundingSphere;
        meshAsset->boundingBox = importData->boundingBox;

        // process materials from import data
        Builder::SkinnedMeshHelper::ProcessImportMeshMaterials(meshAsset->materials, importData);

        Builder::MeshBuilder builder;
        bool bBuildSuccess = builder.Build(meshAsset, importData, sceneInfo, buildOptions);
        if (!bBuildSuccess) {
            continue;
        }

        if (buildOptions.bRecomputeNormals) {
            meshAsset->bHasNormals = true;
        } else if (importData->bHasNormals) {
            meshAsset->bHasNormals = true;
        } else {
            meshAsset->bHasNormals = false;
        }

        if (buildOptions.bRecomputeTangents) {
            meshAsset->bHasTangents = true;
        } else if (importData->bHasTangents) {
            meshAsset->bHasTangents = true;
        } else {
            meshAsset->bHasTangents = false;
        }

        std::shared_ptr<Objects::MeshRenderer> meshRenderer = std::make_shared<Objects::MeshRenderer>();
        meshRenderer->mesh = ConvertToMesh(meshAsset);
        sceneNode->components.push_back(meshRenderer);
    }
}

void Scene::ProcessSkinnedMesh() {
    // get bindpose and clusters from FBX skeleton
    auto sdkManager = Importer::GetInstance()->sdkManager;
    // let's put the elements to their bind pose! (and we restore them after
    // we have built the ClusterInformation.
    int default_NbPoses = sdkManager->GetBindPoseCount(sceneInfo->scene);
    // If there are no BindPoses, the following will generate them.
    // SdkManager->CreateMissingBindPoses(Scene);

    // if we created missing bind poses, update the number of bind poses
    int NbPoses = sdkManager->GetBindPoseCount(sceneInfo->scene);

    if (NbPoses != default_NbPoses) {
        LOG_WARN("Missing bind pose for skinned mesh on scene.");
    }

    for (const auto& kvp : sceneInfo->meshInfo) {
        if (!kvp.second.bIsSkelMesh) {
            continue;
        }
        FbxNode* fbxNode = kvp.first;
        // find the mesh by its name
        FbxMesh* fbxMesh = fbxNode->GetMesh();
        if (!fbxMesh) {
            continue;
        }
        auto sceneNodeKV = nodeMap.find(fbxNode);
        if (sceneNodeKV == nodeMap.end()) {
            continue;
        }
        std::shared_ptr<Objects::Entity> sceneNode = sceneNodeKV->second;

        std::shared_ptr<MeshImportData> importData = ImportMesh(fbxNode, fbxMesh, true);
        if (!importData) {
            continue;
        }

        Builder::MeshBuildSettings buildOptions;
        auto options = Importer::GetInstance()->options;
        if (options->computeNormal == Options::NormalOptions::ImportOrComputeNormal) {
            buildOptions.bRecomputeNormals = !importData->bHasNormals;
        } else if (options->computeNormal == Options::NormalOptions::ForceComputeNormal) {
            buildOptions.bRecomputeNormals = true;
        } else {
            buildOptions.bRecomputeNormals = false;
        }
        if (options->computeTangent == Options::TangentOptions::ImportOrComputeTangent) {
            buildOptions.bRecomputeTangents = !importData->bHasTangents;
        } else if (options->computeTangent == Options::TangentOptions::ForceComputeTangent) {
            buildOptions.bRecomputeTangents = true;
        } else {
            buildOptions.bRecomputeTangents = false;
        }

        std::shared_ptr<Builder::SkinnedMesh> skinnedMeshAsset = std::make_shared<Builder::SkinnedMesh>();
        skinnedMeshAsset->fbxMesh = fbxMesh;
        skinnedMeshAsset->name = importData->name;
        // Store whether or not this mesh has vertex colors
        skinnedMeshAsset->bHasVertexColors = importData->bHasVertexColors;
        skinnedMeshAsset->numTexCoords = importData->numTexCoords;
        skinnedMeshAsset->boundingSphere = importData->boundingSphere;
        skinnedMeshAsset->boundingBox = importData->boundingBox;

        // process materials from import data
        Builder::SkinnedMeshHelper::ProcessImportMeshMaterials(skinnedMeshAsset->materials, importData);

        // process reference skeleton from import data
        int skeletalDepth = 0;
        if (!Builder::SkinnedMeshHelper::ProcessImportSkinnedMeshSkeleton(skinnedMeshAsset->skeleton, skeletalDepth, importData)) {
            continue;
        }

        // process bone influences from import data
        Builder::SkinnedMeshHelper::ProcessImportMeshInfluences(importData, skinnedMeshAsset->name);

        Builder::SkinnedMeshBuilder builder;
        bool bBuildSuccess = builder.Build(skinnedMeshAsset, importData, sceneInfo, buildOptions);
        if (!bBuildSuccess) {
            continue;
        }
        skinnedMeshAsset->CalculateInvRefMatrices();

        if (buildOptions.bRecomputeNormals) {
            skinnedMeshAsset->bHasNormals = true;
        } else if (importData->bHasNormals) {
            skinnedMeshAsset->bHasNormals = true;
        } else {
            skinnedMeshAsset->bHasNormals = false;
        }

        if (buildOptions.bRecomputeTangents) {
            skinnedMeshAsset->bHasTangents = true;
        } else if (importData->bHasTangents) {
            skinnedMeshAsset->bHasTangents = true;
        } else {
            skinnedMeshAsset->bHasTangents = false;
        }

        std::shared_ptr<Objects::MeshRenderer> skinnedMeshRenderer = std::make_shared<Objects::MeshRenderer>();
        std::vector<std::shared_ptr<Assets::AnimationClipAsset>> animationClips;
        std::shared_ptr<Assets::SkeletonAsset> skeletonAsset = nullptr;
        {
            int boneNum = skinnedMeshAsset->skeleton.GetBoneNum();
            if (boneNum) {
                auto& root = skinnedMeshAsset->skeleton.GetBoneInfo()[0];
                if (skeletonAssets.find(root.source) == skeletonAssets.end()) {
                    skeletonAsset = skeletonAssets[root.source] = Builder::ConvertToSkeleton(skinnedMeshAsset->skeleton, root, true);
                    if (options->bParseAnimations && sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>() > 0) {
                        auto animations = ImportAnimations(fbxNode, importData->sortedLinks, skeletonAsset->names, skeletonAsset->paths, skinnedMeshAsset->skeleton);
                        Builder::AnimationBuildSettings animationBuildOptions;
                        Builder::AnimationClipBuilder animationBuilder;
                        for (auto animation : animations) {
                            auto animationAsset = std::make_shared<Assets::AnimationClipAsset>();
                            animationBuilder.Build(animationAsset, animation, animationBuildOptions);
                            animationClips.push_back(animationAsset);
                        }
                    }
                } else {
                    skeletonAsset = skeletonAssets[root.source];
                }
                if (root.source->GetParent() && nodeMap.find(root.source->GetParent()) != nodeMap.end()) {
                    std::shared_ptr<Objects::Skeleton> skeletonFound = nullptr;
                    for (int i = 0; i < nodeMap[root.source->GetParent()]->components.size(); i++) {
                        Objects::EComponentType compType = nodeMap[root.source->GetParent()]->components[i]->GetTypeName();
                        if (compType == Objects::EComponentType::Skeleton) {
                            skeletonFound = std::static_pointer_cast<Objects::Skeleton>(nodeMap[root.source->GetParent()]->components[i]);
                            break;
                        }
                    }
                    if (skeletonFound == nullptr) {
                        std::shared_ptr<Objects::Skeleton> skeleton = std::make_shared<Objects::Skeleton>();
                        skeleton->skeleton = skeletonAsset;
                        skinnedMeshRenderer->skeleton = skeleton;
                        nodeMap[root.source->GetParent()]->components.push_back(skeleton);
                        std::shared_ptr<Objects::Animation> animation = std::make_shared<Objects::Animation>();
                        animation->skeleton = skeleton;
                        for (auto& animationClip : animationClips) {
                            std::string clipName = animationClip->name;
                            int renameCount = 0;
                            while (animation->animationClips.find(clipName) != animation->animationClips.end()) {
                                clipName = animationClip->name + "_" + std::to_string(++renameCount);
                            }
                            animation->animationClips[clipName] = animationClip;
                        }
                        nodeMap[root.source->GetParent()]->components.push_back(animation);
                    } else {
                        skinnedMeshRenderer->skeleton = skeletonFound;
                    }
                }
            }
        }

        skinnedMeshRenderer->mesh = ConvertToMesh(skinnedMeshAsset);
        sceneNode->components.push_back(skinnedMeshRenderer);
    }
}

void Scene::ProcessAnimation() {
    auto options = Importer::GetInstance()->options;
    FbxNode* sceneRoot = sceneInfo->scene->GetRootNode();

    std::function<void(FbxNode*, std::vector<FbxNode*>&)> RecursiveBuildSkeletonByNode = [&](FbxNode* link, std::vector<FbxNode*>& outSortedLinks) {
        outSortedLinks.push_back(link);
        for (int childIndex = 0; childIndex < link->GetChildCount(); childIndex++) {
            RecursiveBuildSkeletonByNode(link->GetChild(childIndex), outSortedLinks);
        }
    };

    for (int childIndex = 0; childIndex < sceneRoot->GetChildCount(); childIndex++) {
        auto root = sceneRoot->GetChild(childIndex);
        std::vector<FbxNode*> allLinks;

        RecursiveBuildSkeletonByNode(root, allLinks);
        for (auto& animation : animations) {
            for (FbxNode* node : animation.second) {
                auto found = std::find(allLinks.begin(), allLinks.end(), node);
                if (found != allLinks.end()) {
                    allLinks.erase(found);
                }
            }
        }

        if (allLinks.size()) {
            std::map<FbxNode*, std::vector<FbxNode*>> rootLinks;
            for (auto link : allLinks) {
                FbxNode* current = link;
                while (current->GetParent() && std::find(allLinks.begin(), allLinks.end(), current->GetParent()) != allLinks.end()) {
                    current = current->GetParent();
                }
                if (rootLinks.find(current) == rootLinks.end()) {
                    std::vector<FbxNode*> links;
                    links.push_back(link);
                    rootLinks.emplace(current, std::move(links));
                } else {
                    auto& links = rootLinks[current];
                    if (std::find(links.begin(), links.end(), link) == links.end()) {
                        links.push_back(link);
                    }
                }
            }

            for (auto kvp : rootLinks) {
                auto sortedLinks = kvp.second;
                std::vector<MeshImportData::Bone> bones;
                FbxArray<FbxAMatrix> linkMatrices;
                linkMatrices.Grow(static_cast<int>(sortedLinks.size()));
                linkMatrices[0].SetIdentity();
                for (int linkIndex = 0; linkIndex < sortedLinks.size(); linkIndex++) {
                    // Add a bone for each FBX Link
                    bones.push_back(MeshImportData::Bone());
                    FbxNode* link = sortedLinks[linkIndex];

                    int parentIndex = -1;  // base value for root if no parent found
                    if (linkIndex != 0) {
                        const FbxNode* linkParent = link->GetParent();
                        // get the link parent index.
                        for (int parentLinkIndex = 0; parentLinkIndex < linkIndex; parentLinkIndex++)  // <LinkIndex because parent is guaranteed to be before child in
                                                                                                       // sortedLink
                        {
                            FbxNode* Otherlink = sortedLinks[parentLinkIndex];
                            if (Otherlink == linkParent) {
                                parentIndex = parentLinkIndex;
                                break;
                            }
                        }
                    }

                    linkMatrices[linkIndex] = link->EvaluateGlobalTransform();

                    FbxVector4 localLinkT;
                    FbxQuaternion localLinkQ;
                    FbxVector4 localLinkS;

                    if (linkIndex) {
                        FbxAMatrix matrix;
                        matrix = linkMatrices[parentIndex].Inverse() * linkMatrices[linkIndex];
                        localLinkT = matrix.GetT();
                        localLinkQ = matrix.GetQ();
                        localLinkS = matrix.GetS();
                    } else  // skeleton root
                    {
                        // for root, this is global coordinate
                        localLinkT = linkMatrices[linkIndex].GetT();
                        localLinkQ = linkMatrices[linkIndex].GetQ();
                        localLinkS = linkMatrices[linkIndex].GetS();
                    }

                    // set bone
                    MeshImportData::Bone& bone = bones[linkIndex];
                    std::string boneName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(link->GetName()));
                    bone.source = link;
                    bone.name = boneName;

                    // Check for nan and for zero scale
                    {
                        bool bFoundNan = false;
                        bool bFoundZeroScale = false;
                        for (int i = 0; i < 4; i++) {
                            if (i < 3) {
                                if (Maths::IsNaN(localLinkT[i]) || Maths::IsNaN(localLinkS[i])) {
                                    bFoundNan = true;
                                }
                                if (Maths::IsNearlyZero(localLinkS[i])) {
                                    bFoundZeroScale = true;
                                }
                            }
                            if (Maths::IsNaN(localLinkQ[i])) {
                                bFoundNan = true;
                            }
                        }

                        if (bFoundNan) {
                            LOG_WARN(fmt::format("Transform data for bone \"{}\" contains NaN.", boneName));
                        }

                        if (bFoundZeroScale) {
                            LOG_WARN(fmt::format("Transform data for bone \"{}\" contains dimension with zero scale.", boneName));
                        }
                    }

                    MeshImportData::JointPosition& jointMatrix = bone.bonePosition;
                    FbxSkeleton* skeleton = link->GetSkeleton();
                    if (skeleton) {
                        jointMatrix.length = FbxDataConverter::ConvertDist(skeleton->LimbLength.Get());
                        jointMatrix.xSize = FbxDataConverter::ConvertDist(skeleton->Size.Get());
                        jointMatrix.ySize = FbxDataConverter::ConvertDist(skeleton->Size.Get());
                        jointMatrix.zSize = FbxDataConverter::ConvertDist(skeleton->Size.Get());
                    } else {
                        jointMatrix.length = 1.;
                        jointMatrix.xSize = 100.;
                        jointMatrix.ySize = 100.;
                        jointMatrix.zSize = 100.;
                    }

                    // get the link parent and children
                    bone.parentIndex = parentIndex;
                    bone.numChildren = 0;
                    for (int childIndex = 0; childIndex < link->GetChildCount(); childIndex++) {
                        FbxNode* child = link->GetChild(childIndex);
                        bool isSkeleton = false;
                        {
                            FbxNodeAttribute* attr = child->GetNodeAttribute();
                            if (attr) {
                                FbxNodeAttribute::EType atrType = attr->GetAttributeType();
                                if (atrType == FbxNodeAttribute::eSkeleton || atrType == FbxNodeAttribute::eMesh || atrType == FbxNodeAttribute::eNull) {
                                    isSkeleton = true;
                                }
                            }
                        }
                        if (isSkeleton) {
                            bone.numChildren++;
                        }
                    }

                    jointMatrix.transform.translation = FbxDataConverter::ConvertPos(localLinkT);
                    jointMatrix.transform.rotation = FbxDataConverter::ConvertRotToQuat(localLinkQ);
                    jointMatrix.transform.scale = FbxDataConverter::ConvertScale(localLinkS);
                }
                MeshImportData::Bone& rootBone = bones[0];
                if (rootBone.source->GetParent()) {
                    Maths::Transform& RootTransform = rootBone.bonePosition.transform;
                    Maths::Transform GlobalSkeletalNode;
                    GlobalSkeletalNode.SetFromMatrix(FbxDataConverter::ConvertMatrix(rootBone.source->GetParent()->EvaluateGlobalTransform().Inverse()));
                    RootTransform.SetFromMatrix(GlobalSkeletalNode.ToMatrixWithScale() * RootTransform.ToMatrixWithScale());
                }
                std::shared_ptr<MeshImportData> importData = std::make_shared<MeshImportData>();
                importData->sortedLinks = sortedLinks;
                importData->bones = bones;
                Builder::SkinnedMeshSkeleton skeleton;
                int skeletalDepth;
                Builder::SkinnedMeshHelper::ProcessImportSkinnedMeshSkeleton(skeleton, skeletalDepth, importData);
                std::vector<std::shared_ptr<Assets::AnimationClipAsset>> animationClips;
                std::shared_ptr<Assets::SkeletonAsset> skeletonAsset = nullptr;
                {
                    int boneNum = skeleton.GetBoneNum();
                    if (boneNum) {
                        auto& root = skeleton.GetBoneInfo()[0];
                        if (skeletonAssets.find(root.source) == skeletonAssets.end()) {
                            skeletonAsset = skeletonAssets[root.source] = Builder::ConvertToSkeleton(skeleton, root, false);
                            if (options->bParseAnimations && sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>() > 0) {
                                auto animations = ImportAnimations(sortedLinks[0], importData->sortedLinks, skeletonAsset->names, skeletonAsset->paths, skeleton);
                                Builder::AnimationBuildSettings animationBuildOptions;
                                Builder::AnimationClipBuilder animationBuilder;
                                for (auto animation : animations) {
                                    auto animationAsset = std::make_shared<Assets::AnimationClipAsset>();
                                    animationBuilder.Build(animationAsset, animation, animationBuildOptions);
                                    animationClips.push_back(animationAsset);
                                }
                            }
                        } else {
                            skeletonAsset = skeletonAssets[root.source];
                        }
                        if (root.source->GetParent() && nodeMap.find(root.source->GetParent()) != nodeMap.end()) {
                            std::shared_ptr<Objects::Skeleton> skeletonFound = nullptr;
                            for (int i = 0; i < nodeMap[root.source->GetParent()]->components.size(); i++) {
                                Objects::EComponentType compType = nodeMap[root.source->GetParent()]->components[i]->GetTypeName();
                                if (compType == Objects::EComponentType::Skeleton) {
                                    skeletonFound = std::static_pointer_cast<Objects::Skeleton>(nodeMap[root.source->GetParent()]->components[i]);
                                    break;
                                }
                            }
                            if (skeletonFound == nullptr) {
                                std::shared_ptr<Objects::Skeleton> skeleton = std::make_shared<Objects::Skeleton>();
                                skeleton->skeleton = skeletonAsset;
                                nodeMap[root.source->GetParent()]->components.push_back(skeleton);
                                std::shared_ptr<Objects::Animation> animation = std::make_shared<Objects::Animation>();
                                animation->skeleton = skeleton;
                                for (auto& animationClip : animationClips) {
                                    std::string clipName = animationClip->name;
                                    int renameCount = 0;
                                    while (animation->animationClips.find(clipName) != animation->animationClips.end()) {
                                        clipName = animationClip->name + "_" + std::to_string(++renameCount);
                                    }
                                    animation->animationClips[clipName] = animationClip;
                                }
                                nodeMap[root.source->GetParent()]->components.push_back(animation);
                            }
                        }
                    }
                }
            }
        }
    }
}

}}  // namespace Fbx::Importer