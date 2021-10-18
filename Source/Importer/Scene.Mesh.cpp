#include <filesystem>
#include "Scene.h"

namespace Fbx { namespace Importer {
MeshImportData::MeshWedge::MeshWedge(uint32_t maxTexCoord) { uvs.resize(maxTexCoord); }

MeshImportData::Vertex::Vertex(uint32_t maxTexCoord) { uvs.resize(maxTexCoord); }

FbxNode* Scene::GetRootSkeleton(FbxNode* link) {
    FbxNode* rootBone = link;

    // mesh and dummy are used as bone if they are in the skeleton hierarchy
    while (rootBone && rootBone->GetParent()) {
        bool bIsBlenderArmatureBone = false;
        if (sceneInfo->bIsCreateByBlender) {
            // Hack to support armature dummy node from blender
            // Users do not want the null attribute node named armature which is the parent of the real
            // root bone in blender fbx file This is a hack since if a rigid mesh group root node is
            // named "armature" it will be skip
            const char* rootBoneParentName = rootBone->GetParent()->GetName();
            FbxNode* grandFather = rootBone->GetParent()->GetParent();
            bIsBlenderArmatureBone = (grandFather == nullptr || grandFather == sceneInfo->scene->GetRootNode()) && (Utils::InsensitiveCaseEquals(rootBoneParentName, "armature"));
        }

        FbxNodeAttribute* attr = rootBone->GetParent()->GetNodeAttribute();
        if (attr && (attr->GetAttributeType() == FbxNodeAttribute::eMesh || (attr->GetAttributeType() == FbxNodeAttribute::eNull && !bIsBlenderArmatureBone) || attr->GetAttributeType() == FbxNodeAttribute::eSkeleton) && rootBone->GetParent() != sceneInfo->scene->GetRootNode()) {
            // in some case, skeletal mesh can be ancestor of bones
            // this avoids this situation
            if (attr->GetAttributeType() == FbxNodeAttribute::eMesh) {
                FbxMesh* Mesh = (FbxMesh*)attr;
                if (Mesh->GetDeformerCount(FbxDeformer::eSkin) > 0) {
                    break;
                }
            }

            rootBone = rootBone->GetParent();
        } else {
            break;
        }
    }

    return rootBone;
}

FbxPose* Scene::RetrievePoseFromBindPose(FbxNode* node) {
    const int poseCount = sceneInfo->scene->GetPoseCount();
    for (int poseIndex = 0; poseIndex < poseCount; poseIndex++) {
        FbxPose* currentPose = sceneInfo->scene->GetPose(poseIndex);

        // current pose is bind pose,
        if (currentPose && currentPose->IsBindPose()) {
            // IsValidBindPose doesn't work reliably
            // It checks all the parent chain(regardless root given), and if the parent doesn't have
            // correct bind pose, it fails It causes more false positive issues than the real issue we
            // have to worry about If you'd like to try this, set CHECK_VALID_BIND_POSE to 1, and try
            // the error message when Autodesk fixes this bug, then we might be able to re-open this
            std::string poseName = currentPose->GetName();
            // all error report status
            FbxStatus status;

            // it does not make any difference of checking with different node
            // it is possible pose 0 -> node array 2, but isValidBindPose function returns true even
            // with node array 0
            std::string currentName = node->GetName();
            NodeList pMissingAncestors, pMissingDeformers, pMissingDeformersAncestors, pWrongMatrices;

            if (currentPose->IsValidBindPoseVerbose(node, pMissingAncestors, pMissingDeformers, pMissingDeformersAncestors, pWrongMatrices, 0.0001, &status)) {
                LOG_INFO(fmt::format("Bind pose {} found for {}.", poseName, ImporterHelper::UTF8ToNative(node->GetName())));
                return currentPose;
            } else {
                // first try to fix up
                // add missing ancestors
                for (int i = 0; i < pMissingAncestors.GetCount(); i++) {
                    FbxAMatrix mat = pMissingAncestors.GetAt(i)->EvaluateGlobalTransform(FBXSDK_TIME_ZERO);
                    currentPose->Add(pMissingAncestors.GetAt(i), mat);
                }

                pMissingAncestors.Clear();
                pMissingDeformers.Clear();
                pMissingDeformersAncestors.Clear();
                pWrongMatrices.Clear();

                // check it again
                if (currentPose->IsValidBindPose(node)) {
                    LOG_INFO(fmt::format("Bind pose {} found for {}.", poseName, ImporterHelper::UTF8ToNative(node->GetName())));
                    return currentPose;
                } else {
                    // first try to find parent who is null group and see if you can try test it
                    // again
                    FbxNode* parentNode = node->GetParent();
                    while (parentNode) {
                        FbxNodeAttribute* Attr = parentNode->GetNodeAttribute();
                        if (Attr && Attr->GetAttributeType() == FbxNodeAttribute::eNull) {
                            // found it
                            break;
                        }

                        // find next parent
                        parentNode = parentNode->GetParent();
                    }

                    if (parentNode && currentPose->IsValidBindPose(parentNode)) {
                        LOG_INFO(fmt::format("Bind pose {} found for {}.", poseName, ImporterHelper::UTF8ToNative(node->GetName())));
                        return currentPose;
                    } else {
                        LOG_INFO(fmt::format("Failed to find bind pose {} found for {}: {}.", poseName, ImporterHelper::UTF8ToNative(node->GetName()), status.GetErrorString()));
                    }
                }
            }
        }
    }

    return nullptr;
}

FbxPose* Scene::CreateOrRetrievePoseFromBindPose(FbxNode* node) {
    FbxPose* bindPose = RetrievePoseFromBindPose(node);
    // get bind pose
    if (bindPose == nullptr) {
        LOG_WARN("No bind pose was found for skin, trying to create one.");
        // if failed, delete bind pose, and retry.
        const int poseCount = sceneInfo->scene->GetPoseCount();
        for (int poseIndex = poseCount - 1; poseIndex >= 0; --poseIndex) {
            FbxPose* currentPose = sceneInfo->scene->GetPose(poseIndex);
            // current pose is bind pose,
            if (currentPose && currentPose->IsBindPose()) {
                sceneInfo->scene->RemovePose(poseIndex);
                currentPose->Destroy();
            }
        }
        Importer::GetInstance()->sdkManager->CreateMissingBindPoses(sceneInfo->scene);
        bindPose = RetrievePoseFromBindPose(node);
        if (bindPose == nullptr) {
            LOG_ERROR("Failed to create a new bind pose.");
        } else {
            LOG_INFO("Create a new bind pose successfully.");
        }
    }
    return bindPose;
}

void Scene::RecursiveBuildSkeleton(FbxNode* link, std::vector<FbxNode*>& outSortedLinks) {
    bool isSkeleton = false;
    {
        FbxNodeAttribute* attr = link->GetNodeAttribute();
        if (attr) {
            FbxNodeAttribute::EType atrType = attr->GetAttributeType();
            if (atrType == FbxNodeAttribute::eSkeleton || atrType == FbxNodeAttribute::eMesh || atrType == FbxNodeAttribute::eNull) {
                isSkeleton = true;
            }
        }
    }
    if (isSkeleton) {
        outSortedLinks.push_back(link);
        int childIndex;
        for (childIndex = 0; childIndex < link->GetChildCount(); childIndex++) {
            RecursiveBuildSkeleton(link->GetChild(childIndex), outSortedLinks);
        }
    }
}

void Scene::BuildSkeletonSystem(std::vector<FbxNode*>& sortedLinks, const std::vector<FbxCluster*>& clusters) {
    std::vector<FbxNode*> rootLinks;
    for (int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
        FbxNode* link = clusters[clusterIndex]->GetLink();
        if (link) {
            link = GetRootSkeleton(link);
            int linkIndex;
            for (linkIndex = 0; linkIndex < rootLinks.size(); linkIndex++) {
                if (link == rootLinks[linkIndex]) {
                    break;
                }
            }

            // this link is a new root, add it
            if (linkIndex == rootLinks.size()) {
                rootLinks.push_back(link);
            }
        }
    }

    for (int linkIndex = 0; linkIndex < rootLinks.size(); linkIndex++) {
        RecursiveBuildSkeleton(rootLinks[linkIndex], sortedLinks);
    }
}

bool Scene::BuildSkeletonBones(const std::vector<FbxNode*>& sortedLinks, const std::vector<FbxCluster*>& clusters, FbxPose* bindPose, std::vector<MeshImportData::Bone>& bones, int& rootIdx) {
    FbxArray<FbxAMatrix> linkMatrices;
    linkMatrices.Grow(static_cast<int>(sortedLinks.size()));
    linkMatrices[0].SetIdentity();

    bool bAnyLinksNotInBindPose = false;
    std::string linksWithoutBindPoses;

    for (int linkIndex = 0; linkIndex < sortedLinks.size(); linkIndex++) {
        // Add a bone for each FBX Link
        bones.push_back(MeshImportData::Bone());
        FbxNode* link = sortedLinks[linkIndex];
        int parentIndex = -1;  // base value for root if no parent found
        if (linkIndex == 0) {
            rootIdx = linkIndex;
        } else {
            const FbxNode* linkParent = link->GetParent();
            // get the link parent index.
            for (int parentLinkIndex = 0; parentLinkIndex < linkIndex; parentLinkIndex++)  // <LinkIndex because parent is guaranteed to be before child in
                                                                                           // sortedLink
            {
                FbxNode* otherlink = sortedLinks[parentLinkIndex];
                if (otherlink == linkParent) {
                    parentIndex = parentLinkIndex;
                    break;
                }
            }

            if (parentIndex == -1)  // We found another root inside the hierarchy, this is not supported
            {
                LOG_ERROR("Multi root found for skeleton, this is not supported.");
                return false;
            }
        }

        bool bIsLinkFound = false;

        // there are some links, they have no cluster, but in bindpose
        if (bindPose) {
            int poseLinkIndex = bindPose->Find(link);
            if (poseLinkIndex >= 0) {
                // bool IsLocalMatrix(int pIndex) const Get the type of the matrix.
                // Parameters pIndex Index of the queried item.Returns true if the matrix is defined in the Local coordinate space and
                // false otherwise.Remarks If the FbxPose object is configured to hold BindPose data,
                // this method will always return false.
                FbxMatrix noneAffineMatrix = bindPose->GetMatrix(poseLinkIndex);
                FbxAMatrix matrix = *(FbxAMatrix*)(double*)&noneAffineMatrix;
                linkMatrices[linkIndex] = matrix;
                bIsLinkFound = true;
            }
        }

        if (!bIsLinkFound) {
            // since now we set use time 0 as ref pose this won't unlikely happen
            // but leaving it just in case it still has case where it's missing partial bind pose

            bAnyLinksNotInBindPose = true;
            linksWithoutBindPoses += ImporterHelper::UTF8ToNative(link->GetName());
            linksWithoutBindPoses += ",";

            for (int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
                FbxCluster* cluster = clusters[clusterIndex];
                if (link == cluster->GetLink()) {
                    cluster->GetTransformLinkMatrix(linkMatrices[linkIndex]);
                    bIsLinkFound = true;
                    break;
                }
            }
        }

        if (!bIsLinkFound) {
            linkMatrices[linkIndex] = link->EvaluateGlobalTransform();
        } else {
            if (sceneInfo->scaleFactor != 1.0) {
                FbxVector4 trans = linkMatrices[linkIndex].GetT();
                trans[0] *= sceneInfo->scaleFactor;
                trans[1] *= sceneInfo->scaleFactor;
                trans[2] *= sceneInfo->scaleFactor;
                linkMatrices[linkIndex].SetT(trans);
            }
        }

        // Add the join orientation
        linkMatrices[linkIndex] = linkMatrices[linkIndex];

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
                LOG_WARN(fmt::format("Transform data for bone {} contains NaN.", boneName));
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

    if (bAnyLinksNotInBindPose) {
        LOG_WARN(fmt::format("Info for bones {} is missing in bind pose.", linksWithoutBindPoses));
    }

    return true;
}

bool Scene::ImportBones(std::vector<FbxNode*>& sortedLinks, FbxNode* node, FbxMesh* fbxMesh, std::vector<MeshImportData::Bone>& bones) {
    FbxNode* link = NULL;
    std::vector<FbxCluster*> clusterArray;

    //
    // create the bones / skinning
    //
    const int skinDeformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
    for (int deformerIndex = 0; deformerIndex < skinDeformerCount; deformerIndex++) {
        FbxSkin* Skin = (FbxSkin*)fbxMesh->GetDeformer(deformerIndex, FbxDeformer::eSkin);
        for (int clusterIndex = 0; clusterIndex < Skin->GetClusterCount(); clusterIndex++) {
            clusterArray.push_back(Skin->GetCluster(clusterIndex));
        }
    }

    if (clusterArray.size() == 0) {
        LOG_WARN(fmt::format("No associated clusters found for {}.", ImporterHelper::UTF8ToNative(node->GetName())));
        return false;
    }

    FbxPose* bindPose = CreateOrRetrievePoseFromBindPose(node);

    // recurse through skeleton and build ordered table
    BuildSkeletonSystem(sortedLinks, clusterArray);

    // error check
    // if no bond is found
    if (sortedLinks.size() == 0) {
        LOG_ERROR(fmt::format("No skeleton info found for {}.", ImporterHelper::UTF8ToNative(node->GetName())));
        return false;
    }

    int linkIndex;

    // Check for duplicate bone names and issue a warning if found
    for (linkIndex = 0; linkIndex < sortedLinks.size(); linkIndex++) {
        link = sortedLinks[linkIndex];
        for (int altLinkIndex = linkIndex + 1; altLinkIndex < sortedLinks.size(); altLinkIndex++) {
            FbxNode* altLink = sortedLinks[altLinkIndex];
            if (strcmp(link->GetName(), altLink->GetName()) == 0) {
                LOG_ERROR(fmt::format("Failed to import bones {}.", ImporterHelper::UTF8ToNative(link->GetName())));
                return false;
            }
        }
    }

    int rootIdx = -1;
    if (!BuildSkeletonBones(sortedLinks, clusterArray, bindPose, bones, rootIdx) || rootIdx == -1) {
        return false;
    }

    // UE4 中会把骨骼根节点的坐标映射成相对于mesh的位置，这是不符合引擎的设计的。
    // In case we do a scene import we need a relative to skinned mesh transform instead of a global
    // FbxAMatrix GlobalSkeletalNodeFbx = sceneInfo->scene->GetAnimationEvaluator()->GetNodeGlobalTransform(node, 0);
    // FTransform GlobalSkeletalNode;
    // GlobalSkeletalNode.SetFromMatrix(FbxDataConverter::ConvertMatrix(GlobalSkeletalNodeFbx.Inverse()));
    // MeshImportData::Bone& RootBone = bones[rootIdx];
    // FTransform& RootTransform = RootBone.bonePosition.transform;
    // RootTransform.SetFromMatrix(GlobalSkeletalNode.ToMatrixWithScale() * RootTransform.ToMatrixWithScale());
    MeshImportData::Bone& rootBone = bones[rootIdx];
    if (rootBone.source->GetParent()) {
        Maths::Transform& RootTransform = rootBone.bonePosition.transform;
        Maths::Transform GlobalSkeletalNode;
        GlobalSkeletalNode.SetFromMatrix(FbxDataConverter::ConvertMatrix(rootBone.source->GetParent()->EvaluateGlobalTransform().Inverse()));
        RootTransform.SetFromMatrix(GlobalSkeletalNode.ToMatrixWithScale() * RootTransform.ToMatrixWithScale());
    }
    return true;
}
int Scene::DoUnSmoothVerts(std::shared_ptr<MeshImportData> importData, bool bDuplicateUnSmoothWedges) {
    struct FaceRecord {
        int faceIndex;
        int hookIndex;
        int wedgeIndex;
        uint32_t smoothFlags;
        uint32_t fanFlags;
    };

    struct VertsFans {
        std::vector<FaceRecord> faceRecord;
        int fanGroupCount;
    };

    struct Influences {
        std::vector<int> rawInfIndices;
    };

    struct WedgeList {
        std::vector<int> wedgeList;
    };

    struct FaceSet {
        std::vector<int> faceRecordIndedices;
    };

    auto FacesAreSmoothlyConnected = [](std::shared_ptr<MeshImportData> importData, int face1, int face2) -> bool {
        if (face1 == face2) {
            return true;
        }

        // Smoothing groups match at least one bit in binary AND ?
        if ((importData->faces[face1].smoothingGroups & importData->faces[face2].smoothingGroups) == 0) {
            return false;
        }

        int vertMatches = 0;
        for (int i = 0; i < 3; i++) {
            int Point1 = importData->wedges[importData->faces[face1].wedgeIndex[i]].vertexIndex;

            for (int j = 0; j < 3; j++) {
                int Point2 = importData->wedges[importData->faces[face2].wedgeIndex[j]].vertexIndex;
                if (Point2 == Point1) {
                    vertMatches++;
                }
            }
        }

        return (vertMatches >= 2);
    };

    //
    // Connectivity: triangles with non-matching smoothing groups will be physically split.
    //
    // -> Splitting involves: the UV+material-contaning vertex AND the 3d point.
    //
    // -> Tally smoothing groups for each and every (textured) vertex.
    //
    // -> Collapse:
    // -> start from a vertex and all its adjacent triangles - go over
    // each triangle - if any connecting one (sharing more than one vertex) gives a smoothing match,
    // accumulate it. Then IF more than one resulting section,
    // ensure each boundary 'vert' is split _if not already_ to give each smoothing group
    // independence from all others.
    //

    int duplicatedVertCount = 0;
    int remappedHooks = 0;

    int totalSmoothMatches = 0;
    int totalConnexChex = 0;

    // Link _all_ faces to vertices.
    std::vector<VertsFans> fans;
    std::vector<Influences> pointInfluences;  // vertex => [ ImportData->Influences index ]
    std::vector<WedgeList> pointWedges;       // vertex => [ ImportData->Wedges index ]

    fans.resize(importData->points.size());             // Fans.AddExactZeroed(			Thing->SkinData.Points.Num() );
    pointInfluences.resize(importData->points.size());  // PointInfluences.AddExactZeroed( Thing->SkinData.Points.Num() );
    pointWedges.resize(importData->points.size());      // PointWedges.AddExactZeroed(	 Thing->SkinData.Points.Num() );

    // Existing points map 1:1
    importData->pointToRawMap.resize(importData->points.size(), 0);
    for (int i = 0; i < importData->points.size(); i++) {
        importData->pointToRawMap[i] = i;
    }

    for (int i = 0; i < importData->influences.size(); i++) {
        if (pointInfluences.size() <= importData->influences[i].vertexIndex) {
            auto ZeroToAdd = importData->influences[i].vertexIndex - pointInfluences.size() + 1;
            for (int ii = 0; ii < ZeroToAdd; ii++) {
                pointInfluences.push_back(Influences());
            }
        }
        pointInfluences[importData->influences[i].vertexIndex].rawInfIndices.push_back(i);
    }

    for (int i = 0; i < importData->wedges.size(); i++) {
        if (static_cast<uint32_t>(pointWedges.size()) <= importData->wedges[i].vertexIndex) {
            auto ZeroToAdd = importData->wedges[i].vertexIndex - pointWedges.size() + 1;
            for (int ii = 0; ii < ZeroToAdd; ii++) {
                pointWedges.push_back(WedgeList());
            }
        }

        pointWedges[importData->wedges[i].vertexIndex].wedgeList.push_back(i);
    }

    for (int f = 0; f < importData->faces.size(); f++) {
        // For each face, add a pointer to that face into the Fans[vertex].
        for (int i = 0; i < 3; i++) {
            int wedgeIndex = importData->faces[f].wedgeIndex[i];
            int pointIndex = importData->wedges[wedgeIndex].vertexIndex;
            FaceRecord newFR;

            newFR.faceIndex = f;
            newFR.hookIndex = i;
            newFR.wedgeIndex = wedgeIndex;  // This face touches the point courtesy of Wedges[Wedgeindex].
            newFR.smoothFlags = importData->faces[f].smoothingGroups;
            newFR.fanFlags = 0;
            fans[pointIndex].faceRecord.push_back(newFR);
            fans[pointIndex].fanGroupCount = 0;
        }
    }

    // Investigate connectivity and assign common group numbers (1..+) to the fans' individual FanFlags.
    for (int p = 0; p < fans.size(); p++)  // The fan of faces for each 3d point 'p'.
    {
        // All faces connecting.
        if (fans[p].faceRecord.size() > 0) {
            int facesProcessed = 0;
            std::vector<FaceSet> faceSets;  // Sets with indices INTO FANS, not into face array.

            // Digest all faces connected to this vertex (p) into one or more smooth sets. only need to check
            // all faces MINUS one..
            while (facesProcessed < fans[p].faceRecord.size()) {
                // One loop per group. For the current ThisFaceIndex, tally all truly connected ones
                // and put them in a new TArray. Once no more can be connected, stop.

                int newSetIndex = static_cast<int>(faceSets.size());  // 0 to start
                faceSets.push_back(FaceSet());                        // first one will be just ThisFaceIndex.

                // Find the first non-processed face. There will be at least one.
                int thisFaceFanIndex = 0;
                {
                    int searchIndex = 0;
                    while (fans[p].faceRecord[searchIndex].fanFlags == -1)  // -1 indicates already  processed.
                    {
                        searchIndex++;
                    }
                    thisFaceFanIndex = searchIndex;  // Fans[p].FaceRecord[SearchIndex].FaceIndex;
                }

                // Initial face.
                faceSets[newSetIndex].faceRecordIndedices.push_back(thisFaceFanIndex);  // Add the unprocessed Face index to the "local smoothing group" [NewSetIndex].
                fans[p].faceRecord[thisFaceFanIndex].fanFlags = -1;                     // Mark as processed.
                facesProcessed++;

                // Find all faces connected to this face, and if there's any
                // smoothing group matches, put it in current face set and mark it as processed;
                // until no more match.
                int newMatches = 0;
                do {
                    newMatches = 0;
                    // Go over all current faces in this faceset and set if the FaceRecord (local smoothing groups) has
                    // any matches. there will be at least one face already in this faceset - the first face in the fan.
                    for (int currentFaceSetIter = 0; currentFaceSetIter < faceSets[newSetIndex].faceRecordIndedices.size(); currentFaceSetIter++) {
                        int hookFaceIdx = fans[p].faceRecord[faceSets[newSetIndex].faceRecordIndedices[currentFaceSetIter]].faceIndex;

                        // Go over the fan looking for matches.
                        for (int currentFaceRecordIter = 0; currentFaceRecordIter < fans[p].faceRecord.size(); currentFaceRecordIter++) {
                            // Skip if same face, skip if face already processed.
                            if ((hookFaceIdx != fans[p].faceRecord[currentFaceRecordIter].faceIndex) && (fans[p].faceRecord[currentFaceRecordIter].fanFlags != -1)) {
                                totalConnexChex++;
                                // Process if connected with more than one vertex, AND smooth..
                                if (FacesAreSmoothlyConnected(importData, hookFaceIdx, fans[p].faceRecord[currentFaceRecordIter].faceIndex)) {
                                    totalSmoothMatches++;
                                    fans[p].faceRecord[currentFaceRecordIter].fanFlags = -1;  // Mark as processed.
                                    facesProcessed++;
                                    // Add
                                    faceSets[newSetIndex].faceRecordIndedices.push_back(currentFaceRecordIter);  // Store FAN index of this face index into smoothing
                                                                                                                 // group's faces.
                                    // Tally
                                    newMatches++;
                                }
                            }  // not the same...
                        }      // all faces in fan
                    }          // all faces in FaceSet
                } while (newMatches);

            }  // Repeat until all faces processed.

            // For the new non-initialized  face sets,
            // Create a new point, influences, and uv-vertex(-ices) for all individual FanFlag groups with an index of
            // 2+ and also remap the face's vertex into those new ones.
            if (faceSets.size() > 1) {
                for (int f = 1; f < faceSets.size(); f++) {
                    ASSERT(importData->points.size() == importData->pointToRawMap.size());

                    // We duplicate the current vertex. (3d point)
                    int newPointIndex = static_cast<int>(importData->points.size());
                    importData->points.push_back(glm::vec3(0));
                    importData->points[newPointIndex] = importData->points[p];

                    importData->pointToRawMap.push_back(0);
                    importData->pointToRawMap[newPointIndex] = p;

                    duplicatedVertCount++;

                    // Duplicate all related weights.
                    for (int t = 0; t < pointInfluences[p].rawInfIndices.size(); t++) {
                        // Add new weight
                        int newWeightIndex = static_cast<int>(importData->influences.size());
                        importData->influences.push_back(MeshImportData::RawBoneInfluence());
                        importData->influences[newWeightIndex] = importData->influences[pointInfluences[p].rawInfIndices[t]];
                        importData->influences[newWeightIndex].vertexIndex = newPointIndex;
                    }

                    // Duplicate any and all Wedges associated with it; and all Faces' wedges involved.
                    for (int w = 0; w < pointWedges[p].wedgeList.size(); w++) {
                        int oldWedgeIndex = pointWedges[p].wedgeList[w];
                        int newWedgeIndex = static_cast<int>(importData->wedges.size());

                        if (bDuplicateUnSmoothWedges) {
                            importData->wedges.push_back(MeshImportData::Vertex());
                            importData->wedges[newWedgeIndex] = importData->wedges[oldWedgeIndex];
                            importData->wedges[newWedgeIndex].vertexIndex = newPointIndex;

                            //  Update relevant face's Wedges. Inelegant: just check all associated faces for every new
                            //  wedge.
                            for (int s = 0; s < faceSets[f].faceRecordIndedices.size(); s++) {
                                int FanIndex = faceSets[f].faceRecordIndedices[s];
                                if (fans[p].faceRecord[FanIndex].wedgeIndex == oldWedgeIndex) {
                                    // Update just the right one for this face (HoekIndex!)
                                    importData->faces[fans[p].faceRecord[FanIndex].faceIndex].wedgeIndex[fans[p].faceRecord[FanIndex].hookIndex] = newWedgeIndex;
                                    remappedHooks++;
                                }
                            }
                        } else {
                            importData->wedges[oldWedgeIndex].vertexIndex = newPointIndex;
                        }
                    }
                }
            }  //  if FaceSets.Num(). -> duplicate stuff
        }      //	while( FacesProcessed < Fans[p].FaceRecord.Num() )
    }          // Fans for each 3d point

    ASSERT(importData->points.size() == importData->pointToRawMap.size());

    return duplicatedVertCount;
}

std::shared_ptr<MeshImportData> Scene::ImportMesh(FbxNode* fbxNode, FbxMesh* fbxMesh, bool bForSkinnedMesh) {
    std::string fbxMeshName = ImporterHelper::UTF8ToNative(fbxMesh->GetName()).size() ? ImporterHelper::UTF8ToNative(fbxMesh->GetName()) : ImporterHelper::UTF8ToNative(fbxNode->GetName());
    fbxMeshName = Utils::SanitizeObjectName(fbxMeshName);
    if (fbxMeshName == "") {
        fbxMeshName = "UnnamedMesh";
    }

    std::shared_ptr<MeshImportData> importData = std::make_shared<MeshImportData>();
    importData->name = fbxMeshName;

    std::vector<FbxNode*>& sortedLinks = importData->sortedLinks;
    if (bForSkinnedMesh) {
        std::vector<MeshImportData::Bone>& bones = importData->bones;
        if (!ImportBones(sortedLinks, fbxNode, fbxMesh, bones)) {
            return nullptr;
        }
        // Do some checks before proceeding, check to make sure the number of bones does not exceed the maximum
        // supported
        if (sortedLinks.size() > 65536) {
            LOG_ERROR(fmt::format("Mesh {} contains {:d} bones, exceeding limit 65536.", fbxMeshName, sortedLinks.size()));
            return nullptr;
        }
    }

    // Create a list of all unique fbx materials.  This needs to be done as a separate pass before reading geometry
    // so that we know about all possible materials before assigning material indices to each triangle
    std::vector<FbxSurfaceMaterial*> fbxMaterials;

    for (int materialIndex = 0; materialIndex < fbxNode->GetMaterialCount(); materialIndex++) {
        FbxSurfaceMaterial* fbxMaterial = fbxNode->GetMaterial(materialIndex);
        if (std::find(fbxMaterials.begin(), fbxMaterials.end(), fbxMaterial) == fbxMaterials.end()) {
            fbxMaterials.push_back(fbxMaterial);
            MeshImportData::RawMaterial newMaterial;

            newMaterial.materialImportName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(fbxMaterial->GetName()));
            // Add an entry for each unique material
            importData->materials.push_back(newMaterial);
        }
    }

    // remove the bad polygons before getting any data from mesh
    fbxMesh->RemoveBadPolygons();

    // Get the base layer of the mesh
    FbxLayer* baseLayer = fbxMesh->GetLayer(0);
    if (baseLayer == NULL) {
        LOG_ERROR(fmt::format("No geometry info found for mesh {}.", fbxMeshName));
        return nullptr;
    }

    //
    //	store the UVs in arrays for fast access in the later looping of triangles
    //
    // mapping from UVSets to Fbx LayerElementUV
    // Fbx UVSets may be duplicated, remove the duplicated UVSets in the mapping
    int layerCount = fbxMesh->GetLayerCount();
    std::vector<std::string> UVSets;
    if (layerCount > 0) {
        int UVLayerIndex;
        for (UVLayerIndex = 0; UVLayerIndex < layerCount; UVLayerIndex++) {
            FbxLayer* lLayer = fbxMesh->GetLayer(UVLayerIndex);
            int UVSetCount = lLayer->GetUVSetCount();
            if (UVSetCount) {
                FbxArray<FbxLayerElementUV const*> EleUVs = lLayer->GetUVSets();
                for (int UVIndex = 0; UVIndex < UVSetCount; UVIndex++) {
                    FbxLayerElementUV const* ElementUV = EleUVs[UVIndex];
                    if (ElementUV) {
                        const char* UVSetName = ElementUV->GetName();
                        std::string LocalUVSetName = ImporterHelper::UTF8ToNative(UVSetName);
                        if (LocalUVSetName.empty()) {
                            LocalUVSetName = "UVmap_" + std::to_string(UVLayerIndex);
                        }
                        if (std::find(UVSets.begin(), UVSets.end(), LocalUVSetName) == UVSets.end()) {
                            UVSets.push_back(LocalUVSetName);
                        }
                    }
                }
            }
        }
    }

    std::vector<std::shared_ptr<Assets::MaterialAsset>> materials;

    FindOrImportMaterialsFromNode(fbxNode, materials, UVSets, bForSkinnedMesh);

    // Maps local mesh material index to global material index
    std::vector<int> materialMapping;
    const int materialCount = fbxNode->GetMaterialCount();
    materialMapping.resize(materialCount);
    for (int materialIndex = 0; materialIndex < materialCount; materialIndex++) {
        FbxSurfaceMaterial* fbxMaterial = fbxNode->GetMaterial(materialIndex);

        int existingMatIndex = -1;
        auto materialFound = std::find(fbxMaterials.begin(), fbxMaterials.end(), fbxMaterial);
        existingMatIndex = materialFound == fbxMaterials.end() ? -1 : static_cast<int>(materialFound - fbxMaterials.begin());
        if (existingMatIndex != -1) {
            // Reuse existing material
            materialMapping[materialIndex] = existingMatIndex;

            if (materialIndex >= 0 && materials.size() > materialIndex) {
                importData->materials[existingMatIndex].material = materials[materialIndex];
            }
        } else {
            materialMapping[materialIndex] = 0;
        }
    }

    if (layerCount > 0 && Importer::GetInstance()->options->bPreserveSmoothingGroups) {
        // Check and see if the smooothing data is valid.  If not generate it from the normals
        baseLayer = fbxMesh->GetLayer(0);
        if (baseLayer) {
            const FbxLayerElementSmoothing* smoothingLayer = baseLayer->GetSmoothing();

            if (smoothingLayer) {
                bool bValidSmoothingData = false;
                FbxLayerElementArrayTemplate<int>& directArray = smoothingLayer->GetDirectArray();
                for (int smoothingIndex = 0; smoothingIndex < directArray.GetCount(); smoothingIndex++) {
                    if (directArray[smoothingIndex] != 0) {
                        bValidSmoothingData = true;
                        break;
                    }
                }

                if (!bValidSmoothingData && fbxMesh->GetPolygonVertexCount() > 0) {
                    Importer::GetInstance()->geometryConverter->ComputeEdgeSmoothingFromNormals(fbxMesh);
                }
            }
        }
    }
    // Must do this before triangulating the mesh due to an FBX bug in TriangulateMeshAdvance
    int layerSmoothingCount = fbxMesh->GetLayerCount(FbxLayerElement::eSmoothing);
    for (int i = 0; i < layerSmoothingCount; i++) {
        FbxLayerElementSmoothing const* smoothingInfo = fbxMesh->GetLayer(i)->GetSmoothing();
        if (smoothingInfo && smoothingInfo->GetMappingMode() != FbxLayerElement::eByPolygon) {
            Importer::GetInstance()->geometryConverter->ComputePolygonSmoothingFromEdgeSmoothing(fbxMesh, i);
        }
    }

    if (!fbxMesh->IsTriangleMesh()) {
        LOG_INFO(fmt::format("Triangulating mesh {} ...", ImporterHelper::UTF8ToNative(fbxNode->GetName())));
        const bool bReplace = true;
        FbxNodeAttribute* convertedNode = Importer::GetInstance()->geometryConverter->Triangulate(fbxMesh, bReplace);
        if (convertedNode != NULL && convertedNode->GetAttributeType() == FbxNodeAttribute::eMesh) {
            fbxMesh = convertedNode->GetNode()->GetMesh();
        } else {
            LOG_ERROR(fmt::format("Failed to triangulate mesh {}.", ImporterHelper::UTF8ToNative(fbxNode->GetName())));
            return nullptr;
        }
    }

    // renew the base layer
    baseLayer = fbxMesh->GetLayer(0);
    FbxSkin* skin = bForSkinnedMesh ? (FbxSkin*)static_cast<FbxGeometry*>(fbxMesh)->GetDeformer(0, FbxDeformer::eSkin) : NULL;
    FbxShape* fbxShape = nullptr;
    //
    //	store the UVs in arrays for fast access in the later looping of triangles
    //
    uint32_t uniqueUVCount = static_cast<uint32_t>(UVSets.size());
    FbxLayerElementUV** layerElementUV = NULL;
    FbxLayerElement::EReferenceMode* UVReferenceMode = NULL;
    FbxLayerElement::EMappingMode* UVMappingMode = NULL;
    if (uniqueUVCount > 0) {
        layerElementUV = new FbxLayerElementUV*[uniqueUVCount];
        UVReferenceMode = new FbxLayerElement::EReferenceMode[uniqueUVCount];
        UVMappingMode = new FbxLayerElement::EMappingMode[uniqueUVCount];
    } else {
        LOG_ERROR(fmt::format("Missing uv info for {}, trying to create one.", ImporterHelper::UTF8ToNative(fbxNode->GetName())));
    }
    layerCount = fbxMesh->GetLayerCount();
    for (uint32_t UVIndex = 0; UVIndex < uniqueUVCount; UVIndex++) {
        layerElementUV[UVIndex] = NULL;
        for (int UVLayerIndex = 0; UVLayerIndex < layerCount; UVLayerIndex++) {
            FbxLayer* lLayer = fbxMesh->GetLayer(UVLayerIndex);
            int UVSetCount = lLayer->GetUVSetCount();
            if (UVSetCount) {
                FbxArray<FbxLayerElementUV const*> EleUVs = lLayer->GetUVSets();
                for (int FbxUVIndex = 0; FbxUVIndex < UVSetCount; FbxUVIndex++) {
                    FbxLayerElementUV const* ElementUV = EleUVs[FbxUVIndex];
                    if (ElementUV) {
                        const char* UVSetName = ElementUV->GetName();
                        std::string LocalUVSetName = ImporterHelper::UTF8ToNative(UVSetName);
                        if (LocalUVSetName.empty()) {
                            LocalUVSetName = "UVmap_" + std::to_string(UVLayerIndex);
                        }
                        if (LocalUVSetName == UVSets[UVIndex]) {
                            layerElementUV[UVIndex] = const_cast<FbxLayerElementUV*>(ElementUV);
                            UVReferenceMode[UVIndex] = layerElementUV[UVIndex]->GetReferenceMode();
                            UVMappingMode[UVIndex] = layerElementUV[UVIndex]->GetMappingMode();
                            break;
                        }
                    }
                }
            }
        }
    }

    //
    // get the smoothing group layer
    //
    bool bSmoothingAvailable = false;

    FbxLayerElementSmoothing const* smoothingInfo = baseLayer->GetSmoothing();
    FbxLayerElement::EReferenceMode smoothingReferenceMode(FbxLayerElement::eDirect);
    FbxLayerElement::EMappingMode smoothingMappingMode(FbxLayerElement::eByEdge);
    if (smoothingInfo) {
        if (smoothingInfo->GetMappingMode() == FbxLayerElement::eByEdge) {
            if (!Importer::GetInstance()->geometryConverter->ComputePolygonSmoothingFromEdgeSmoothing(fbxMesh)) {
                LOG_WARN(fmt::format("Failed to transfer smooth group info for {}.", fbxMeshName));
                bSmoothingAvailable = false;
            } else {
                // After using the geometry converter we always have to get the Layer and the smoothing info
                baseLayer = fbxMesh->GetLayer(0);
                smoothingInfo = baseLayer->GetSmoothing();
            }
        }

        if (smoothingInfo->GetMappingMode() == FbxLayerElement::eByPolygon) {
            bSmoothingAvailable = true;
        }

        smoothingReferenceMode = smoothingInfo->GetReferenceMode();
        smoothingMappingMode = smoothingInfo->GetMappingMode();
    }

    //
    //	get the "material index" layer
    //
    FbxLayerElementMaterial* layerElementMaterial = baseLayer->GetMaterials();
    FbxLayerElement::EMappingMode materialMappingMode = layerElementMaterial ? layerElementMaterial->GetMappingMode() : FbxLayerElement::eByPolygon;

    uniqueUVCount = Maths::Min<uint32_t>(uniqueUVCount, Configuration::MaxTexCoord);

    // One UV set is required but only import up to MAX_TEXCOORDS number of uv layers
    importData->numTexCoords = Maths::Max<uint32_t>(importData->numTexCoords, uniqueUVCount);

    //
    // get the first vertex color layer
    //

    FbxLayerElementVertexColor* layerElementVertexColor = baseLayer->GetVertexColors();
    FbxLayerElement::EReferenceMode vertexColorReferenceMode(FbxLayerElement::eDirect);
    FbxLayerElement::EMappingMode vertexColorMappingMode(FbxLayerElement::eByControlPoint);
    if (layerElementVertexColor) {
        vertexColorReferenceMode = layerElementVertexColor->GetReferenceMode();
        vertexColorMappingMode = layerElementVertexColor->GetMappingMode();
        importData->bHasVertexColors = true;
    }

    //
    // get the first normal layer
    //
    FbxLayerElementNormal* layerElementNormal = baseLayer->GetNormals();
    FbxLayerElementTangent* LayerElementTangent = baseLayer->GetTangents();
    FbxLayerElementBinormal* layerElementBinormal = baseLayer->GetBinormals();

    // whether there is normal, tangent and binormal data in this mesh
    bool bHasNormalInformation = layerElementNormal != NULL;
    bool bHasTangentInformation = LayerElementTangent != NULL && layerElementBinormal != NULL;

    importData->bHasNormals = bHasNormalInformation;
    importData->bHasTangents = bHasTangentInformation;

    FbxLayerElement::EReferenceMode normalReferenceMode(FbxLayerElement::eDirect);
    FbxLayerElement::EMappingMode normalMappingMode(FbxLayerElement::eByControlPoint);
    if (layerElementNormal) {
        normalReferenceMode = layerElementNormal->GetReferenceMode();
        normalMappingMode = layerElementNormal->GetMappingMode();
    }

    FbxLayerElement::EReferenceMode tangentReferenceMode(FbxLayerElement::eDirect);
    FbxLayerElement::EMappingMode tangentMappingMode(FbxLayerElement::eByControlPoint);
    if (LayerElementTangent) {
        tangentReferenceMode = LayerElementTangent->GetReferenceMode();
        tangentMappingMode = LayerElementTangent->GetMappingMode();
    }

    //
    // create the points / wedges / faces
    //
    int controlPointsCount = fbxMesh->GetControlPointsCount();
    int existPointNum = static_cast<int>(importData->points.size());
    {
        const int controlPointsCount = fbxMesh->GetControlPointsCount();
        const int existPointNum = static_cast<int>(importData->points.size());
        for (int ii = 0; ii < controlPointsCount; ii++) {
            importData->points.push_back(glm::vec3(0));
        }

        int controlPointsIndex;
        bool bInvalidPositionFound = false;
        for (controlPointsIndex = 0; controlPointsIndex < controlPointsCount; controlPointsIndex++) {
            FbxVector4 position;
            if (fbxShape) {
                position = fbxShape->GetControlPoints()[controlPointsIndex];
            } else {
                position = fbxMesh->GetControlPoints()[controlPointsIndex];
            }

            glm::vec3 convertedPosition = FbxDataConverter::ConvertPos(position);

            // ensure user when this happens if attached to debugger
            auto NaNTest = glm::isnan(convertedPosition);
            if (NaNTest.x || NaNTest.y || NaNTest.z) {
                if (!bInvalidPositionFound) {
                    bInvalidPositionFound = true;
                }

                convertedPosition = glm::vec3(0);
            }

            importData->points[controlPointsIndex + existPointNum] = convertedPosition;
        }

        if (bInvalidPositionFound) {
            LOG_WARN(fmt::format("Position info for mesh {} contains NaN.", fbxMeshName));
        }
    }

    // Construct the matrices for the conversion from right handed to left handed system
    FbxAMatrix totalMatrix;
    FbxAMatrix totalMatrixForNormal;
    // totalMatrix = ComputeSkeletalMeshTotalMatrix(node, RootNode);
    totalMatrix.SetIdentity();
    totalMatrixForNormal = totalMatrix.Inverse();
    totalMatrixForNormal = totalMatrixForNormal.Transpose();

    bool bIsOddNegativeScale = ImporterHelper::IsOddNegativeScale(totalMatrix);

    int triangleCount = fbxMesh->GetPolygonCount();
    int existFaceNum = static_cast<int>(importData->faces.size());
    for (int ii = 0; ii < triangleCount; ii++) {
        importData->faces.push_back(MeshImportData::Triangle());
    }
    int existWedgesNum = static_cast<int>(importData->wedges.size());
    MeshImportData::Vertex tmpWedges[3];

    for (int triangleIndex = existFaceNum, localIndex = 0; triangleIndex < existFaceNum + triangleCount; triangleIndex++, localIndex++) {
        MeshImportData::Triangle& triangle = importData->faces[triangleIndex];

        //
        // smoothing mask
        //
        // set the face smoothing by default. It could be any number, but not zero
        triangle.smoothingGroups = 255;
        if (bSmoothingAvailable) {
            if (smoothingInfo) {
                if (smoothingMappingMode == FbxLayerElement::eByPolygon) {
                    int lSmoothingIndex = (smoothingReferenceMode == FbxLayerElement::eDirect) ? localIndex : smoothingInfo->GetIndexArray().GetAt(localIndex);
                    triangle.smoothingGroups = smoothingInfo->GetDirectArray().GetAt(lSmoothingIndex);
                } else {
                    LOG_ERROR(fmt::format("Mesh {} contains unsupported smoothing group info.", fbxMeshName));
                }
            }
        }

        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
            // If there are odd number negative scale, invert the vertex order for triangles
            int newVertexIndex = bIsOddNegativeScale ? 2 - vertexIndex : vertexIndex;

            int controlPointIndex = fbxMesh->GetPolygonVertex(localIndex, vertexIndex);
            //
            // normals, tangents and binormals
            //
            if (bHasNormalInformation) {
                int tmpIndex = localIndex * 3 + vertexIndex;
                // normals may have different reference and mapping mode than tangents and binormals
                int normalMapIndex = (normalMappingMode == FbxLayerElement::eByControlPoint) ? controlPointIndex : tmpIndex;
                int normalValueIndex = (normalReferenceMode == FbxLayerElement::eDirect) ? normalMapIndex : layerElementNormal->GetIndexArray().GetAt(normalMapIndex);

                // tangents and binormals share the same reference, mapping mode and index array
                int tangentMapIndex = tmpIndex;

                FbxVector4 tempValue;

                if (bHasTangentInformation) {
                    tempValue = LayerElementTangent->GetDirectArray().GetAt(tangentMapIndex);
                    tempValue = totalMatrixForNormal.MultT(tempValue);
                    triangle.tangentX[newVertexIndex] = FbxDataConverter::ConvertDir(tempValue);
                    triangle.tangentX[newVertexIndex] = glm::normalize(triangle.tangentX[newVertexIndex]);

                    tempValue = layerElementBinormal->GetDirectArray().GetAt(tangentMapIndex);
                    tempValue = totalMatrixForNormal.MultT(tempValue);
                    triangle.tangentY[newVertexIndex] = FbxDataConverter::ConvertDir(tempValue);
                    triangle.tangentY[newVertexIndex] = glm::normalize(triangle.tangentY[newVertexIndex]);
                }

                tempValue = layerElementNormal->GetDirectArray().GetAt(normalValueIndex);
                tempValue = totalMatrixForNormal.MultT(tempValue);
                triangle.tangentZ[newVertexIndex] = FbxDataConverter::ConvertDir(tempValue);
                triangle.tangentZ[newVertexIndex] = glm::normalize(triangle.tangentZ[newVertexIndex]);

            } else {
                for (int normalIndex = 0; normalIndex < 3; normalIndex++) {
                    triangle.tangentX[normalIndex] = glm::vec3(0);
                    triangle.tangentY[normalIndex] = glm::vec3(0);
                    triangle.tangentZ[normalIndex] = glm::vec3(0);
                }
            }
        }

        //
        // material index
        //
        triangle.materialIndex = 0;  // default value
        if (materialCount > 0) {
            if (layerElementMaterial) {
                switch (materialMappingMode) {
                        // material index is stored in the IndexArray, not the DirectArray (which is irrelevant with
                        // 2009.1)
                    case FbxLayerElement::eAllSame: {
                        triangle.materialIndex = materialMapping[layerElementMaterial->GetIndexArray().GetAt(0)];
                    } break;
                    case FbxLayerElement::eByPolygon: {
                        int index = layerElementMaterial->GetIndexArray().GetAt(localIndex);
                        if (!(index >= 0 && materialMapping.size() > index)) {
                            LOG_WARN("Inconsistent material index detected.");
                        } else {
                            triangle.materialIndex = materialMapping[index];
                        }
                    } break;
                }
            }

            // When import morph, we don't check the material index
            // because we don't import material for morph, so the ImportData.Materials contains zero material
            if (!fbxShape && (triangle.materialIndex < 0 || triangle.materialIndex >= fbxMaterials.size())) {
                LOG_WARN("Inconsistent material index detected.");
                triangle.materialIndex = 0;
            }
        }
        importData->maxMaterialIndex = Maths::Max<uint32_t>(importData->maxMaterialIndex, triangle.materialIndex);

        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
            // If there are odd number negative scale, invert the vertex order for triangles
            int newVertexIndex = bIsOddNegativeScale ? 2 - vertexIndex : vertexIndex;

            tmpWedges[newVertexIndex].materialIndex = triangle.materialIndex;
            tmpWedges[newVertexIndex].vertexIndex = existPointNum + fbxMesh->GetPolygonVertex(localIndex, vertexIndex);
            // Initialize all colors to white.
            tmpWedges[newVertexIndex].color = {1, 1, 1, 1};
        }

        //
        // uvs
        //
        uint32_t UVLayerIndex;
        // Some FBX meshes can have no UV sets, so also check the UniqueUVCount
        for (UVLayerIndex = 0; UVLayerIndex < uniqueUVCount; UVLayerIndex++) {
            // ensure the layer has data
            if (layerElementUV[UVLayerIndex] != NULL) {
                // Get each UV from the layer
                for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
                    // If there are odd number negative scale, invert the vertex order for triangles
                    int newVertexIndex = bIsOddNegativeScale ? 2 - vertexIndex : vertexIndex;

                    int lControlPointIndex = fbxMesh->GetPolygonVertex(localIndex, vertexIndex);
                    int UVMapIndex = (UVMappingMode[UVLayerIndex] == FbxLayerElement::eByControlPoint) ? lControlPointIndex : localIndex * 3 + vertexIndex;
                    int UVIndex = (UVReferenceMode[UVLayerIndex] == FbxLayerElement::eDirect) ? UVMapIndex : layerElementUV[UVLayerIndex]->GetIndexArray().GetAt(UVMapIndex);
                    FbxVector2 UVVector = layerElementUV[UVLayerIndex]->GetDirectArray().GetAt(UVIndex);

                    tmpWedges[newVertexIndex].uvs[UVLayerIndex].x = static_cast<float>(UVVector[0]);
                    tmpWedges[newVertexIndex].uvs[UVLayerIndex].y = 1.f - static_cast<float>(UVVector[1]);
                }
            } else if (UVLayerIndex == 0) {
                // Set all UV's to zero.  If we are here the mesh had no UV sets so we only need to do this for the
                // first UV set which always exists.

                for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
                    tmpWedges[vertexIndex].uvs[UVLayerIndex].x = 0.0f;
                    tmpWedges[vertexIndex].uvs[UVLayerIndex].y = 0.0f;
                }
            }
        }

        // Read vertex colors if they exist.
        if (layerElementVertexColor) {
            switch (vertexColorMappingMode) {
                case FbxLayerElement::eByControlPoint: {
                    for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
                        int newVertexIndex = bIsOddNegativeScale ? 2 - vertexIndex : vertexIndex;

                        FbxColor vertexColor = (vertexColorReferenceMode == FbxLayerElement::eDirect) ? layerElementVertexColor->GetDirectArray().GetAt(fbxMesh->GetPolygonVertex(localIndex, vertexIndex))
                                                                                                      : layerElementVertexColor->GetDirectArray().GetAt(layerElementVertexColor->GetIndexArray().GetAt(fbxMesh->GetPolygonVertex(localIndex, vertexIndex)));

                        tmpWedges[newVertexIndex].color = glm::vec4(float(vertexColor.mRed), float(vertexColor.mGreen), float(vertexColor.mBlue), float(vertexColor.mAlpha));
                    }
                } break;
                case FbxLayerElement::eByPolygonVertex: {
                    for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
                        int newVertexIndex = bIsOddNegativeScale ? 2 - vertexIndex : vertexIndex;

                        FbxColor vertexColor =
                            (vertexColorReferenceMode == FbxLayerElement::eDirect) ? layerElementVertexColor->GetDirectArray().GetAt(localIndex * 3 + vertexIndex) : layerElementVertexColor->GetDirectArray().GetAt(layerElementVertexColor->GetIndexArray().GetAt(localIndex * 3 + vertexIndex));

                        tmpWedges[newVertexIndex].color = glm::vec4(float(vertexColor.mRed), float(vertexColor.mGreen), float(vertexColor.mBlue), float(vertexColor.mAlpha));
                    }
                } break;
            }
        }

        //
        // basic wedges matching : 3 unique per face. TODO Can we do better ?
        //
        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
            int w;

            w = static_cast<int>(importData->wedges.size());
            importData->wedges.push_back(MeshImportData::Vertex());
            importData->wedges[w].vertexIndex = tmpWedges[vertexIndex].vertexIndex;
            importData->wedges[w].materialIndex = tmpWedges[vertexIndex].materialIndex;
            importData->wedges[w].color = tmpWedges[vertexIndex].color;
            for (int CoordIndex = 0; CoordIndex < Configuration::MaxTexCoord; CoordIndex++) {
                importData->wedges[w].uvs[CoordIndex] = tmpWedges[vertexIndex].uvs[CoordIndex];
            }

            triangle.wedgeIndex[vertexIndex] = w;
        }
    }

    // now we can work on a per-cluster basis with good ordering
    if (bForSkinnedMesh) {
        if (skin)  // skinned mesh
        {
            // create influences for each cluster
            for (int clusterIndex = 0; clusterIndex < skin->GetClusterCount(); clusterIndex++) {
                FbxCluster* cluster = skin->GetCluster(clusterIndex);
                // When Maya plug-in exports rigid binding, it will generate "CompensationCluster" for each ancestor
                // links. FBX writes these "CompensationCluster" out. The CompensationCluster also has weight 1 for
                // vertices. Importer should skip these clusters.
                if (!cluster || (strcmp(cluster->GetUserDataID(), "Maya_ClusterHint") == 0 && strcmp(cluster->GetUserData(), "CompensationCluster") == 0)) {
                    continue;
                }

                FbxNode* link = cluster->GetLink();
                // find the bone index
                int boneIndex = -1;
                auto linkFound = std::find(sortedLinks.begin(), sortedLinks.end(), link);
                boneIndex = linkFound == sortedLinks.end() ? -1 : static_cast<int>(linkFound - sortedLinks.begin());

                //	get the vertex indices
                int controlPointIndicesCount = cluster->GetControlPointIndicesCount();
                int* controlPointIndices = cluster->GetControlPointIndices();
                double* weights = cluster->GetControlPointWeights();

                //	for each vertex index in the cluster
                for (int controlPointIndex = 0; controlPointIndex < controlPointIndicesCount; controlPointIndex++) {
                    importData->influences.push_back(MeshImportData::RawBoneInfluence());
                    importData->influences.back().boneIndex = boneIndex;
                    importData->influences.back().weight = static_cast<float>(weights[controlPointIndex]);
                    importData->influences.back().vertexIndex = existPointNum + controlPointIndices[controlPointIndex];
                }
            }
        } else  // for rigid mesh
        {
            // find the bone index, the bone is the node itself
            int boneIndex = -1;
            auto linkFound = std::find(sortedLinks.begin(), sortedLinks.end(), fbxNode);
            boneIndex = linkFound == sortedLinks.end() ? -1 : static_cast<int>(linkFound - sortedLinks.begin());

            //	for each vertex in the mesh
            for (int controlPointIndex = 0; controlPointIndex < controlPointsCount; controlPointIndex++) {
                importData->influences.push_back(MeshImportData::RawBoneInfluence());
                importData->influences.back().boneIndex = boneIndex;
                importData->influences.back().weight = 1.0;
                importData->influences.back().vertexIndex = existPointNum + controlPointIndex;
            }
        }
    } else {
        //	for each vertex in the mesh
        for (int controlPointIndex = 0; controlPointIndex < controlPointsCount; controlPointIndex++) {
            importData->influences.push_back(MeshImportData::RawBoneInfluence());
            importData->influences.back().boneIndex = -1;
            importData->influences.back().weight = 1.0;
            importData->influences.back().vertexIndex = existPointNum + controlPointIndex;
        }
    }
    //
    // clean up
    //
    if (layerElementUV) {
        delete[] layerElementUV;
    }
    if (UVReferenceMode) {
        delete[] UVReferenceMode;
    }
    if (UVMappingMode) {
        delete[] UVMappingMode;
    }

    if (importData->materials.size() > 0) {
        std::vector<MeshImportData::RawMaterial> existingMatList = importData->materials;

        std::vector<uint16_t> usedMaterialIndex;
        // Find all material that are use by the mesh faces
        int faceNum = static_cast<int>(importData->faces.size());
        for (int triangleIndex = 0; triangleIndex < faceNum; triangleIndex++) {
            MeshImportData::Triangle& Triangle = importData->faces[triangleIndex];
            if (std::find(usedMaterialIndex.begin(), usedMaterialIndex.end(), Triangle.materialIndex) == usedMaterialIndex.end()) {
                usedMaterialIndex.push_back(Triangle.materialIndex);
            }
        }
        // Remove any unused material.
        if (usedMaterialIndex.size() < existingMatList.size()) {
            std::vector<int> remapIndex;
            std::vector<MeshImportData::RawMaterial>& newMatList = importData->materials;
            newMatList.clear();
            for (int existingMatIndex = 0; existingMatIndex < existingMatList.size(); existingMatIndex++) {
                if (std::find(usedMaterialIndex.begin(), usedMaterialIndex.end(), (uint8_t)existingMatIndex) != usedMaterialIndex.end()) {
                    newMatList.push_back(existingMatList[existingMatIndex]);
                    remapIndex.push_back(static_cast<int>(newMatList.size() - 1));
                } else {
                    remapIndex.push_back(-1);
                }
            }
            importData->maxMaterialIndex = 0;
            // Remap the face material index
            for (int triangleIndex = 0; triangleIndex < faceNum; triangleIndex++) {
                MeshImportData::Triangle& triangle = importData->faces[triangleIndex];
                ASSERT(remapIndex[triangle.materialIndex] != -1);
                triangle.materialIndex = remapIndex[triangle.materialIndex];
                importData->maxMaterialIndex = Maths::Max<uint32_t>(importData->maxMaterialIndex, triangle.materialIndex);
            }
        }
    }

    if (Importer::GetInstance()->options->bPreserveSmoothingGroups) {
        bool bDuplicateUnSmoothWedges = false;  // We deprecate legacy build
        DoUnSmoothVerts(importData, bDuplicateUnSmoothWedges);
    } else {
        for (int ii = 0; ii < importData->points.size(); ii++) {
            importData->pointToRawMap.push_back(0);
        }
        for (int pointIdx = 0; pointIdx < importData->points.size(); pointIdx++) {
            importData->pointToRawMap[pointIdx] = pointIdx;
        }
    }

    if (sceneInfo->scaleFactor != 1.0) {
        for (auto& point : importData->points) {
            point *= sceneInfo->scaleFactor;
        }
    }

    // Create initial bounding box based on expanded version of reference pose for meshes without physics assets.
    // Can be overridden by artist.
    std::shared_ptr<Maths::Box> boundingBox = std::make_shared<Maths::Box>(importData->points);
    std::shared_ptr<Maths::BoxSphereBounds> boundingSphere = std::make_shared<Maths::BoxSphereBounds>(*boundingBox);
    const glm::vec3 boundingBoxSize = boundingBox->GetSize();

    if (importData->points.size() > 2 && boundingBoxSize.x < THRESH_POINTS_ARE_SAME && boundingBoxSize.y < THRESH_POINTS_ARE_SAME && boundingBoxSize.z < THRESH_POINTS_ARE_SAME) {
        LOG_ERROR(fmt::format("Bouding box with size smaller than {} will be discarded.", THRESH_POINTS_ARE_SAME));
        return nullptr;
    }

    importData->boundingSphere = boundingSphere;
    importData->boundingBox = boundingBox;

    return importData;
}
}}  // namespace Fbx::Importer