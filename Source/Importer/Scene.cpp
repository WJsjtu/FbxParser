#include "Scene.h"
#define NOMINMAX
#include "ghc/filesystem.hpp"
#include <set>
#include <numeric>

namespace Fbx { namespace Importer {

Importer::Importer() {
    sdkManager = FbxManager::Create();
    FbxIOSettings* ios = FbxIOSettings::Create(sdkManager, IOSROOT);
    sdkManager->SetIOSettings(ios);
    geometryConverter = std::make_shared<FbxGeometryConverter>(sdkManager);
}

Importer::~Importer() {
    geometryConverter = nullptr;
    if (sdkManager) {
        sdkManager->Destroy();
    }
    sdkManager = nullptr;
}

std::shared_ptr<Importer> Importer::StaticInstance = nullptr;

std::shared_ptr<Importer> Importer::GetInstance() {
    if (!StaticInstance) {
        struct make_shared_enabler : public Importer {};
        StaticInstance = std::make_shared<make_shared_enabler>();
    }
    return StaticInstance;
}

std::string ImporterHelper::MakeName(const char* name) {
    const char specialChars[] = {'.', ',', '/', '`', '%'};

    const int len = static_cast<int>(strlen(name));
    char* tmpName = new char[len + 1];

    strncpy(tmpName, name, len + 1);

    for (int i = 0; i < sizeof(specialChars) / sizeof(char); i++) {
        char* charPtr = tmpName;
        while ((charPtr = strchr(charPtr, specialChars[i])) != 0) {
            charPtr[0] = '_';
        }
    }

    std::string result = tmpName;

    if (result.find(":") != std::string::npos) {
        for (int i = 0; i < result.length(); i++) {
            if (result.at(i) == ':') {
                result.at(i) = '_';
            }
        }
    }
    while (result.back() == ' ') {
        result.pop_back();
    }
    while (result.front() == ' ') {
        result.erase(result.begin());
    }

    delete[] tmpName;
    return result;
}

std::string ImporterHelper::MakeName(const std::string& str) { return MakeName(str.c_str()); }

std::string ImporterHelper::NativeToUTF8(const std::string& str) {
#if _WIN32
    char* u8cstr = nullptr;
    FbxAnsiToUTF8(str.c_str(), u8cstr);
    if (!u8cstr) {
        return str;
    } else {
        std::string u8str = u8cstr;
        delete[] u8cstr;
        return u8str;
    }
#else
    return str;
#endif
}

std::string ImporterHelper::UTF8ToNative(const std::string& str) {
#if _WIN32
    char* u8cstr = nullptr;
    FbxUTF8ToAnsi(str.c_str(), u8cstr);
    if (!u8cstr) {
        return str;
    } else {
        std::string u8str = u8cstr;
        delete[] u8cstr;
        return u8str;
    }
#else
    return str;
#endif
}

FbxNode* ImporterHelper::RecursiveFindParentLodGroup(FbxNode* parentNode) {
    if (parentNode == nullptr) return nullptr;
    if (parentNode->GetNodeAttribute() && parentNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup) return parentNode;
    return RecursiveFindParentLodGroup(parentNode->GetParent());
}

FbxNode* ImporterHelper::RecursiveGetFirstMeshNode(FbxNode* node, FbxNode* nodeToFind) {
    if (node == nullptr) {
        return nullptr;
    }
    if (node->GetMesh() != nullptr) return node;
    for (int childIndex = 0; childIndex < node->GetChildCount(); ++childIndex) {
        FbxNode* meshNode = RecursiveGetFirstMeshNode(node->GetChild(childIndex), nodeToFind);
        if (nodeToFind == nullptr) {
            if (meshNode != nullptr) {
                return meshNode;
            }
        } else if (meshNode == nodeToFind) {
            return meshNode;
        }
    }
    return nullptr;
}

FbxNode* ImporterHelper::FindLODGroupNode(FbxNode* nodeLodGroup, int LodIndex, FbxNode* nodeToFind) {
    ASSERT(nodeLodGroup->GetChildCount() >= LodIndex);
    FbxNode* childNode = nodeLodGroup->GetChild(LodIndex);
    if (childNode == nullptr) {
        return nullptr;
    }
    return RecursiveGetFirstMeshNode(childNode, nodeToFind);
}

void ImporterHelper::GetNodeSampleRate(FbxNode* node, FbxAnimLayer* animLayer, std::vector<int>& nodeAnimSampleRates, bool bCurve, bool bBlendCurve) {
    if (bCurve) {
        const int maxElement = 9;
        FbxAnimCurve* curves[maxElement];

        curves[0] = node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, false);
        curves[1] = node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, false);
        curves[2] = node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, false);
        curves[3] = node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, false);
        curves[4] = node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, false);
        curves[5] = node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, false);
        curves[6] = node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, false);
        curves[7] = node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, false);
        curves[8] = node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, false);

        for (int curveIndex = 0; curveIndex < maxElement; ++curveIndex) {
            FbxAnimCurve* currentCurve = curves[curveIndex];
            if (currentCurve) {
                int curveAnimRate = GetAnimationCurveRate(currentCurve);
                if (curveAnimRate != 0) {
                    if (std::find(nodeAnimSampleRates.begin(), nodeAnimSampleRates.end(), curveAnimRate) == nodeAnimSampleRates.end()) {
                        nodeAnimSampleRates.push_back(curveAnimRate);
                    }
                }
            }
        }
    }

    if (bBlendCurve) {
        FbxGeometry* geometry = (FbxGeometry*)node->GetNodeAttribute();
        if (geometry) {
            int blendShapeDeformerCount = geometry->GetDeformerCount(FbxDeformer::eBlendShape);
            for (int blendShapeIndex = 0; blendShapeIndex < blendShapeDeformerCount; ++blendShapeIndex) {
                FbxBlendShape* blendShape = (FbxBlendShape*)geometry->GetDeformer(blendShapeIndex, FbxDeformer::eBlendShape);

                int blendShapeChannelCount = blendShape->GetBlendShapeChannelCount();
                for (int channelIndex = 0; channelIndex < blendShapeChannelCount; ++channelIndex) {
                    FbxBlendShapeChannel* Channel = blendShape->GetBlendShapeChannel(channelIndex);

                    if (Channel) {
                        FbxAnimCurve* currentCurve = geometry->GetShapeChannel(blendShapeIndex, channelIndex, animLayer);
                        if (currentCurve) {
                            int curveAnimRate = GetAnimationCurveRate(currentCurve);
                            if (curveAnimRate != 0) {
                                if (std::find(nodeAnimSampleRates.begin(), nodeAnimSampleRates.end(), curveAnimRate) == nodeAnimSampleRates.end()) {
                                    nodeAnimSampleRates.push_back(curveAnimRate);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int ImporterHelper::GetAnimationCurveRate(FbxAnimCurve* currentCurve) {
    if (currentCurve == nullptr) return 0;

    int keyCount = currentCurve->KeyGetCount();

    FbxTimeSpan timeInterval(FBXSDK_TIME_INFINITE, FBXSDK_TIME_MINUS_INFINITE);
    bool bValidTimeInterval = currentCurve->GetTimeInterval(timeInterval);
    if (keyCount > 1 && bValidTimeInterval) {
        double keyAnimLength = timeInterval.GetDuration().GetSecondDouble();
        if (keyAnimLength != 0.0) {
            //////////////////////////////////////////////////////////////////////////
            // 1. Look if we have high frequency keys(resampling).

            // Basic sample rate is compute by dividing the KeyCount by the anim length. This is valid only if
            // all keys are time equidistant. But if we find a rate over DEFAULT_SAMPLERATE, we can estimate that
            // there is a constant frame rate between the key and simply return the rate.
            int sampleRate = Maths::RoundToInt((keyCount - 1) / keyAnimLength);
            if (sampleRate >= DEFAULT_SAMPLERATE) {
                // We import a curve with more then 30 keys per frame
                return sampleRate;
            }

            //////////////////////////////////////////////////////////////////////////
            // 2. Compute the sample rate of every keys with there time. Use the
            //    least common multiplier to get a sample rate that go through all keys.

            sampleRate = 1;
            float oldKeyTime = 0.0f;
            std::set<int> deltaComputed;
            // Reserve some space
            // DeltaComputed.Reserve(30);
            const float keyMultiplier = (1.0f / KINDA_SMALL_NUMBER);
            // Find also the smallest delta time between keys
            for (int keyIndex = 0; keyIndex < keyCount; ++keyIndex) {
                float keyTime = (float)(currentCurve->KeyGet(keyIndex).GetTime().GetSecondDouble());
                // Collect the smallest delta time, there is no delta in case the first animation key time is negative
                float delta = (keyTime < 0 && keyIndex == 0) ? 0.0f : keyTime - oldKeyTime;
                // use the fractional part of the delta to have the delta between 0.0f and 1.0f
                delta = Maths::Fractional(delta);
                int deltaKey = Maths::RoundToInt(delta * keyMultiplier);
                if (!Maths::IsNearlyZero(delta, KINDA_SMALL_NUMBER) && (deltaComputed.find(deltaKey) == deltaComputed.end())) {
                    int computeSampleRate = GetTimeSampleRate(delta);
                    deltaComputed.insert(deltaKey);
                    // Use the least common multiplier with the new delta entry
                    int leastCommonMultiplier = Maths::Min(Maths::LeastCommonMultiplier(sampleRate, computeSampleRate), Maths::RoundToInt(MaxReferenceRate));
                    sampleRate = leastCommonMultiplier != 0 ? leastCommonMultiplier : Maths::Max3(Maths::RoundToInt(DEFAULT_SAMPLERATE), sampleRate, computeSampleRate);
                }
                oldKeyTime = keyTime;
            }
            return sampleRate;
        }
    }

    return 0;
}

// Get the smallest sample rate(integer) representing the DeltaTime(time between 0.0f and 1.0f).
//@DeltaTime: the time to find the rate between 0.0f and 1.0f
int ImporterHelper::GetTimeSampleRate(const float deltaTime) {
    float originalSampleRateDivider = 1.0f / deltaTime;
    float sampleRateDivider = originalSampleRateDivider;
    float sampleRemainder = Maths::Fractional(sampleRateDivider);
    float multiplier = 2.0f;
    float integerPrecision = Maths::Min(Maths::Max(KINDA_SMALL_NUMBER * sampleRateDivider, KINDA_SMALL_NUMBER),
                                        0.1f);  // The precision is limit between KINDA_SMALL_NUMBER and 0.1f
    while (!Maths::IsNearlyZero(sampleRemainder, integerPrecision) && !Maths::IsNearlyEqual(sampleRemainder, 1.0f, integerPrecision)) {
        sampleRateDivider = originalSampleRateDivider * multiplier;
        sampleRemainder = Maths::Fractional(sampleRateDivider);
        if (sampleRateDivider > MaxReferenceRate) {
            sampleRateDivider = DEFAULT_SAMPLERATE;
            break;
        }
        multiplier += 1.0f;
    }
    return Maths::Min(Maths::RoundToInt(sampleRateDivider), Maths::RoundToInt(MaxReferenceRate));
}

bool ImporterHelper::IsOddNegativeScale(FbxAMatrix& matrix) {
    FbxVector4 Scale = matrix.GetS();
    int NegativeNum = 0;

    if (Scale[0] < 0) NegativeNum++;
    if (Scale[1] < 0) NegativeNum++;
    if (Scale[2] < 0) NegativeNum++;

    return NegativeNum == 1 || NegativeNum == 3;
}

bool CompressRawAnimSequenceTrack(FRawAnimSequenceTrack& rawTrack, int numFrames, std::string errorName, float maxPosDiff, float maxAngleDiff) {
    bool bRemovedKeys = false;

    for (int i = 0; i < rawTrack.posKeys.size(); i++) {
        rawTrack.posIndices.push_back(i);
    }
    for (int i = 0; i < rawTrack.scaleKeys.size(); i++) {
        rawTrack.scaleIndices.push_back(i);
    }

    for (int i = 0; i < rawTrack.rotKeys.size(); i++) {
        rawTrack.rotIndices.push_back(i);
    }
    // Check variation of position keys
    if ((rawTrack.posKeys.size() > 1) && (maxPosDiff >= 0.0f)) {
        glm::vec3 FirstPos = rawTrack.posKeys[0];
        bool bFramesIdentical = true;
        for (int j = 1; j < rawTrack.posKeys.size() && bFramesIdentical; j++) {
            if (glm::length(FirstPos - rawTrack.posKeys[j]) > maxPosDiff) {
                bFramesIdentical = false;
            }
        }

        // If all keys are the same, remove all but first frame
        if (bFramesIdentical) {
            bRemovedKeys = true;
            rawTrack.posKeys.resize(1);
            rawTrack.posIndices.resize(1);
            ASSERT(rawTrack.posKeys.size() == 1);
        }
    }

    // Check variation of rotational keys
    if ((rawTrack.rotKeys.size() > 1) && (maxAngleDiff >= 0.0f)) {
        glm::quat firstRot = rawTrack.rotKeys[0];
        bool bFramesIdentical = true;
        for (int j = 1; j < rawTrack.rotKeys.size() && bFramesIdentical; j++) {
            auto quatError = [](const glm::quat& Q1, const glm::quat& Q2) -> float {
                const float cosom = abs(Q1.x * Q2.x + Q1.y * Q2.y + Q1.z * Q2.z + Q1.w * Q2.w);
                return (abs(cosom) < 0.9999999f) ? acos(cosom) * (1.f / 3.1415926535) : 0.0f;
            };

            if (quatError(firstRot, rawTrack.rotKeys[j]) > maxAngleDiff) {
                bFramesIdentical = false;
            }
        }

        // If all keys are the same, remove all but first frame
        if (bFramesIdentical) {
            bRemovedKeys = true;
            rawTrack.rotKeys.resize(1);
            rawTrack.rotIndices.resize(1);
            ASSERT(rawTrack.rotKeys.size() == 1);
        }
    }

    float maxScaleDiff = 0.0001f;

    // Check variation of Scaleition keys
    if ((rawTrack.scaleKeys.size() > 1) && (maxScaleDiff >= 0.0f)) {
        glm::vec3 firstScale = rawTrack.scaleKeys[0];
        bool bFramesIdentical = true;
        for (int j = 1; j < rawTrack.scaleKeys.size() && bFramesIdentical; j++) {
            if (glm::length(firstScale - rawTrack.scaleKeys[j]) > maxScaleDiff) {
                bFramesIdentical = false;
            }
        }

        // If all keys are the same, remove all but first frame
        if (bFramesIdentical) {
            bRemovedKeys = true;
            rawTrack.scaleKeys.resize(1);
            rawTrack.scaleIndices.resize(1);
            ASSERT(rawTrack.scaleKeys.size() == 1);
        }
    }

    return bRemovedKeys;
}

bool ImporterHelper::CompressRawAnimData(std::vector<FRawAnimSequenceTrack>& rawAnimationData, int numFrames, std::string errorName, float maxPosDiff, float maxAngleDiff) {
    bool bRemovedKeys = false;
    // This removes trivial keys, and this has to happen before the removing tracks
    for (int trackIndex = 0; trackIndex < rawAnimationData.size(); trackIndex++) {
        bRemovedKeys |= CompressRawAnimSequenceTrack(rawAnimationData[trackIndex], numFrames, errorName, maxPosDiff, maxAngleDiff);
    }

    // bool bCompressScaleKeys = false;
    //// go through remove keys if not needed
    // for (int trackIndex = 0; trackIndex < rawAnimationData.size(); trackIndex++) {
    //    FRawAnimSequenceTrack const& rawData = rawAnimationData[trackIndex];
    //    if (rawData.scaleKeys.size() > 0) {
    //        // if scale key exists, see if we can just empty it

    //        if ((rawData.scaleKeys.size() > 1) || glm::all(glm::equal(rawData.scaleKeys[0], glm::vec3(1, 1, 1))) == false) {
    //            bCompressScaleKeys = true;
    //            break;
    //        }
    //    }
    //}

    //// if we don't have scale, we should delete all scale keys
    //// if you have one track that has scale, we still should support scale, so compress scale
    // if (!bCompressScaleKeys) {
    //    // then remove all scale keys
    //    for (int trackIndex = 0; trackIndex < rawAnimationData.size(); trackIndex++) {
    //        FRawAnimSequenceTrack& rawData = rawAnimationData[trackIndex];
    //        rawData.scaleKeys.clear();
    //    }
    //}
    return bRemovedKeys;
}

SceneInfo::~SceneInfo() {
    if (importer) {
        importer->Destroy();
        importer = nullptr;
    }
    if (scene) {
        scene->Destroy();
        scene = nullptr;
    }
    nodeUniqueNameToOriginalNameMap.Clear();
    FbxTextureToUniqueNameMap.clear();
}

std::shared_ptr<SceneInfo> Importer::GetFileSceneInfo(const std::string& filename, std::shared_ptr<Options> options) {
    int SDKMajor, SDKMinor, SDKRevision;
    std::shared_ptr<SceneInfo> result = std::make_shared<SceneInfo>();
    result->importer = FbxImporter::Create(sdkManager, "");
    if (options->embeddingExtractionFolder != "") {
        result->importer->SetEmbeddingExtractionFolder(ImporterHelper::NativeToUTF8(options->embeddingExtractionFolder).c_str());
    }

    // Get the version number of the FBX files generated by the
    // version of FBX SDK that you are using.
    FbxManager::GetFileFormatVersion(SDKMajor, SDKMinor, SDKRevision);

    // Initialize the importer by providing a filename.
    const bool importSuccess = result->importer->Initialize(filename.c_str());

    FbxIOFileHeaderInfo* fileHeaderInfo = result->importer->GetFileHeaderInfo();
    if (fileHeaderInfo) {
        // Example of creator file info string
        // Blender (stable FBX IO) - 2.78 (sub 0) - 3.7.7
        // Maya and Max use the same string where they specify the fbx sdk version, so we cannot know it is coming
        // from which software We need blender creator when importing skinned mesh containing the "armature" dummy
        // node as the parent of the root joint. We want to remove this dummy "armature" node
        std::string creatorStr(fileHeaderInfo->mCreator.Buffer());
        if (creatorStr.rfind("Blender", 0) == 0) {
            result->bIsCreateByBlender = true;
        }
    }
    if (!importSuccess)  // Problem with the file to be imported
    {
        std::string errorMessage = "";
        std::string detail = ImporterHelper::UTF8ToNative(result->importer->GetStatus().GetErrorString());
        errorMessage += "FBX 文件打开错误：" + detail;
        LOG_ERROR(errorMessage);
        if (result->importer->GetStatus().GetCode() == FbxStatus::eInvalidFileVersion) {
            LOG_ERROR("FBX文件版本过时，解析使用的FBX SDK版本为" + std::to_string(SDKMajor) + "." + std::to_string(SDKMinor) + "." + std::to_string(SDKRevision) + " 。");
        }
        return nullptr;
    }

    // Version out of date warning
    int FileMajor = 0, FileMinor = 0, FileRevision = 0;
    result->importer->GetFileVersion(FileMajor, FileMinor, FileRevision);
    int FileVersion = (FileMajor << 16 | FileMinor << 8 | FileRevision);
    int SDKVersion = (SDKMajor << 16 | SDKMinor << 8 | SDKRevision);
    if (FileVersion != SDKVersion) {
        // Appending the SDK version to the config key causes the warning to automatically reappear even if
        // previously suppressed when the SDK version we use changes.

        std::string fileVerStr = std::to_string(FileMajor) + "." + std::to_string(FileMinor) + "." + std::to_string(FileRevision);
        std::string sdkVerStr = std::to_string(SDKMajor) + "." + std::to_string(SDKMinor) + "." + std::to_string(SDKRevision);

        const std::string warningText = "检测到较为过时的FBX文件：导入的FBX文件版本为" + fileVerStr + "，解析使用的FBXSDK版本为" + sdkVerStr + "，解析过程有可能会出现一些非预期的结果。";
        LOG_WARN(warningText);
    }

    bool bStatus;

    result->fileBasePath = ghc::filesystem::path(filename).parent_path().generic_string();

    // Create the Scene
    result->scene = FbxScene::Create(sdkManager, "");
    LOG_INFO("正在从FBX文件" + filename + "中加载场景……");

    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_MATERIAL, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_TEXTURE, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_LINK, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_SHAPE, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_GOBO, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_ANIMATION, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_SKINS, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_DEFORMATION, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, true);
    (*(result->importer->GetIOSettings())).SetBoolProp(IMP_TAKE, true);

    // Import the scene.
    bStatus = result->importer->Import(result->scene);

    if (!bStatus) {
        std::string errorMessage = ImporterHelper::UTF8ToNative(result->importer->GetStatus().GetErrorString());
        LOG_ERROR("FBX场景加载失败：" + errorMessage);
        return nullptr;
    }

    {
        std::string baseFilename = (ghc::filesystem::path(filename).stem().generic_string());

        std::set<std::string> allNodeName;
        int currentNameIndex = 1;
        for (int nodeIndex = 0; nodeIndex < result->scene->GetNodeCount(); nodeIndex++) {
            FbxNode* node = result->scene->GetNode(nodeIndex);
            std::string nodeName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(node->GetName()));
            if (nodeName.empty()) {
                do {
                    nodeName = "ncl1_" + std::to_string(currentNameIndex++);
                } while (allNodeName.find(nodeName) != allNodeName.end());

                node->SetName(ImporterHelper::NativeToUTF8(nodeName).c_str());
                LOG_WARN("FBX文件中包含未命名的节点，自动分配节点名：" + nodeName + "。");
            }
            // Do not allow node to be named same as filename as this creates problems later on (reimport)
            if (allNodeName.find(nodeName) != allNodeName.end() || Utils::InsensitiveCaseEquals(nodeName, baseFilename)) {
                std::string uniqueNodeName;
                do {
                    uniqueNodeName = nodeName + std::to_string(currentNameIndex++);
                } while (allNodeName.find(uniqueNodeName) != allNodeName.end());

                FbxString UniqueName(ImporterHelper::NativeToUTF8(uniqueNodeName).c_str());
                result->nodeUniqueNameToOriginalNameMap[UniqueName] = node->GetName();
                node->SetName(UniqueName);
                LOG_WARN("FBX文件中包含命名相同的节点，自动重命名节点：" + nodeName + " -> " + uniqueNodeName + "。");
            }
            allNodeName.insert(nodeName);
        }
    }

    {
        FbxArray<FbxSurfaceMaterial*> materialArray;
        result->scene->FillMaterialArray(materialArray);

        std::set<std::string> allMaterialAndTextureNames;
        std::set<FbxFileTexture*> materialTextures;

        auto FixNameIfNeeded = [this, &allMaterialAndTextureNames](const std::string& assetName, std::function<void(const std::string& /*UniqueName*/)> applyUniqueNameFunction, std::function<std::string(const std::string& /*UniqueName*/)> getErrorTextFunction) {
            std::string uniqueName(assetName);
            if (allMaterialAndTextureNames.find(uniqueName) != allMaterialAndTextureNames.end()) {
                // Use the fbx nameclash 1 convention: NAMECLASH1_KEY
                // This will add _ncl1_
                std::string assetBaseName = uniqueName + "_ncl1_";
                int nameIndex = 1;
                do {
                    uniqueName = assetBaseName + std::to_string(nameIndex++);
                } while (allMaterialAndTextureNames.find(uniqueName) != allMaterialAndTextureNames.end());

                // Apply the unique name.
                applyUniqueNameFunction(uniqueName);
                LOG_WARN(getErrorTextFunction(uniqueName));
            }
            allMaterialAndTextureNames.insert(uniqueName);
        };

        auto GetFbxMaterialTextures = [](const FbxSurfaceMaterial& material) -> std::set<FbxFileTexture*> {
            std::set<FbxFileTexture*> textureSet;

            int textureIndex;
            FBXSDK_FOR_EACH_TEXTURE(textureIndex) {
                FbxProperty property = material.FindProperty(FbxLayerElement::sTextureChannelNames[textureIndex]);

                if (property.IsValid()) {
                    // We use auto as the parameter type to allow for a generic lambda accepting both FbxProperty and
                    // FbxLayeredTexture
                    auto AddSrcTextureToSet = [&textureSet](const auto& inObject) {
                        int NbTextures = inObject.template GetSrcObjectCount<FbxTexture>();
                        for (int texIndex = 0; texIndex < NbTextures; ++texIndex) {
                            FbxFileTexture* texture = inObject.template GetSrcObject<FbxFileTexture>(texIndex);
                            if (texture) {
                                textureSet.insert(texture);
                            }
                        }
                    };

                    // Here we have to check if it's layered textures, or just textures:
                    const int layeredTextureCount = property.GetSrcObjectCount<FbxLayeredTexture>();
                    if (layeredTextureCount > 0) {
                        for (int layerIndex = 0; layerIndex < layeredTextureCount; ++layerIndex) {
                            if (const FbxLayeredTexture* lLayeredTexture = property.GetSrcObject<FbxLayeredTexture>(layerIndex)) {
                                AddSrcTextureToSet(*lLayeredTexture);
                            }
                        }
                    } else {
                        // no layered texture simply get on the property
                        AddSrcTextureToSet(property);
                    }
                }
            }

            return textureSet;
        };

        // First rename materials to unique names and gather their texture.
        for (int materialIndex = 0; materialIndex < materialArray.Size(); ++materialIndex) {
            FbxSurfaceMaterial* material = materialArray[materialIndex];
            std::string materialName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(material->GetName()));
            auto materialTextures = GetFbxMaterialTextures(*material);
            materialTextures.insert(materialTextures.begin(), materialTextures.end());

            material->SetName(ImporterHelper::NativeToUTF8(materialName).c_str());

            FixNameIfNeeded(materialName, [&](const std::string& UniqueName) { material->SetName(ImporterHelper::NativeToUTF8(UniqueName).c_str()); }, [&](const std::string& UniqueName) { return "FBX文件中包含重名的材质，已重命名材质'" + materialName + "'-> '" + UniqueName + "'。"; });
        }

        // Then rename make sure the texture have unique names as well.
        for (FbxFileTexture* currentTexture : materialTextures) {
            std::string absoluteFilename = ImporterHelper::UTF8ToNative(currentTexture->GetFileName());
            std::string textureName = ghc::filesystem::path(absoluteFilename).stem().generic_string();
            auto SanitizeInvalidCharsInline = [](std::string& InText, const char* InvalidChars = INVALID_OBJECTNAME_CHARACTERS) {
                const char* InvalidChar = InvalidChars ? InvalidChars : "";
                while (*InvalidChar) {
                    for (int i = 0; i < InText.length(); i++) {
                        if (InText.at(i) == *InvalidChar) {
                            InText.at(i) = '_';
                        }
                    }
                    ++InvalidChar;
                }
                return InText;
            };
            textureName = SanitizeInvalidCharsInline(textureName);

            FixNameIfNeeded(textureName, [&](const std::string& UniqueName) { result->FbxTextureToUniqueNameMap.emplace(currentTexture, UniqueName); }, [&](const std::string& UniqueName) { return "FBX文件中包含重名的贴图，已重命名贴图'" + textureName + "'-> '" + UniqueName + "'。"; });
        }
    }

    {
        int FileMajor, FileMinor, FileRevision;
        // Get the version number of the FBX file format.
        result->importer->GetFileVersion(FileMajor, FileMinor, FileRevision);
        result->FbxFileVersion = std::to_string(FileMajor) + "." + std::to_string(FileMinor) + "." + std::to_string(FileRevision);

        result->FbxFileCreator = result->importer->GetFileHeaderInfo()->mCreator.Buffer();
        // output result
        LOG_INFO("FBX场景加载成功。");

        const FbxGlobalSettings& globalSettings = result->scene->GetGlobalSettings();
        FbxTime::EMode TimeMode = globalSettings.GetTimeMode();
        // Set the original framerate from the current fbx file
        result->originalFbxFramerate = FbxTime::GetFrameRate(TimeMode);
        result->originalFileAxisSystem = result->scene->GetGlobalSettings().GetAxisSystem();
        result->originalFileUnitSystem = result->scene->GetGlobalSettings().GetSystemUnit();

        FbxDocumentInfo* docInfo = result->scene->GetSceneInfo();
        if (docInfo) {
            result->FbxLastSavedVendor = ImporterHelper::UTF8ToNative(docInfo->LastSaved_ApplicationVendor.Get().Buffer());
            result->FBXLastSavedAppName = ImporterHelper::UTF8ToNative(docInfo->LastSaved_ApplicationName.Get().Buffer());
            result->FBXLastSavedAppVersion = ImporterHelper::UTF8ToNative(docInfo->LastSaved_ApplicationVersion.Get().Buffer());
        }
    }

    {
        FbxTimeSpan globalTimeSpan(FBXSDK_TIME_INFINITE, FBXSDK_TIME_MINUS_INFINITE);

        result->totalMaterialNum = result->scene->GetMaterialCount();
        result->totalTextureNum = result->scene->GetTextureCount();
        result->totalGeometryNum = 0;
        result->nonSkinnedMeshNum = 0;
        result->skinnedMeshNum = 0;
        for (int geometryIndex = 0; geometryIndex < result->scene->GetGeometryCount(); geometryIndex++) {
            FbxGeometry* geometry = result->scene->GetGeometry(geometryIndex);
            if (geometry->GetAttributeType() == FbxNodeAttribute::eMesh) {
                FbxNode* geoNode = geometry->GetNode();
                FbxMesh* mesh = (FbxMesh*)geometry;
                // Skip staticmesh sub LOD group that will be merge with the other same lod index mesh
                // if (geoNode && mesh->GetDeformerCount(FbxDeformer::eSkin) <= 0) {
                //    FbxNode* parentNode = ImporterHelper::RecursiveFindParentLodGroup(geoNode->GetParent());
                //    if (parentNode != nullptr && parentNode->GetNodeAttribute() &&
                //        parentNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup) {
                //        bool bIsLodRoot = false;
                //        for (int childIndex = 0; childIndex < parentNode->GetChildCount(); ++childIndex) {
                //            FbxNode* MeshNode = ImporterHelper::FindLODGroupNode(parentNode, childIndex);
                //            if (geoNode == MeshNode) {
                //                bIsLodRoot = true;
                //                break;
                //            }
                //        }
                //        if (!bIsLodRoot) {
                //            // Skip static mesh sub LOD
                //            continue;
                //        }
                //    }
                //}
                result->totalGeometryNum++;

                FbxMeshInfo& meshInfo = result->meshInfo.emplace(geoNode, FbxMeshInfo()).first->second;
                if (geometry->GetName()[0] != '\0')
                    meshInfo.name = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(geometry->GetName()));
                else
                    meshInfo.name = geoNode ? ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(geoNode->GetName())) : "None";
                meshInfo.bTriangulated = mesh->IsTriangleMesh();
                meshInfo.materialNum = geoNode ? geoNode->GetMaterialCount() : 0;
                meshInfo.faceNum = mesh->GetPolygonCount();
                meshInfo.vertexNum = mesh->GetControlPointsCount();
                meshInfo.owner = geoNode;

                // LOD info
                meshInfo.LODGroup = "";
                if (geoNode) {
                    FbxNode* parentNode = ImporterHelper::RecursiveFindParentLodGroup(geoNode->GetParent());
                    if (parentNode != nullptr && parentNode->GetNodeAttribute() && parentNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup) {
                        meshInfo.LODGroup = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(parentNode->GetName()));
                        for (int LODIndex = 0; LODIndex < parentNode->GetChildCount(); LODIndex++) {
                            FbxNode* meshNode = ImporterHelper::FindLODGroupNode(parentNode, LODIndex, geoNode);
                            if (geoNode == meshNode) {
                                meshInfo.LODLevel = LODIndex;
                                meshInfo.LODParent = parentNode;
                                break;
                            }
                        }
                    }
                }

                // skinned mesh
                if (mesh->GetDeformerCount(FbxDeformer::eSkin) > 0) {
                    result->skinnedMeshNum++;
                    meshInfo.bIsSkelMesh = true;
                    meshInfo.morphNum = mesh->GetShapeCount();
                    // skeleton root
                    FbxSkin* skin = (FbxSkin*)mesh->GetDeformer(0, FbxDeformer::eSkin);
                    int clusterCount = skin->GetClusterCount();
                    FbxNode* link = NULL;
                    for (int clusterId = 0; clusterId < clusterCount; ++clusterId) {
                        FbxCluster* cluster = skin->GetCluster(clusterId);
                        link = cluster->GetLink();
                        while (link && link->GetParent() && link->GetParent()->GetSkeleton()) {
                            link = link->GetParent();
                        }

                        if (link != NULL) {
                            break;
                        }
                    }

                    meshInfo.skeletonRoot = link ? ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(link->GetName())) : "None";
                    meshInfo.skeletonElemNum = link ? link->GetChildCount(true) : 0;

                    if (link) {
                        FbxTimeSpan animTimeSpan(FBXSDK_TIME_INFINITE, FBXSDK_TIME_MINUS_INFINITE);
                        link->GetAnimationInterval(animTimeSpan);
                        globalTimeSpan.UnionAssignment(animTimeSpan);
                    }
                } else {
                    result->nonSkinnedMeshNum++;
                    meshInfo.bIsSkelMesh = false;
                    meshInfo.skeletonRoot = "";
                }
                meshInfo.uniqueId = mesh->GetUniqueID();
            }
        }

        result->bHasAnimation = false;
        int animCurveNodeCount = result->scene->GetSrcObjectCount<FbxAnimCurveNode>();
        // sadly Max export with animation curve node by default without any change, so
        // we'll have to skip the first two curves, which is translation/rotation
        // if there is a valid animation, we'd expect there are more curve nodes than 2.
        for (int animCurveNodeIndex = 2; animCurveNodeIndex < animCurveNodeCount; animCurveNodeIndex++) {
            FbxAnimCurveNode* CurAnimCruveNode = result->scene->GetSrcObject<FbxAnimCurveNode>(animCurveNodeIndex);
            if (CurAnimCruveNode->IsAnimated(true)) {
                result->bHasAnimation = true;
                break;
            }
        }

        result->frameRate = FbxTime::GetFrameRate(result->scene->GetGlobalSettings().GetTimeMode());

        if (globalTimeSpan.GetDirection() == FBXSDK_TIME_FORWARD) {
            result->totalTime = (globalTimeSpan.GetDuration().GetMilliSeconds()) / 1000.f * result->frameRate;
        } else {
            result->totalTime = 0;
        }
    }

    {
        std::set<FbxUInt64> nodeGeometryIds;
        for (int nodeIndex = 0; nodeIndex < result->scene->GetNodeCount(); ++nodeIndex) {
            FbxNode* sceneNode = result->scene->GetNode(nodeIndex);
            FbxGeometry* nodeGeometry = static_cast<FbxGeometry*>(sceneNode->GetMesh());

            if (nodeGeometry) {
                nodeGeometryIds.insert(nodeGeometry->GetUniqueID());
            }
        }

        for (int geoIndex = 0; geoIndex < result->scene->GetGeometryCount(); ++geoIndex) {
            FbxGeometry* geometry = result->scene->GetGeometry(geoIndex);

            if (nodeGeometryIds.find(geometry->GetUniqueID()) == nodeGeometryIds.end()) {
                std::string geometryName = (geometry->GetName() && geometry->GetName()[0] != '\0') ? ImporterHelper::UTF8ToNative(geometry->GetName()) : "[未命名的几何体]";
                LOG_WARN("FBX文件中网格 " + geometryName + "没有被场景的任何节点引用。");
            }
        }
    }

    {
        const std::string LodPrefix = "LOD";
        std::map<std::string, std::vector<uint64_t>> LodSuffixNodeMap;
        std::map<std::string, std::string> LodSuffixNodeNameMap;
        std::map<uint64_t, FbxNode*> nodeMap;
        for (int nodeIndex = 0; nodeIndex < result->scene->GetNodeCount(); nodeIndex++) {
            FbxNode* sceneNode = result->scene->GetNode(nodeIndex);
            if (sceneNode == nullptr) {
                continue;
            }
            FbxGeometry* nodeGeometry = static_cast<FbxGeometry*>(sceneNode->GetMesh());
            if (nodeGeometry && nodeGeometry->GetUniqueID() != sceneNode->GetUniqueID()) {
                std::string sceneNodeName = ImporterHelper::UTF8ToNative(sceneNode->GetName());
                if (sceneNodeName.size() < 5) {
                    continue;
                }
                auto range = sceneNodeName.find(LodPrefix);
                if (range != std::string::npos) {
                    continue;
                }
                if (sceneNodeName.size() - range - 3 <= 0) {
                    continue;
                }
                char charAt4 = *(sceneNodeName.begin() + range + 3);
                std::string LODXNumber;
                if (charAt4 == '_') {
                    LODXNumber = sceneNodeName.substr(range + 4);
                } else {
                    LODXNumber = sceneNodeName.substr(range + 3);
                }
                if (Utils::IsNumeric(LODXNumber)) {
                    nodeMap[sceneNode->GetUniqueID()] = sceneNode;
                    int LodNumber = atoi(LODXNumber.c_str());
                    std::string matchName = sceneNodeName.substr(0, range);
                    std::string LODName = matchName;
                    if (sceneNode->GetParent()) {
                        uint64_t ParentUniqueID = sceneNode->GetParent()->GetUniqueID();
                        std::string ParentID = std::to_string(ParentUniqueID);
                        LODName = ParentID + "_" + matchName;
                    }
                    std::vector<uint64_t>& LodSuffixNodeValues = LodSuffixNodeMap[LODName];
                    LodSuffixNodeNameMap[LODName] = matchName;
                    // Add LOD in the correct order
                    if (LodNumber >= LodSuffixNodeValues.size()) {
                        int addCount = LodNumber + 1 - LodSuffixNodeValues.size();
                        for (int addIndex = 0; addIndex < addCount; ++addIndex) {
                            LodSuffixNodeValues.push_back(std::numeric_limits<uint64_t>::max());
                        }
                    }
                    LodSuffixNodeValues[LodNumber] = sceneNode->GetUniqueID();
                }
            }
        }

        for (const auto& kvp : LodSuffixNodeMap) {
            if (kvp.second.size() <= 1) {
                continue;
            }
            // Find the first valid node to be able to discover the parent of this LOD Group
            const std::vector<uint64_t>& LodGroupNodes = kvp.second;
            FbxNode* firstNode = nullptr;
            int validNodeCount = 0;
            for (int currentLodIndex = 0; currentLodIndex < LodGroupNodes.size(); ++currentLodIndex) {
                if (LodGroupNodes[currentLodIndex] != std::numeric_limits<uint64_t>::max()) {
                    if (firstNode == nullptr) {
                        firstNode = nodeMap[LodGroupNodes[currentLodIndex]];
                    }
                    validNodeCount++;
                }
            }
            // Do not create LODGroup with less then two child
            if (validNodeCount <= 1) {
                continue;
            }
            ASSERT(firstNode != nullptr);
            // Set the parent node, we assume all node in LodGroupNodes have the same parent
            FbxNode* parentNode = firstNode->GetParent() == nullptr ? result->scene->GetRootNode() : firstNode->GetParent();
            if (parentNode->GetNodeAttribute() && parentNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup) {
                // LODGroup already exist no need to create one
                continue;
            }

            for (int currentLodIndex = 0; currentLodIndex < LodGroupNodes.size(); currentLodIndex++) {
                if (LodGroupNodes[currentLodIndex] == std::numeric_limits<uint64_t>::max()) {
                    LOG_WARN("FBX文件中节点" + ImporterHelper::UTF8ToNative(parentNode->GetName()) + "下缺少LOD level为" + std::to_string(currentLodIndex) + "的网格信息。");
                    continue;
                }
                FbxNode* currentNode = nodeMap[LodGroupNodes[currentLodIndex]];
                auto meshFound = result->meshInfo.find(currentNode);
                if (meshFound != result->meshInfo.end()) {
                    meshFound->second.LODGroup = LodSuffixNodeNameMap[kvp.first] + "LODGroup";
                    meshFound->second.LODLevel = currentLodIndex;
                    meshFound->second.LODParent = parentNode;
                }
            }
        }

        for (const auto& kvp : result->meshInfo) {
            if (kvp.second.LODParent) {
                result->meshLODInfo[kvp.second.LODParent].push_back(kvp.first);
            }
        }
    }

    return result;
}

std::shared_ptr<Scene> Importer::ImportScene(std::shared_ptr<SceneInfo> sceneInfo, std::shared_ptr<Options> options) {
    this->options = options;
    std::shared_ptr<Scene> scene = ConvertScene(sceneInfo);
    if (scene) {
        scene->ProcessMesh();
        scene->ProcessSkinnedMesh();
        scene->ProcessAnimation();
    }
    return scene;
}

std::shared_ptr<Scene> Importer::ConvertScene(std::shared_ptr<SceneInfo> sceneInfo) {
    if (!sceneInfo || !options) {
        return nullptr;
    }

    // Merge the anim stack before the conversion since the above 0 layer will not be converted
    int animStackCount = sceneInfo->scene->GetSrcObjectCount<FbxAnimStack>();
    // Merge the animation stack layer before converting the scene
    for (int animStackIndex = 0; animStackIndex < animStackCount; animStackIndex++) {
        FbxAnimStack* curAnimStack = sceneInfo->scene->GetSrcObject<FbxAnimStack>(animStackIndex);
        if (curAnimStack->GetMemberCount() > 1) {
            int resampleRate = DEFAULT_SAMPLERATE;
            {
                if (options->bResample) {
                    std::vector<int> curveAnimSampleRates;
                    int maxStackResampleRate = 0;
                    int animStackLayerCount = curAnimStack->GetMemberCount();
                    for (int layerIndex = 0; layerIndex < animStackLayerCount; ++layerIndex) {
                        FbxAnimLayer* animLayer = (FbxAnimLayer*)curAnimStack->GetMember(layerIndex);
                        for (int nodeIndex = 0; nodeIndex < sceneInfo->scene->GetNodeCount(); ++nodeIndex) {
                            FbxNode* node = sceneInfo->scene->GetNode(nodeIndex);
                            // Get both the transform properties curve and the blend shape animation sample rate
                            ImporterHelper::GetNodeSampleRate(node, animLayer, curveAnimSampleRates, true, true);
                        }
                    }

                    maxStackResampleRate = curveAnimSampleRates.size() > 0 ? 1 : maxStackResampleRate;
                    // Find the lowest sample rate that will pass by all the keys from all curves
                    for (int curveSampleRate : curveAnimSampleRates) {
                        if (curveSampleRate >= MaxReferenceRate && maxStackResampleRate < curveSampleRate) {
                            maxStackResampleRate = curveSampleRate;
                        } else if (maxStackResampleRate < MaxReferenceRate) {
                            int leastCommonMultiplier = Maths::LeastCommonMultiplier(maxStackResampleRate, curveSampleRate);
                            maxStackResampleRate = leastCommonMultiplier != 0 ? leastCommonMultiplier : Maths::Max3(Maths::RoundToInt(DEFAULT_SAMPLERATE), maxStackResampleRate, curveSampleRate);
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
                        resampleRate = maxStackResampleRate;
                    }
                }
            }

            FbxTime lFramePeriod;
            lFramePeriod.SetSecondDouble(1.0 / resampleRate);

            FbxTimeSpan lTimeSpan = curAnimStack->GetLocalTimeSpan();
            curAnimStack->BakeLayers(sceneInfo->scene->GetAnimationEvaluator(), lTimeSpan.GetStart(), lTimeSpan.GetStop(), lFramePeriod);

            // always apply unroll filter
            FbxAnimCurveFilterUnroll unrollFilter;

            FbxAnimLayer* lLayer = curAnimStack->GetMember<FbxAnimLayer>(0);
            unrollFilter.Reset();

            // The Unroll filter expects only rotation curves, we need to walk the scene and extract the
            // rotation curves from the nodes property. This can become time consuming but we have no choice.
            std::function<void(FbxNode*, FbxAnimLayer*, FbxAnimCurveFilterUnroll*)> ApplyUnroll = [&](FbxNode* pNode, FbxAnimLayer* pLayer, FbxAnimCurveFilterUnroll* pUnrollFilter) -> void {
                if (!pNode || !pLayer || !pUnrollFilter) {
                    return;
                }

                FbxAnimCurveNode* lCN = pNode->LclRotation.GetCurveNode(pLayer);
                if (lCN) {
                    FbxAnimCurve* lRCurve[3];
                    lRCurve[0] = lCN->GetCurve(0);
                    lRCurve[1] = lCN->GetCurve(1);
                    lRCurve[2] = lCN->GetCurve(2);

                    // Set bone rotation order
                    EFbxRotationOrder RotationOrder = eEulerXYZ;
                    pNode->GetRotationOrder(FbxNode::eSourcePivot, RotationOrder);
                    pUnrollFilter->SetRotationOrder((FbxEuler::EOrder)(RotationOrder));

                    pUnrollFilter->Apply(lRCurve, 3);
                }

                for (int i = 0; i < pNode->GetChildCount(); i++) {
                    ApplyUnroll(pNode->GetChild(i), pLayer, pUnrollFilter);
                }
            };

            ApplyUnroll(sceneInfo->scene->GetRootNode(), lLayer, &unrollFilter);
        }
    }

    FbxAMatrix axisConversionMatrix;
    axisConversionMatrix.SetIdentity();

    if (options->bConvertAxis) {
        FbxAxisSystem importAxis(FbxAxisSystem::eOpenGL);

        FbxAxisSystem sourceSetup = sceneInfo->scene->GetGlobalSettings().GetAxisSystem();

        if (sourceSetup != importAxis) {
            FbxRootNodeUtility::RemoveAllFbxRoots(sceneInfo->scene);
            importAxis.ConvertScene(sceneInfo->scene);

            FbxAMatrix sourceMatrix;
            sourceSetup.GetMatrix(sourceMatrix);
            FbxAMatrix importMatrix;
            importAxis.GetMatrix(importMatrix);
            axisConversionMatrix = importMatrix * sourceMatrix.Inverse();
        }
    }

    FbxDataConverter::SetAxisConversionMatrix(axisConversionMatrix);

    if (options->bConvertUnit && sceneInfo->scene->GetGlobalSettings().GetSystemUnit() != FbxSystemUnit::m) {
        sceneInfo->scaleFactor = sceneInfo->scene->GetGlobalSettings().GetSystemUnit().GetConversionFactorTo(FbxSystemUnit::m);
    }

    // Reset all the transform evaluation cache since we change some node transform
    sceneInfo->scene->GetAnimationEvaluator()->Reset();

    std::shared_ptr<Scene> result = std::make_shared<Scene>();
    std::shared_ptr<Objects::Entity> root = std::make_shared<Objects::Entity>();
    result->root = root;
    std::function<void(FbxNode*, std::shared_ptr<Objects::Entity>)> FillNode = [&](FbxNode* fbxNode, std::shared_ptr<Objects::Entity> node) {
        int nodeCount = fbxNode->GetChildCount();
        for (int nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex) {
            FbxNode* childFbxNode = fbxNode->GetChild(nodeIndex);
            std::shared_ptr<Objects::Entity> childNode = std::make_shared<Objects::Entity>();
            if (sceneInfo->scaleFactor != 1.0) {
                FbxDouble3 lclTranslation = childFbxNode->LclTranslation.Get();
                lclTranslation[0] *= sceneInfo->scaleFactor;
                lclTranslation[1] *= sceneInfo->scaleFactor;
                lclTranslation[2] *= sceneInfo->scaleFactor;
                childFbxNode->LclTranslation.Set(lclTranslation);
            }
            childNode->name = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(childFbxNode->GetName()));
            ;
            childNode->transform = std::make_shared<Objects::Transform>();
            Maths::Transform trans;
            trans.SetFromMatrix(glm::inverse(FbxDataConverter::ConvertMatrix(fbxNode->EvaluateGlobalTransform())) * FbxDataConverter::ConvertMatrix(childFbxNode->EvaluateGlobalTransform()));
            Types::ConvertFromGLM(childNode->transform->transform.translation, trans.translation);
            Types::ConvertFromGLM(childNode->transform->transform.quaternion, trans.rotation);
            Types::ConvertFromGLM(childNode->transform->transform.scale, trans.scale);
            node->children.push_back(childNode);
            result->nodeMap[childFbxNode] = childNode;
            FillNode(childFbxNode, childNode);
        }
    };
    FbxNode* fbxRootNode = sceneInfo->scene->GetRootNode();
    if (sceneInfo->scaleFactor != 1.0) {
        FbxDouble3 lclTranslation = fbxRootNode->LclTranslation.Get();
        lclTranslation[0] *= sceneInfo->scaleFactor;
        lclTranslation[1] *= sceneInfo->scaleFactor;
        lclTranslation[2] *= sceneInfo->scaleFactor;
        fbxRootNode->LclTranslation.Set(lclTranslation);
    }

    root->transform = std::make_shared<Objects::Transform>();
    Maths::Transform trans;
    trans.SetFromMatrix(FbxDataConverter::ConvertMatrix(fbxRootNode->EvaluateGlobalTransform()));
    Types::ConvertFromGLM(root->transform->transform.translation, trans.translation);
    Types::ConvertFromGLM(root->transform->transform.quaternion, trans.rotation);
    Types::ConvertFromGLM(root->transform->transform.scale, trans.scale);

    root->name = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(fbxRootNode->GetName()));
    FillNode(fbxRootNode, root);
    result->nodeMap[fbxRootNode] = root;
    result->sceneInfo = sceneInfo;
    return result;
}

FbxAMatrix FbxDataConverter::AxisConversionMatrix;
FbxAMatrix FbxDataConverter::AxisConversionMatrixInv;

float FbxDataConverter::ConvertDist(FbxDouble distance) {
    float out;
    out = (float)distance;
    return out;
}

glm::vec3 FbxDataConverter::ConvertPos(FbxVector4 vector) {
    glm::vec3 out;
    out.x = (float)(vector[0]);
    out.y = (float)(vector[1]);
    out.z = (float)(vector[2]);
    return out;
}

glm::vec3 FbxDataConverter::ConvertScale(FbxDouble3 vector) {
    glm::vec3 out;
    out.x = (float)(vector[0]);
    out.y = (float)(vector[1]);
    out.z = (float)(vector[2]);
    return out;
}

glm::vec3 FbxDataConverter::ConvertScale(FbxVector4 vector) {
    glm::vec3 out;
    out.x = (float)(vector[0]);
    out.y = (float)(vector[1]);
    out.z = (float)(vector[2]);
    return out;
}

glm::quat FbxDataConverter::ConvertRotToQuat(FbxQuaternion quaternion) {
    glm::quat out;
    out.x = (float)(quaternion[0]);
    out.y = (float)(quaternion[1]);
    out.z = (float)(quaternion[2]);
    out.w = (float)(quaternion[3]);

    return out;
}

glm::mat4 FbxDataConverter::ConvertMatrix(const FbxAMatrix& matrix) {
    glm::mat4 out;

    for (int i = 0; i < 4; ++i) {
        const FbxVector4 Row = matrix.GetRow(i);
        out[i][0] = (float)(Row[0]);
        out[i][1] = (float)(Row[1]);
        out[i][2] = (float)(Row[2]);
        out[i][3] = (float)(Row[3]);
    }

    return out;
}

FbxVector4 FbxDataConverter::ConvertToFbxPos(glm::vec3 vector) {
    FbxVector4 out;
    out[0] = vector[0];
    out[1] = vector[1];
    out[2] = vector[2];

    return out;
}

FbxVector4 FbxDataConverter::ConvertToFbxRot(glm::vec3 vector) {
    FbxVector4 out;
    out[0] = vector[0];
    out[1] = vector[1];
    out[2] = vector[2];

    return out;
}

FbxVector4 FbxDataConverter::ConvertToFbxScale(glm::vec3 vector) {
    FbxVector4 out;
    out[0] = vector[0];
    out[1] = vector[1];
    out[2] = vector[2];

    return out;
}

glm::vec3 FbxDataConverter::ConvertDir(FbxVector4 vector) {
    glm::vec3 out;
    out[0] = (float)(vector[0]);
    out[1] = (float)(vector[1]);
    out[2] = (float)(vector[2]);
    return out;
}

Maths::Transform FbxDataConverter::ConvertTransform(FbxAMatrix matrix) {
    Maths::Transform out;
    out.translation = ConvertPos(matrix.GetT());
    out.scale = ConvertScale(matrix.GetS());
    out.rotation = ConvertRotToQuat(matrix.GetQ());
    return out;
}

}}  // namespace Fbx::Importer