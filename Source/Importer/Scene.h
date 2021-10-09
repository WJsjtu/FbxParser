#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "Mesh.h"
#include "fbxsdk.h"
#undef snprintf
#include "Animation.h"
#include "FbxParser.h"
#include "FbxParser.private.h"

namespace Fbx {

namespace Builder {
class SkinnedMeshSkeleton;
}

namespace Importer {

struct FbxMeshInfo {
    std::string name;
    uint64_t uniqueId;
    int faceNum;
    int vertexNum;
    bool bTriangulated;
    int materialNum;
    bool bIsSkelMesh;
    std::string skeletonRoot;
    int skeletonElemNum;
    std::string LODGroup;
    FbxNode* owner;
    FbxNode* LODParent;
    int LODLevel;
    int morphNum;
};

/**
 * FBX basic data conversion class.
 */
class FbxDataConverter {
public:
    static void SetAxisConversionMatrix(FbxAMatrix ConversionMatrix) {
        AxisConversionMatrix = ConversionMatrix;
        AxisConversionMatrixInv = ConversionMatrix.Inverse();
    }
    static const FbxAMatrix& GetAxisConversionMatrix() { return AxisConversionMatrix; }
    static const FbxAMatrix& GetAxisConversionMatrixInv() { return AxisConversionMatrixInv; }

    static glm::vec3 ConvertPos(FbxVector4 vector);
    static glm::vec3 ConvertDir(FbxVector4 vector);
    static glm::vec3 ConvertScale(FbxDouble3 vector);
    static glm::vec3 ConvertScale(FbxVector4 vector);
    static glm::quat ConvertRotToQuat(FbxQuaternion quaternion);
    static float ConvertDist(FbxDouble distance);
    static glm::mat4 ConvertMatrix(const FbxAMatrix& matrix);

    static FbxVector4 ConvertToFbxPos(glm::vec3 vector);
    static FbxVector4 ConvertToFbxRot(glm::vec3 vector);
    static FbxVector4 ConvertToFbxScale(glm::vec3 vector);

    // FbxCamera with no rotation faces X with Y-up while ours faces X with Z-up so add a -90 degrees roll to compensate
    static glm::vec3 GetCameraRotation() { return glm::vec3(0.f, 0.f, 0.f); }

    // FbxLight with no rotation faces -Z while ours faces Y so add a 90 degrees pitch to compensate
    static glm::vec3 GetLightRotation() { return glm::vec3(0.f, 0.f, 0.f); }

    static Maths::Transform ConvertTransform(FbxAMatrix matrix);

private:
    static FbxAMatrix AxisConversionMatrix;
    static FbxAMatrix AxisConversionMatrixInv;
};

class SceneInfo {
public:
    ~SceneInfo();
    // std::string toString();

    FbxImporter* importer;
    FbxScene* scene;
    bool bIsCreateByBlender = false;
    std::string fileBasePath;
    /**
     * A map holding the original name of the renamed fbx nodes,
     * It is used namely to associate collision meshes to their corresponding static mesh if it has been renamed.
     */
    FbxMap<FbxString, FbxString> nodeUniqueNameToOriginalNameMap;
    /**
     * A map holding pairs of fbx texture that needs to be renamed with the
     * associated string to avoid name conflicts.
     */
    std::map<FbxFileTexture*, std::string> FbxTextureToUniqueNameMap;

    std::string FbxFileVersion;
    std::string FbxFileCreator;
    std::string FbxLastSavedVendor;
    std::string FBXLastSavedAppName;
    std::string FBXLastSavedAppVersion;

    double originalFbxFramerate;
    FbxAxisSystem originalFileAxisSystem;
    FbxSystemUnit originalFileUnitSystem;

    int totalMaterialNum = 0;
    int totalTextureNum = 0;
    int totalGeometryNum = 0;
    int nonSkinnedMeshNum = 0;
    int skinnedMeshNum = 0;

    bool bHasAnimation = false;
    double frameRate = 0;
    double totalTime = 0;

    double scaleFactor = 1.0;

    std::map<FbxNode*, FbxMeshInfo> meshInfo;
    std::map<FbxNode*, std::vector<FbxNode*>> meshLODInfo;
};

class Scene {
public:
    std::shared_ptr<Objects::Entity> root;
    std::map<FbxNode*, std::shared_ptr<Objects::Entity>> nodeMap;
    std::shared_ptr<SceneInfo> sceneInfo;
    std::map<FbxTexture*, std::string> FbxTextureToUniqueNameMap;

    std::map<FbxFileTexture*, std::shared_ptr<Assets::TextureAsset>> textureAssets;
    std::map<FbxSurfaceMaterial*, std::shared_ptr<Assets::MaterialAsset>> materialAssets;
    std::map<FbxNode*, std::shared_ptr<Assets::SkeletonAsset>> skeletonAssets;
    std::map<FbxAnimStack*, std::vector<FbxNode*>> animations;

    void ProcessSkinnedMesh();

    void ProcessMesh();

    void ProcessAnimation();

protected:
    // Mesh

    std::shared_ptr<MeshImportData> ImportMesh(FbxNode* fbxNode, FbxMesh* fbxMesh, bool bForSkinnedMesh);
    FbxNode* GetRootSkeleton(FbxNode* link);
    FbxPose* RetrievePoseFromBindPose(FbxNode* node);
    FbxPose* CreateOrRetrievePoseFromBindPose(FbxNode* node);
    void RecursiveBuildSkeleton(FbxNode* link, std::vector<FbxNode*>& outSortedLinks);
    void BuildSkeletonSystem(std::vector<FbxNode*>& sortedLinks, const std::vector<FbxCluster*>& clusters);
    bool BuildSkeletonBones(const std::vector<FbxNode*>& sortedLinks, const std::vector<FbxCluster*>& clusters, FbxPose* bindPose, std::vector<MeshImportData::Bone>& bones, int& rootIdx);
    bool ImportBones(std::vector<FbxNode*>& sortedLinks, FbxNode* node, FbxMesh* fbxMesh, std::vector<MeshImportData::Bone>& bones);
    void FindOrImportMaterialsFromNode(FbxNode* fbxNode, std::vector<std::shared_ptr<Assets::MaterialAsset>>& outMaterials, std::vector<std::string>& UVSets, bool bForSkinnedMesh);
    std::shared_ptr<Assets::MaterialAsset> CreateMaterial(FbxSurfaceMaterial* fbxMaterial, std::vector<std::string>& outUVSets, bool bForSkinnedMesh);
    bool CreateAndLinkExpressionForMaterialProperty(FbxSurfaceMaterial* fbxMaterial, std::shared_ptr<Assets::MaterialAsset> exportMaterial, const char* materialProperty, Assets::MaterialAsset::Parameter& materialInput, bool bSetupAsNormalMap, std::vector<std::string>& UVSet);
    std::shared_ptr<Assets::TextureAsset> ImportTexture(FbxFileTexture* fbxTexture, bool bSetupAsNormalMap);
    void FixupMaterial(FbxSurfaceMaterial* fbxMaterial, std::shared_ptr<Assets::MaterialAsset> exportMaterial);
    int DoUnSmoothVerts(std::shared_ptr<MeshImportData> importData, bool bDuplicateUnSmoothWedges);

    // Animation
    std::vector<std::shared_ptr<AnimationImportData>> ImportAnimations(FbxNode* rootNode, std::vector<FbxNode*>& sortedLinks, const std::vector<std::string>& fbxRawBoneNames, const std::vector<std::string>& fbxRawBonePaths, Builder::SkinnedMeshSkeleton& skeleton);
    bool IsValidAnimationData(std::vector<FbxNode*>& sortedLinks, FbxNode* node, int& validTakeCount);
    FbxTimeSpan GetAnimationTimeSpan(FbxNode* rootNode, FbxAnimStack* animStack);
    int GetMaxSampleRate(std::vector<FbxNode*>& sortedLinks, FbxNode* rootNode);
    bool ValidateAnimStack(std::vector<FbxNode*>& sortedLinks, FbxNode* rootNode, FbxAnimStack* curAnimStack, int resampleRate, bool bImportMorph, FbxTimeSpan& animTimeSpan);
    void ImportBlendShapeCurves(std::shared_ptr<AnimationImportData> animImportSettings, FbxAnimStack* curAnimStack, int& outKeyCount);
    void ImportBoneTracks(Builder::SkinnedMeshSkeleton skeleton, std::shared_ptr<AnimationImportData> animImportSettings, FbxNode* skeletalMeshRootNode, FbxAnimStack* animStack, const int resampleRate, int& outTotalNumKeys);
};

class ImporterHelper {
public:
    static std::string MakeName(const char* name);
    static std::string MakeName(const std::string& str);
    static std::string NativeToUTF8(const std::string& str);
    static std::string UTF8ToNative(const std::string& str);
    static FbxNode* RecursiveFindParentLodGroup(FbxNode* parentNode);
    static FbxNode* RecursiveGetFirstMeshNode(FbxNode* node, FbxNode* nodeToFind);
    static FbxNode* FindLODGroupNode(FbxNode* nodeLodGroup, int LodIndex, FbxNode* nodeToFind = nullptr);
    static void GetNodeSampleRate(FbxNode* node, FbxAnimLayer* animLayer, std::vector<int>& nodeAnimSampleRates, bool bCurve, bool bBlendCurve);
    static int GetAnimationCurveRate(FbxAnimCurve* currentCurve);
    static int GetTimeSampleRate(const float deltaTime);
    static bool IsOddNegativeScale(FbxAMatrix& matrix);

    static bool ShouldImportCurve(FbxAnimCurve* curve, bool bDoNotImportWithZeroValues);
    static bool CompressRawAnimData(std::vector<FRawAnimSequenceTrack>& rawAnimationData, int numFrames, std::string errorName, float maxPosDiff, float maxAngleDiff);
};
class Importer {
public:
    ~Importer();
    static std::shared_ptr<Importer> GetInstance();

    std::shared_ptr<SceneInfo> GetFileSceneInfo(const std::string& filename, std::shared_ptr<Options> options);
    std::shared_ptr<Scene> ImportScene(std::shared_ptr<SceneInfo> sceneInfo, std::shared_ptr<Options> options);
    std::shared_ptr<Options> options;

public:
    FbxManager* sdkManager;
    std::shared_ptr<FbxGeometryConverter> geometryConverter;

protected:
    Importer();

    static std::shared_ptr<Importer> StaticInstance;
    std::shared_ptr<Scene> ConvertScene(std::shared_ptr<SceneInfo> sceneInfo);
};

}  // namespace Importer
}  // namespace Fbx
