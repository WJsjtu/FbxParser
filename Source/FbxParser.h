#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <iostream>

#pragma warning(disable : 4251)
#pragma warning(disable : 4275)

#ifdef _WIN32
#ifdef FBX_DLL
#define FBX_PORT __declspec(dllexport)
#else
#define FBX_PORT __declspec(dllimport)
#endif
#else
#if __has_attribute(visibility)
#define FBX_PORT __attribute__((visibility("default")))
#else
#define FBX_PORT
#endif
#endif

namespace Fbx {

namespace Configuration {
FBX_PORT extern uint32_t MaxTexCoord;
FBX_PORT extern uint32_t MaxFrameRate;
FBX_PORT extern uint32_t DefaultFrameRate;
FBX_PORT extern float VectorComparsionThreshold;
FBX_PORT extern float PointComparsionThreshold;
FBX_PORT extern float UVComparsionThreshold;
}  // namespace Configuration

class FBX_PORT Options {
public:
    enum class FBX_PORT NormalOptions {
        ImportOrComputeNormal = 0,
        ForceComputeNormal,
        IgnoreNormal,
    };

    enum class FBX_PORT TangentOptions {
        ImportOrComputeTangent = 0,
        ForceComputeTangent,
        IgnoreTangent,
    };

    enum class FBX_PORT EFBXAnimationLengthType {
        /** This option imports animation frames based on what is defined at the time of export */
        FBXALIT_ExportedTime,
        /** Will import the range of frames that have animation. Can be useful if the exported range is longer than the actual animation in the FBX file */
        FBXALIT_AnimatedKey,
    };

    bool bConvertAxis = true;
    bool bConvertUnit = true;
    bool bParseAnimations = true;
    bool bResample = true;

    /** If checked, triangles with non-matching smoothing groups will be physically split. */
    bool bPreserveSmoothingGroups = true;

    /** Threshold to compare vertex position equality. */
    float thresholdPosition = Configuration::PointComparsionThreshold;

    /** Threshold to compare normal, tangent or bi-normal equality. */
    float thresholdTangentNormal = Configuration::VectorComparsionThreshold;

    /** Threshold to compare UV equality. */
    float thresholdUV = Configuration::UVComparsionThreshold;

    /** Threshold to compare vertex position equality when computing morph target deltas. */
    float morphThresholdPosition = Configuration::PointComparsionThreshold;

    uint32_t maxBones = 96;

    NormalOptions computeNormal = NormalOptions::ForceComputeNormal;

    TangentOptions computeTangent = TangentOptions::ForceComputeTangent;

    EFBXAnimationLengthType animationLengthParseType = EFBXAnimationLengthType::FBXALIT_ExportedTime;

    int resampleRate = Configuration::DefaultFrameRate;

    bool bParseMorphTargets = true;

    bool bDoNotParseCurveWithZero = true;

    bool bParseCustomAttribute = true;

    bool bParseBoneTracks = true;

    bool bPreserveLocalTransform = false;

    std::string embeddingExtractionFolder = "";
};

struct FBX_PORT FbxInfo {
    std::string fileVersion;
    std::string fileCreator;
    std::string lastSavedVendor;
    std::string lastSavedAppName;
    std::string lastSavedAppVersion;
    uint32_t materialCount;
    uint32_t textureCount;
    uint32_t geometryCount;
    uint32_t nonSkinnedMeshCount;
    uint32_t skinnedMeshCount;
    bool bHasAnimation;
    float framerate;
    float time;
    std::string unitSystem;
};

namespace Types {

struct FBX_PORT Vector2 {
    Vector2() = default;
    Vector2(float x, float y);
    float x;
    float y;
};

struct FBX_PORT Vector3 {
    Vector3() = default;
    Vector3(float x, float y, float z);
    float x;
    float y;
    float z;
};

struct FBX_PORT Vector4 {
    Vector4() = default;
    Vector4(float x, float y, float z, float w);
    float x;
    float y;
    float z;
    float w;
};

struct FBX_PORT Matrix4 {
    float data[16];
};

struct FBX_PORT Transform {
    Vector3 translation;
    Vector3 scale;
    Vector4 quaternion;
};

}  // namespace Types

namespace Assets {

class FBX_PORT Asset {
public:
    std::string name;
    std::string uniqueID;
};

class FBX_PORT AnimationClipAsset : public Asset {
public:
    enum class FBX_PORT EAnimationCurveType {
        PositionX = 1,
        PositionY = 2,
        PositionZ = 3,
        ScaleX = 4,
        ScaleY = 5,
        ScaleZ = 6,
        RotationX = 7,
        RotationY = 8,
        RotationZ = 9,
        RotationW = 10,
    };
    struct FBX_PORT CurveInfo {
        int index;
        EAnimationCurveType type;
        int frameCount;
    };
    struct FBX_PORT FrameInfo {
        uint32_t frameIndex;
        float value;
        float inTangent;
        float outTangent;
    };

public:
    float length;
    int frameRate;
    int frameCount;
    std::vector<std::shared_ptr<CurveInfo>> curves;
    std::vector<std::shared_ptr<FrameInfo>> frames;
    std::vector<std::string> paths;
};

class FBX_PORT TextureAsset : public Asset {
public:
    enum class FBX_PORT EWrapMode { Repeat, Clamp };
    enum class FBX_PORT EPixelFormat { RGBA8, RGBA16F };

public:
    uint32_t width = 0;
    uint32_t height = 0;
    EWrapMode wrapU = EWrapMode::Repeat;
    EWrapMode wrapV = EWrapMode::Repeat;
    EPixelFormat pixelFormat = EPixelFormat::RGBA8;
    std::vector<char> raw;
    std::vector<uint8_t> decoded;
    bool bNeedPremultiplyAlpha = false;
    bool bUseMipMap = false;
    bool bEnableCompress = true;
    uint32_t numMips = 0;
    std::string fileType;
    std::string originalPath;
    bool bIsCommonPNGorJPGFile = false;
};

class FBX_PORT MaterialAsset : public Asset {
public:
    enum class FBX_PORT ETextureSamplerType { Color, Mormal };
    enum class FBX_PORT EBlendMode { Opaque, Masked, Translucent, Additive, Modulate, AlphaComposite, AlphaHoldout };
    enum class FBX_PORT EShaderType { Default, Phong };
    struct FBX_PORT TextureInfo {
        std::shared_ptr<TextureAsset> texture;
        ETextureSamplerType samplerType = ETextureSamplerType::Color;
        Types::Vector4 tilling = {1.0, 1.0, 0.0, 0.0};
    };
    struct FBX_PORT ColorInfo {
        Types::Vector4 value = {0.0, 0.0, 0.0, 1.0};
    };

    struct FBX_PORT Parameter {
        ETextureSamplerType type;
        std::shared_ptr<TextureInfo> texture;
        std::shared_ptr<ColorInfo> color;
    };

public:
    EBlendMode blendMode = EBlendMode::Opaque;
    EShaderType shaderType = EShaderType::Phong;
    Parameter baseColor;
    Parameter emissive;
    Parameter specular;
    Parameter roughness;
    Parameter metallic;
    Parameter normal;
    Parameter opacity;
    Parameter opacityMask;
    bool bForSkinnedMesh;
};

class FBX_PORT SkeletonAsset : public Asset {
public:
    struct FBX_PORT Bone {
        std::string uniqueID;
        std::string name;
        std::string path;
        Types::Transform transform;
        std::vector<std::shared_ptr<Bone>> children;
        std::shared_ptr<Bone> parent = nullptr;
    };

public:
    std::shared_ptr<Bone> root = nullptr;
    std::shared_ptr<Assets::SkeletonAsset::Bone> rootBone = nullptr;
    std::vector<std::shared_ptr<Bone>> bones;
    std::vector<std::string> names;
    std::vector<std::string> paths;
    bool bForSkinnedMesh;
};

enum class FBX_PORT EMeshIndiceFormat {
    Bit16 = 1,
    Bit32 = 2,
};

class FBX_PORT MeshAsset : public Asset {
public:
    struct FBX_PORT Vertex {
        Types::Vector3 position;
        Types::Vector4 color;
        std::vector<Types::Vector2> uv;
        Types::Vector4 tangent;
        Types::Vector3 normal;
        std::vector<float> boneWeight;
        std::vector<uint32_t> boneIndex;
    };

    struct FBX_PORT SubMesh {
        uint32_t baseIndex;
        std::shared_ptr<MaterialAsset> material;
        std::vector<std::shared_ptr<Vertex>> vertices;
        uint32_t numTriangles;
    };

    struct FBX_PORT BoundingSphere {
        Types::Vector3 center;
        float radius;
    };

    struct FBX_PORT BoundingBox {
        Types::Vector3 center;
        Types::Vector3 extents;
    };

public:
    std::vector<uint32_t> indexBuffer;
    std::vector<std::shared_ptr<SubMesh>> subMeshes;
    EMeshIndiceFormat indiceFormat;
    bool bHasNormals;
    bool bHasVertexColors;
    bool bHasTangents;
    uint32_t numTexCoords;
    std::string rootBonePath;
    std::vector<std::string> bonePaths;
    std::vector<uint16_t> activeBoneIndices;
    std::vector<Types::Matrix4> boneInverseMatrices;
    std::shared_ptr<BoundingSphere> boundingSphere;
    std::shared_ptr<BoundingBox> boundingBox;
    bool bForSkinnedMesh;
};
}  // namespace Assets

namespace Objects {

enum class FBX_PORT EComponentType { Component, Transform, Skeleton, MeshRenderer, Animation };

class FBX_PORT Component {
public:
    Component();
    std::string id;
    virtual EComponentType GetTypeName();
};

class FBX_PORT Transform : public Component {
public:
    Types::Transform transform;
    virtual EComponentType GetTypeName() override;
};

class FBX_PORT Skeleton : public Component {
public:
    std::shared_ptr<Assets::SkeletonAsset> skeleton;
    virtual EComponentType GetTypeName() override;
};

class FBX_PORT MeshRenderer : public Component {
public:
    std::shared_ptr<Skeleton> skeleton;
    std::shared_ptr<Assets::MeshAsset> mesh;
    virtual EComponentType GetTypeName() override;
};

class FBX_PORT Animation : public Component {
public:
    std::shared_ptr<Skeleton> skeleton;
    std::unordered_map<std::string, std::shared_ptr<Assets::AnimationClipAsset>> animationClips;
    virtual EComponentType GetTypeName() override;
};

class FBX_PORT Entity {
public:
    std::string name;
    std::shared_ptr<Entity> parent;
    std::vector<std::shared_ptr<Entity>> children;
    std::vector<std::shared_ptr<Component>> components;
    std::shared_ptr<Transform> transform;
};

}  // namespace Objects

namespace Utils {
enum class FBX_PORT ELogLevel {
    Info,
    Error,
    Warn,
    Critical,
    Debug,
};

std::function<void(ELogLevel, std::string)> FBX_PORT GetDefaultLogger(const std::string& logFilePath);

std::function<void(ELogLevel, std::string)> FBX_PORT GetGlobalLogger();

void FBX_PORT SetGlobalLogger(std::function<void(ELogLevel, std::string)> logger);

}  // namespace Utils

class FBX_PORT AssertException : public std::exception {
public:
    AssertException(const std::string& msg);

    virtual const char* what() const throw();

    const std::string message;
};

std::pair<std::shared_ptr<FbxInfo>, std::shared_ptr<Objects::Entity>> FBX_PORT ParseFbx(const std::string& filePath, std::shared_ptr<Options> options);

}  // namespace Fbx
