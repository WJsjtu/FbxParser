#pragma once
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <map>
#include <string>
#include "FbxParser.h"
#undef snprintf
#include <assert.h>
#include "rapidjson/pointer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/schema.h"
#include <filesystem>

#define ASSERT assert

#if RAPIDJSON_HAS_STDSTRING
#define JSON_STRING_VALUE(writer, string) writer.String(string.c_str())
#define JSON_RAW_VALUE(writer, string, type) writer.RawValue(string.c_str())
#else
#define JSON_STRING_VALUE(writer, string)                                           \
    {                                                                               \
        auto str = string;                                                          \
        writer.String(str.c_str(), static_cast<rapidjson::SizeType>(str.length())); \
    }
#define JSON_RAW_VALUE(writer, string, type)                                                \
    {                                                                                       \
        auto str = string;                                                                  \
        writer.RawValue(str.c_str(), static_cast<rapidjson::SizeType>(str.length()), type); \
    };
#endif

#define PRETTIFY_JSON(json)                                              \
    {                                                                    \
        rapidjson::Document document;                                    \
        document.Parse(json.c_str());                                    \
        rapidjson::StringBuffer buffer;                                  \
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer); \
        document.Accept(writer);                                         \
        json = buffer.GetString();                                       \
    }

#ifdef _MSC_VER
#define LOG(level, string) Fbx::Utils::GetGlobalLogger()(level, Exporter::ANSItoUTF8(string));
#else
#define LOG(level, string) Fbx::Utils::GetGlobalLogger()(level, string);
#endif

namespace Exporter {
#ifdef _MSC_VER
std::wstring GetStringValueFromHKLM(const std::wstring& regSubKey, const std::wstring& regValue);
std::string ANSItoUTF8(const std::string& strAnsi);
#endif

bool EnsurePath(const std::string& path);

template <typename Range, typename Value = typename Range::value_type>
std::string Join(Range const& elements, const char* const delimiter) {
    std::ostringstream os;
    auto b = begin(elements), e = end(elements);

    if (b != e) {
        std::copy(b, prev(e), std::ostream_iterator<Value>(os, delimiter));
        b = prev(e);
    }
    if (b != e) {
        os << *b;
    }

    return os.str();
}

class Context;

void SetExportDirectory(const std::string& path);

std::string GetExportDirectory();

class ExportStore {
public:
    static bool IsFileConverted(const std::string& relativePath, const std::string& version);
    static bool IsFilePathValid(const std::string& relativePath);
    static void AddBinaryFile(const std::string& relativePath, const std::vector<char>& content, const std::string& assetMD5);
    static void AddImageFile(const std::string& relativePath, const std::vector<char>& imageData, const std::string& assetMD5);
    static void AddJSONFile(const std::string& relativePath, const std::string& content, const std::string& assetMD5);
    static void AddTextFile(const std::string& relativePath, const std::string& content, const std::string& assetMD5);
    static void AddResource(const std::string& descriptionFilePath, const std::string& resourceType, const std::vector<std::string>& dependencies, const std::vector<std::string>& useFile, std::string importSetting = "");
    static void SaveStorage(const std::string& entryRelativePath);

private:
    struct FileInfo {
        std::string MD5;
        std::string filetype;
        std::string path;
    };
    struct AssetInfo {
        std::string descriptionPath;
        std::string type;
        std::vector<std::string> dependencies;
        std::vector<std::string> useFile;
        std::string importSetting;
    };
    static std::map<std::string, FileInfo> files;
    static std::map<std::string, AssetInfo> assets;
    static void SaveFileManifest(const std::string& relativePath, const std::string& fileType, const std::string& assetVersion);
};

class Exportable {
public:
    virtual std::string GetHash() = 0;
    virtual std::string DoGetHash() { return GetHash(); }
    virtual std::string GetExportPath() = 0;
    virtual bool DoExport() = 0;
    std::string Export();
};

class AssetFile : public Exportable {
public:
    std::string rawAssetPath;

    AssetFile(const std::string& assetPath);
    virtual std::string GetHash() override;
};

class MeshBinAssetFile : public AssetFile {
public:
    MeshBinAssetFile(const std::string& assetPath, const std::string& inName, const std::vector<char>& inContent);

    virtual bool DoExport() override;
    virtual std::string GetExportPath() override;
    const std::vector<char>& GetContent();
    std::vector<char> content;
    std::string name;

    bool isPathLocked = false;
};

class ImageAssetFile : public AssetFile {
public:
    ImageAssetFile(const std::string& assetPath, const std::string& inName, const std::string& inType, const std::vector<char>& inContent);

    virtual bool DoExport() override;
    virtual std::string GetExportPath() override;
    const std::vector<char>& GetContent();
    std::vector<char> content;
    std::string name;
    std::string type;

    bool isPathLocked = false;
};

class Resource : public Exportable {
public:
    typedef std::function<std::shared_ptr<Resource>(std::shared_ptr<Fbx::Objects::Entity>)> ConverterFactory;
    static std::map<std::string, ConverterFactory> ResourcesConverters;
    static std::shared_ptr<Resource> GetConverter(std::string type, std::shared_ptr<Fbx::Objects::Entity> go);
    Resource(const std::string& inRawAssetPath);
    std::string AddDependencies(std::shared_ptr<Resource> resource);
    void AddDependencies(const std::string& convertedPath);
    std::string AddFile(std::shared_ptr<AssetFile> file);
    virtual std::string ExportResource() = 0;
    virtual std::string GetResourceType() = 0;

protected:
    std::string rawAssetPath;
    std::vector<std::string> dependencies;
    std::vector<std::string> useFile;
    std::string importSetting = "";
    bool dontExportDescriptionJSON = false;
    virtual bool DoExport();
};

class Avatar : public Resource {
public:
    Avatar(std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton);
    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;

public:
    std::shared_ptr<Fbx::Assets::SkeletonAsset> skeleton;
};

class Texture : public Resource {
public:
    Texture(std::shared_ptr<Fbx::Assets::TextureAsset> inTexture);
    std::shared_ptr<Fbx::Assets::TextureAsset> texture = nullptr;
    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

class Material : public Resource {
public:
    Material(std::shared_ptr<Fbx::Assets::MaterialAsset> inMaterial);

    std::shared_ptr<Fbx::Assets::MaterialAsset> material = nullptr;
    bool bEnableShadowCasting = true;

    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

class Mesh : public Resource {
public:
    Mesh(std::shared_ptr<Fbx::Assets::MeshAsset> inMesh);

    std::shared_ptr<Fbx::Assets::MeshAsset> mesh = nullptr;

    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

class SkinnedMesh : public Resource {
public:
    SkinnedMesh(std::shared_ptr<Fbx::Assets::MeshAsset> inSkinnedMesh, std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton);

    std::shared_ptr<Fbx::Assets::MeshAsset> mesh = nullptr;
    std::shared_ptr<Fbx::Assets::SkeletonAsset> skeleton = nullptr;

    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

class AnimationClip : public Resource {
public:
    AnimationClip(std::shared_ptr<Fbx::Assets::AnimationClipAsset> inAnimationClip);

    std::shared_ptr<Fbx::Assets::AnimationClipAsset> animationClip = nullptr;

    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

class Prefab : public Resource {
public:
    Prefab(std::shared_ptr<Fbx::Objects::Entity> inRoot, const std::string& inName);
    std::shared_ptr<Fbx::Objects::Entity> root;
    std::string name;
    std::shared_ptr<Context> context;

    virtual std::string GetHash() override;
    virtual std::string GetExportPath() override;
    virtual std::string ExportResource() override;
    virtual std::string GetResourceType() override;

    bool isPathLocked = false;
};

}  // namespace Exporter
