#include "Resource.h"
#include <iostream>
#include <string>
#include <locale>
#include <codecvt>
#include <exception>

namespace Exporter {

#ifdef _MSC_VER
#include <Windows.h>

std::wstring GetStringValueFromHKLM(const std::wstring& regSubKey, const std::wstring& regValue) {
    size_t bufferSize = 0xFFF;  // If too small, will be resized down below.
    std::wstring valueBuf;      // Contiguous buffer since C++11.
    valueBuf.resize(bufferSize);
    auto cbData = static_cast<DWORD>(bufferSize * sizeof(wchar_t));
    auto rc = RegGetValueW(HKEY_LOCAL_MACHINE, regSubKey.c_str(), regValue.c_str(), RRF_RT_REG_SZ, nullptr, static_cast<void*>(valueBuf.data()), &cbData);
    while (rc == ERROR_MORE_DATA) {
        // Get a buffer that is big enough.
        cbData /= sizeof(wchar_t);
        if (cbData > static_cast<DWORD>(bufferSize)) {
            bufferSize = static_cast<size_t>(cbData);
        } else {
            bufferSize *= 2;
            cbData = static_cast<DWORD>(bufferSize * sizeof(wchar_t));
        }
        valueBuf.resize(bufferSize);
        rc = RegGetValueW(HKEY_LOCAL_MACHINE, regSubKey.c_str(), regValue.c_str(), RRF_RT_REG_SZ, nullptr, static_cast<void*>(valueBuf.data()), &cbData);
    }
    if (rc == ERROR_SUCCESS) {
        cbData /= sizeof(wchar_t);
        valueBuf.resize(static_cast<size_t>(cbData - 1));  // remove end null character
        return valueBuf;
    } else {
        throw std::runtime_error("Windows system error code: " + std::to_string(rc));
    }
}

std::string ANSItoUTF8(const std::string& strAnsi) {
    std::string result = strAnsi;
    UINT nLen = MultiByteToWideChar(CP_ACP, NULL, result.data(), -1, NULL, NULL);
    WCHAR* wszBuffer = new WCHAR[nLen + 1];
    nLen = MultiByteToWideChar(CP_ACP, NULL, result.data(), -1, wszBuffer, nLen);
    wszBuffer[nLen] = 0;
    nLen = WideCharToMultiByte(CP_UTF8, NULL, wszBuffer, -1, NULL, NULL, NULL, NULL);
    CHAR* szBuffer = new CHAR[nLen + 1];
    nLen = WideCharToMultiByte(CP_UTF8, NULL, wszBuffer, -1, szBuffer, nLen, NULL, NULL);
    szBuffer[nLen] = 0;
    result = std::string(szBuffer);
    delete[] wszBuffer;
    delete[] szBuffer;
    return result;
}
#endif

std::string ExportDirectory;
void SetExportDirectory(const std::string& path) { ExportDirectory = path; }

std::string GetExportDirectory() { return ExportDirectory; }

bool ExistPath(const std::string& path) {
    try {
        return std::filesystem::exists(path);
    } catch (std::filesystem::filesystem_error error) {
        std::cerr << "[std::filesystem::filesystem_error]: " << error.what() << std::endl;
        return false;
    } catch (...) {
        return false;
    }
}

bool EnsurePath(const std::string& path) {
    std::string dir_path = std::filesystem::path(path).parent_path().generic_string();
    try {
        if (!ExistPath(dir_path) || !std::filesystem::is_directory(dir_path)) {
            return std::filesystem::create_directories(dir_path);
        }
        return true;
    } catch (std::filesystem::filesystem_error error) {
        std::cerr << "[std::filesystem::filesystem_error]: " << error.what() << std::endl;
        return false;
    } catch (...) {
        return false;
    }
}

std::string CleanIllegalChar(const std::string& str, bool heightLevel) {
    std::string result = str;
    for (int i = 0; i < result.length(); i++) {
        if (result.at(i) == '<' || result.at(i) == '>' || result.at(i) == '|' || result.at(i) == '\"' || result.at(i) == '?' || result.at(i) == '*' || result.at(i) == '#') {
            result.at(i) = '_';
        }
    }
    if (heightLevel) {
        for (int i = 0; i < result.length(); i++) {
            if (result.at(i) == '/' || result.at(i) == ':') {
                result.at(i) = '_';
            }
        }
    }
    return result;
}

bool ExportStore::IsFileConverted(const std::string& relativePath, const std::string& version) {
    if ((files.find(relativePath) != files.end()) && files[relativePath].MD5 == version) {
        return true;
    }
    return false;
}

bool ExportStore::IsFilePathValid(const std::string& relativePath) { return files.find(relativePath) == files.end(); }

void ExportStore::AddBinaryFile(const std::string& relativePath, const std::vector<char>& content, const std::string& assetMD5) {
    auto savePath = std::filesystem::path(GetExportDirectory()) / CleanIllegalChar(relativePath, true);
    if (EnsurePath(savePath.generic_string())) {
        std::ofstream binaryFile(savePath.generic_string(), std::ios::out | std::ios::binary);
        if (binaryFile) {
            binaryFile.write(content.data(), content.size());
            binaryFile.close();
            SaveFileManifest(relativePath, "arraybuffer", assetMD5);
        }
    }
}

void ExportStore::AddImageFile(const std::string& relativePath, const std::vector<char>& imageData, const std::string& assetMD5) {
    auto savePath = std::filesystem::path(GetExportDirectory()) / CleanIllegalChar(relativePath, true);
    if (EnsurePath(savePath.generic_string())) {
        std::ofstream imageFile(savePath.generic_string(), std::ios::out | std::ios::binary);
        if (imageFile) {
            imageFile.write(imageData.data(), imageData.size());
            imageFile.close();
            SaveFileManifest(relativePath, "image", assetMD5);
        }
    }
}

void ExportStore::AddJSONFile(const std::string& relativePath, const std::string& content, const std::string& assetMD5) {
    auto savePath = std::filesystem::path(GetExportDirectory()) / CleanIllegalChar(relativePath, true);
    if (EnsurePath(savePath.generic_string())) {
        std::ofstream jsonFile(savePath.generic_string(), std::ios::out);
        jsonFile.imbue(std::locale("en_US.utf8"));
#ifdef _MSC_VER
        std::string encoded = ANSItoUTF8(content);
#else
        std::string encoded = content;
#endif
        if (jsonFile) {
            jsonFile.write(encoded.data(), encoded.size());
            jsonFile.close();
            SaveFileManifest(relativePath, "json", assetMD5);
        }
    }
}

void ExportStore::AddTextFile(const std::string& relativePath, const std::string& content, const std::string& assetMD5) {
    auto savePath = std::filesystem::path(GetExportDirectory()) / CleanIllegalChar(relativePath, true);
    if (EnsurePath(savePath.generic_string())) {
        std::ofstream textFile(savePath.generic_string(), std::ios::out);
        textFile.imbue(std::locale("en_US.utf8"));
#ifdef _MSC_VER
        std::string encoded = ANSItoUTF8(content);
#else
        std::string encoded = content;
#endif
        if (textFile) {
            textFile.write(encoded.data(), encoded.size());
            textFile.close();
            SaveFileManifest(relativePath, "text", assetMD5);
        }
    }
}

void ExportStore::AddResource(const std::string& descriptionFilePath, const std::string& resourceType, const std::vector<std::string>& dependencies, const std::vector<std::string>& useFile, std::string importSetting) {
    AssetInfo info;
    info.descriptionPath = descriptionFilePath;
    info.type = resourceType;
    info.dependencies = dependencies;
    info.useFile = useFile;
    info.importSetting = importSetting;
    assets[descriptionFilePath] = std::move(info);
}

std::map<std::string, ExportStore::FileInfo> ExportStore::files;
std::map<std::string, ExportStore::AssetInfo> ExportStore::assets;

void ExportStore::SaveFileManifest(const std::string& relativePath, const std::string& fileType, const std::string& assetVersion) { files[relativePath] = {assetVersion, fileType, relativePath}; }

void ExportStore::SaveStorage(const std::string& entryRelativePath) {
    struct ResourceDescription {
        std::vector<std::string> dependencies;
        std::string type;
        std::string descriptionFileID;
        std::string importSetting;
        std::vector<std::string> useFile;
    };
    struct FileDescription {
        std::string path;
    };
    struct FileItem {
        std::string path;
        std::string filetype;
    };
    std::map<std::string, std::shared_ptr<ResourceDescription>> resourceDefinitions;
    std::map<std::string, std::shared_ptr<FileDescription>> fileDescriptions;
    std::vector<std::shared_ptr<FileItem>> fileItems;
    std::function<void(const std::string& resourceDescriptionFilePath)> WriteResourceRecursive = [&](const std::string& resourceDescriptionFilePath) {
        if (resourceDefinitions.find(resourceDescriptionFilePath) != resourceDefinitions.end()) {
            return;
        }
        if (assets.find(resourceDescriptionFilePath) == assets.end()) {
            return;
        }

        auto rDesc = std::make_shared<ResourceDescription>();
        resourceDefinitions.emplace(resourceDescriptionFilePath, rDesc);
        rDesc->dependencies = assets[resourceDescriptionFilePath].dependencies;
        rDesc->type = assets[resourceDescriptionFilePath].type;
        rDesc->descriptionFileID = resourceDescriptionFilePath;
        if (assets[resourceDescriptionFilePath].importSetting != "") {
            rDesc->importSetting = assets[resourceDescriptionFilePath].importSetting;
        }
        for (auto usingFile : assets[resourceDescriptionFilePath].useFile) {
            rDesc->useFile.push_back(usingFile);
            if (fileDescriptions.find(usingFile) != fileDescriptions.end()) {
                continue;
            }
            if (files.find(usingFile) == files.end()) {
                continue;
            }

            auto fileInfo = files[usingFile];

            auto fDesc = std::make_shared<FileDescription>();
            fileDescriptions.emplace(usingFile, fDesc);
            fDesc->path = usingFile;

            auto fItem = std::make_shared<FileItem>();
            fItem->path = usingFile;
            fItem->filetype = fileInfo.filetype;
            fileItems.push_back(fItem);
        }

        for (auto dep : assets[resourceDescriptionFilePath].dependencies) {
            WriteResourceRecursive(dep);
        }
    };
    WriteResourceRecursive(entryRelativePath);
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("resourceDefinitions");
    writer.StartObject();
    for (auto kvp : resourceDefinitions) {
        writer.String(kvp.first.c_str());
        writer.StartObject();
        writer.String("dependencies");
        writer.StartArray();
        for (auto dep : kvp.second->dependencies) {
            JSON_STRING_VALUE(writer, dep);
        }
        writer.EndArray();
        writer.String("type");
        JSON_STRING_VALUE(writer, kvp.second->type);
        writer.String("descriptionFileID");
        JSON_STRING_VALUE(writer, kvp.second->descriptionFileID);
        if (kvp.second->importSetting != "") {
            writer.String("importSetting");
            JSON_RAW_VALUE(writer, kvp.second->importSetting, rapidjson::Type::kObjectType);
        }
        writer.String("useFile");
        writer.StartArray();
        for (auto file : kvp.second->useFile) {
            JSON_STRING_VALUE(writer, file);
        }
        writer.EndArray();
        writer.EndObject();
    }
    writer.EndObject();

    writer.String("fileDescriptions");
    writer.StartObject();
    for (auto kvp : fileDescriptions) {
        writer.String(kvp.first.c_str());
        writer.StartObject();
        writer.String("path");
        JSON_STRING_VALUE(writer, kvp.second->path);
        writer.EndObject();
    }
    writer.EndObject();

    writer.String("files");
    writer.StartArray();
    for (auto& file : fileItems) {
        writer.StartObject();
        writer.String("path");
        JSON_STRING_VALUE(writer, file->path);
        writer.String("filetype");
        JSON_STRING_VALUE(writer, file->filetype);
        writer.EndObject();
    }
    writer.EndArray();

    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif

    auto savePath = std::filesystem::path(GetExportDirectory()) / "group.manifest.json";
    if (EnsurePath(savePath.generic_string())) {
        std::ofstream manifistFile(savePath.generic_string(), std::ios::out);
        manifistFile.imbue(std::locale("en_US.utf8"));
#ifdef _MSC_VER
        std::string encoded = ANSItoUTF8(json);
#else
        std::string encoded = json;
#endif
        if (manifistFile) {
            manifistFile.write(encoded.data(), encoded.size());
            manifistFile.close();
        }
    }
}

std::string Exportable::Export() {
    if (!ExportStore::IsFileConverted(GetExportPath(), DoGetHash())) {
        LOG(Fbx::Utils::ELogLevel::Info, "Exporting file: " + GetExportPath());
        if (DoExport()) {
            return GetExportPath();
        } else {
            return "";
        }
    }
    return GetExportPath();
}

AssetFile::AssetFile(const std::string& assetPath) : Exportable() { rawAssetPath = assetPath; }

std::string AssetFile::GetHash() { return GetExportPath(); }

MeshBinAssetFile::MeshBinAssetFile(const std::string& assetPath, const std::string& inName, const std::vector<char>& inContent) : AssetFile(assetPath), name(inName), content(inContent) {}

bool MeshBinAssetFile::DoExport() {
    ExportStore::AddBinaryFile(GetExportPath(), GetContent(), GetHash());
    return true;
}

std::string MeshBinAssetFile::GetExportPath() {
    std::string savePath = name + ".mesh.bin";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = name + "_" + std::to_string(++renameCount) + ".mesh.bin";
        }
    }
    return savePath;
}

const std::vector<char>& MeshBinAssetFile::GetContent() { return content; }

ImageAssetFile::ImageAssetFile(const std::string& assetPath, const std::string& inName, const std::string& inType, const std::vector<char>& inContent) : AssetFile(assetPath), name(inName), type(inType), content(inContent) {}

bool ImageAssetFile::DoExport() {
    ExportStore::AddBinaryFile(GetExportPath(), GetContent(), GetHash());
    return true;
}

std::string ImageAssetFile::GetExportPath() {
    std::string savePath = name + type;
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = name + "_" + std::to_string(++renameCount) + type;
        }
    }
    return savePath;
}

const std::vector<char>& ImageAssetFile::GetContent() { return content; }

std::map<std::string, Resource::ConverterFactory> Resource::ResourcesConverters;
std::shared_ptr<Resource> Resource::GetConverter(std::string type, std::shared_ptr<Fbx::Objects::Entity> go) {
    if (ResourcesConverters.find(type) == ResourcesConverters.end()) {
        return nullptr;
    }
    return ResourcesConverters[type](go);
}
Resource::Resource(const std::string& inRawAssetPath) { rawAssetPath = inRawAssetPath; }

std::string Resource::AddDependencies(std::shared_ptr<Resource> resource) {
    std::string convertedPath = resource->Export();
    if (convertedPath != "") {
        dependencies.push_back(convertedPath);
    }
    return convertedPath;
}

void Resource::AddDependencies(const std::string& convertedPath) { dependencies.push_back(convertedPath); }

std::string Resource::AddFile(std::shared_ptr<AssetFile> file) {
    std::string filePath = file->Export();
    useFile.push_back(filePath);
    return filePath;
}

bool Resource::DoExport() {
    std::string json = ExportResource();
    if (json == "" && !dontExportDescriptionJSON) {
        return false;
    }
    std::string exportPath = GetExportPath();
    useFile.push_back(exportPath);
    if (!dontExportDescriptionJSON) {
        ExportStore::AddJSONFile(exportPath, json, DoGetHash());
    }
    ExportStore::AddResource(exportPath, GetResourceType(), dependencies, useFile, importSetting);
    return true;
}

}  // namespace Exporter