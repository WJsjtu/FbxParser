#include <iostream>
#include "Exporter/Engine.h"
#include "Exporter/Resource.h"
#include "ImportOptions.hpp"
#include "cxxopts.hpp"
#include "rapidjson/pointer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/schema.h"
#include "FbxParser.h"
#include <string>

std::string toString(std::shared_ptr<Fbx::FbxInfo> info) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("fileVersion");
    JSON_STRING_VALUE(writer, info->fileVersion);

    writer.String("fileCreator");
    JSON_STRING_VALUE(writer, info->fileCreator);

    writer.String("lastSavedVendor");
    JSON_STRING_VALUE(writer, info->lastSavedVendor);

    writer.String("lastSavedAppName");
    JSON_STRING_VALUE(writer, info->lastSavedAppName);

    writer.String("lastSavedAppVersion");
    JSON_STRING_VALUE(writer, info->lastSavedAppVersion);

    writer.String("materialCount");
    writer.Int(info->materialCount);
    writer.String("textureCount");
    writer.Int(info->textureCount);
    writer.String("geometryCount");
    writer.Int(info->geometryCount);
    writer.String("nonSkinnedMeshCount");
    writer.Int(info->nonSkinnedMeshCount);
    writer.String("skinnedMeshCount");
    writer.Int(info->skinnedMeshCount);
    writer.String("hasAnimation");
    writer.Bool(info->bHasAnimation);
    writer.String("frameRate");
    writer.Double(info->framerate);
    writer.String("totalTime");
    writer.Double(info->time);
    writer.String("unitSystem");
    JSON_STRING_VALUE(writer, info->unitSystem);

    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

std::shared_ptr<Fbx::Options> GetOptions(const std::string& content) {
    /*
     * Validate file json content.
     */
    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError() || !document.IsObject()) {
        LOG(Fbx::Utils::ELogLevel::Error, "Invalid JSON input.");
        return nullptr;
    }

    static rapidjson::Document importOptionsJsonSchemaDocument;
    if (importOptionsJsonSchemaDocument.Parse(ImportOptionsJsonSchema.c_str()).HasParseError()) {
        std::string error = "Failed to parse json:" + std::to_string(importOptionsJsonSchemaDocument.GetErrorOffset()) + " .";
        LOG(Fbx::Utils::ELogLevel::Error, error);
        return nullptr;
    }
    static rapidjson::SchemaDocument schema(importOptionsJsonSchemaDocument);
    static rapidjson::SchemaValidator validator(schema);
    if (!validator.IsValid()) {
        LOG(Fbx::Utils::ELogLevel::Error, "Invalid JSON validator.");
        return nullptr;
    }
    if (!document.Accept(validator)) {
        rapidjson::StringBuffer sb;
        validator.GetInvalidSchemaPointer().StringifyUriFragment(sb);
        std::string info = "Invalid keyword: " + std::string(validator.GetInvalidSchemaKeyword()) + ".\n  Invalid document: " + std::string(sb.GetString()) + ".";
        LOG(Fbx::Utils::ELogLevel::Error, info);
        sb.Clear();
        validator.GetInvalidDocumentPointer().StringifyUriFragment(sb);
        std::string errorFragment = sb.GetString();
        sb.Clear();
        LOG(Fbx::Utils::ELogLevel::Error, "Schema error: " + errorFragment);
        return nullptr;
    }

    auto importOptions = std::make_shared<Fbx::Options>();

    if (document.HasMember("convertAxis")) {
        importOptions->bConvertAxis = document["convertAxis"].GetBool();
    }

    if (document.HasMember("convertUnit")) {
        importOptions->bConvertUnit = document["convertUnit"].GetBool();
    }
    if (document.HasMember("animation")) {
        auto& animationProperties = document["animation"];
        if (animationProperties.HasMember("importAnimations")) {
            importOptions->bParseAnimations = animationProperties["importAnimations"].GetBool();
        }
        if (animationProperties.HasMember("animationLengthImportType")) {
            importOptions->animationLengthParseType = static_cast<Fbx::Options::EFBXAnimationLengthType>(animationProperties["animationLengthImportType"].GetInt());
        }
        if (animationProperties.HasMember("resample")) {
            importOptions->bResample = animationProperties["resample"].GetBool();
        }
        if (animationProperties.HasMember("resampleRate")) {
            importOptions->resampleRate = animationProperties["resampleRate"].GetBool();
        }
        if (animationProperties.HasMember("bImportMorph")) {
            importOptions->bParseMorphTargets = animationProperties["bImportMorph"].GetBool();
        }
        if (animationProperties.HasMember("doNotImportCurveWithZero")) {
            importOptions->bDoNotParseCurveWithZero = animationProperties["doNotImportCurveWithZero"].GetBool();
        }
        if (animationProperties.HasMember("importCustomAttribute")) {
            importOptions->bParseCustomAttribute = animationProperties["importCustomAttribute"].GetBool();
        }
        if (animationProperties.HasMember("importBoneTracks")) {
            importOptions->bParseBoneTracks = animationProperties["importBoneTracks"].GetBool();
        }
        if (animationProperties.HasMember("preserveLocalTransform")) {
            importOptions->bPreserveLocalTransform = animationProperties["preserveLocalTransform"].GetBool();
        }
    }

    if (document.HasMember("mesh")) {
        auto& meshProperties = document["mesh"];
        if (meshProperties.HasMember("preserveSmoothingGroups")) {
            importOptions->bPreserveSmoothingGroups = meshProperties["preserveSmoothingGroups"].GetBool();
        }
        if (meshProperties.HasMember("importMorphTargets")) {
            importOptions->bParseMorphTargets = meshProperties["importMorphTargets"].GetBool();
        }
        if (meshProperties.HasMember("thresholdPosition")) {
            importOptions->thresholdPosition = meshProperties["thresholdPosition"].GetFloat();
        }
        if (meshProperties.HasMember("thresholdTangentNormal")) {
            importOptions->thresholdTangentNormal = meshProperties["thresholdTangentNormal"].GetFloat();
        }
        if (meshProperties.HasMember("thresholdUV")) {
            importOptions->thresholdUV = meshProperties["thresholdUV"].GetFloat();
        }
        if (meshProperties.HasMember("morphThresholdPosition")) {
            importOptions->morphThresholdPosition = meshProperties["morphThresholdPosition"].GetFloat();
        }
        if (meshProperties.HasMember("computeNormal")) {
            importOptions->computeNormal = static_cast<Fbx::Options::NormalOptions>(meshProperties["computeNormal"].GetInt());
        }
        if (meshProperties.HasMember("computeTangent")) {
            importOptions->computeTangent = static_cast<Fbx::Options::TangentOptions>(meshProperties["computeTangent"].GetInt());
        }
    }

    return importOptions;
}

int main(int argc, char* argv[]) {
    cxxopts::Options options("FBX Importer", "A tool for parsing FBX files.");
    std::string inputFile;
    std::string outDirectory = "";
    std::string logFilePath = "";
    std::string tempDirectory = "";
    std::string configFileContent = "";
    std::string version = "0.0.1.3";

    try {
        // clang-format off
		options.add_options()
			("o,output", "Output directory", cxxopts::value<std::string>())
			("c,config", "Configuration file", cxxopts::value<std::string>())
			("l,log", "Log file path", cxxopts::value<std::string>())
			("f,file", "Input file", cxxopts::value<std::string>(), "FILE")
			("t,temp", "Temp directory", cxxopts::value<std::string>())
			("v", "Version");
        // clang-format on

        auto opts = options.parse(argc, argv);

        if (opts.count("v") != 0) {
            std::cout << version << std::endl;
            return 0;
        }

        if (opts.count("output") == 0) {
            LOG(Fbx::Utils::ELogLevel::Error, "COULDN'T find <output> in command line parameters.");
            std::cout << options.help() << std::endl;
            return 1;
        }

        outDirectory = opts["output"].as<std::string>();

        if (opts.count("log") != 0) {
            logFilePath = opts["log"].as<std::string>();
        } else {
            logFilePath = outDirectory + "/fbx.log";
        }
        Fbx::Utils::SetGlobalLogger(Fbx::Utils::GetDefaultLogger(logFilePath));

        if (opts.count("f") == 0) {
            LOG(Fbx::Utils::ELogLevel::Error, "COULDN'T find <f> in command line parameters.");
            std::cout << options.help() << std::endl;
            return 1;
        } else {
            inputFile = opts["f"].as<std::string>();
        }

        if (opts.count("temp") != 0) {
            tempDirectory = opts["temp"].as<std::string>();
        }

        if (opts.count("config") != 0) {
            std::string configFilePath = opts["config"].as<std::string>();
            std::vector<char> ret;
            bool found = false;
            std::ifstream configFile(configFilePath, std::ios_base::in);
            if (configFile) {
                configFile.seekg(0, std::ios::end);
                ret.resize(static_cast<size_t>(configFile.tellg()));
                configFile.seekg(0, std::ios::beg);
                configFile.read(ret.data(), ret.size());
                ret.resize(static_cast<size_t>(configFile.gcount()));
                configFileContent = std::string(ret.begin(), ret.end());
            }
        }

        std::shared_ptr<Fbx::Options> importOptions = nullptr;
        if (configFileContent != "") {
            importOptions = GetOptions(configFileContent);
        }

        if (!importOptions) {
            importOptions = std::make_shared<Fbx::Options>();
        }

        LOG(Fbx::Utils::ELogLevel::Info, "Programe ver: " + version + " .");
        LOG(Fbx::Utils::ELogLevel::Info, "Input file: " + inputFile + " .");
        LOG(Fbx::Utils::ELogLevel::Info, "Output directory: " + outDirectory + " .");
        LOG(Fbx::Utils::ELogLevel::Info, "Log file: " + logFilePath + " .");
        LOG(Fbx::Utils::ELogLevel::Info, "Temporary directory: " + tempDirectory + " .");

#ifdef _MSC_VER
        int acpCode = -1;
        try {
            std::wstring acp = Exporter::GetStringValueFromHKLM(L"SYSTEM\\CurrentControlSet\\Control\\Nls\\CodePage\\", L"ACP");
            acpCode = std::stoi(acp);
        } catch (std::exception& e) {
        }
        LOG(Fbx::Utils::ELogLevel::Info, "Win32 ACP value: " + std::to_string(acpCode) + " .");
#endif

        importOptions->embeddingExtractionFolder = tempDirectory;

        // -o "C:\\Users\\\jasonjwang\\Desktop\\FBX\\out" -f "C:\\Users\\\jasonjwang\\Desktop\\FBX\\Rumba Dancing.fbx" -l "C:\\Users\\\jasonjwang\\Desktop\\FBX\\out\\fbx.log"
        Exporter::SetExportDirectory(outDirectory);
        Exporter::Context::RegisterComponentConverter("Convert0", Exporter::Converter3D);
        Exporter::Context::RegisterComponentConverter("Convert1", Exporter::Converter3DMesh);
        auto result = Fbx::ParseFbx(inputFile, importOptions);
        if (result.first && result.second) {
            auto prefab = std::make_shared<Exporter::Prefab>(result.second, result.second->name);
            auto prefabPath = prefab->Export();
            Exporter::ExportStore::SaveStorage(prefabPath);
            std::string infoJson = toString(result.first);
            auto savePath = ghc::filesystem::path(Exporter::GetExportDirectory()) / "info.json";
            if (Exporter::EnsurePath(savePath.generic_string())) {
                std::ofstream infoFile(savePath.generic_string(), std::ios::out);
#ifdef _MSC_VER
                infoJson = Exporter::ANSItoUTF8(infoJson);
#endif
                if (infoFile) {
                    infoFile.write(infoJson.data(), infoJson.size());
                    infoFile.close();
                }
            }
        }
        LOG(Fbx::Utils::ELogLevel::Info, "Parse FBX file done.");
        return 0;
    } catch (const cxxopts::OptionException& e) {
        LOG(Fbx::Utils::ELogLevel::Error, e.what());
        std::cout << options.help() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        LOG(Fbx::Utils::ELogLevel::Error, e.what());
        return 1;
    } catch (...) {
        LOG(Fbx::Utils::ELogLevel::Error, "Unknown command option error.");
        return 1;
    }
}
