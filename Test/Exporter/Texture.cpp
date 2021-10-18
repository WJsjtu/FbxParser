#include "Resource.h"

namespace Exporter {
Texture::Texture(std::shared_ptr<Fbx::Assets::TextureAsset> inTexture) : Resource(""), texture(inTexture) {}
std::string Texture::GetHash() { return texture->uniqueID; }
std::string Texture::GetExportPath() {
    std::string savePath = texture->name + ".texture2d";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = texture->name + "_" + std::to_string(++renameCount) + ".texture2d";
        }
    }
    return savePath;
}
std::string Texture::ExportResource() {
    if (!texture->decoded.size()) {
        return "";
    }
    std::shared_ptr<ImageAssetFile> imageFile = nullptr;
    std::string imageSrc = "";
    std::string pixelFromat = "";
    static const std::vector<std::string> PixleFormatNames = {"RGBA8", "RGBA16F"};
    static const std::vector<std::string> WrapModeNames = {"Repeat", "Clamp"};
    if (texture->bIsCommonPNGorJPGFile) {
        imageFile = std::make_shared<ImageAssetFile>(texture->originalPath, texture->name, texture->fileType, texture->raw);
        imageSrc = AddFile(imageFile);

    } else {
        std::vector<char> decoded;
        decoded.resize(texture->decoded.size());
        memcpy(decoded.data(), texture->decoded.data(), texture->decoded.size());
		imageFile = std::make_shared<ImageAssetFile>(texture->originalPath, texture->name, texture->fileType + ".dataimage", decoded);
        imageSrc = AddFile(imageFile);
    }
    pixelFromat = PixleFormatNames[static_cast<uint32_t>(texture->pixelFormat)];

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("file");
    writer.StartObject();
    writer.String("src");
    JSON_STRING_VALUE(writer, imageSrc);
    writer.EndObject();

    writer.String("data");
    writer.StartObject();
    writer.String("width");
    writer.Int(texture->width);
    writer.String("height");
    writer.Int(texture->height);
    writer.String("mipmap");
    writer.Int(0);
    writer.String("useMipmap");
    writer.Bool(texture->bUseMipMap);
    writer.String("sRGB");
    writer.Bool(false);
    writer.String("needPremultiplyAlpha");
    writer.Bool(texture->bNeedPremultiplyAlpha);
    writer.String("wrapU");
    JSON_STRING_VALUE(writer, WrapModeNames[static_cast<uint32_t>(texture->wrapU)]);
    writer.String("wrapV");
    JSON_STRING_VALUE(writer, WrapModeNames[static_cast<uint32_t>(texture->wrapV)]);
    writer.String("filterMode");
    JSON_STRING_VALUE(writer, std::string("Bilinear"));
    writer.String("anisoLevel");
    writer.Int(1);
    writer.String("pixelFormat");
    JSON_STRING_VALUE(writer, pixelFromat);
    writer.EndObject();

    writer.String("editorInfo");
    writer.StartObject();
    writer.String("enableCompress");
    writer.Bool(texture->bEnableCompress);
    writer.EndObject();

    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}
std::string Texture::GetResourceType() { return "texture2d"; }
}  // namespace Exporter