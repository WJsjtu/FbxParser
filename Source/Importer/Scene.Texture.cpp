#include "Scene.h"
#include "Decoder.h"
#include "FbxParser.private.h"
#include "ghc/filesystem.hpp"
#include <fstream>

namespace Fbx { namespace Importer {
std::shared_ptr<Assets::TextureAsset> Scene::ImportTexture(FbxFileTexture* fbxTexture, bool bSetupAsNormalMap) {
    if (!fbxTexture) {
        return nullptr;
    }

    // create an texture asset
    std::shared_ptr<Assets::TextureAsset> exportTexture = nullptr;
    std::string absoluteFilename = ImporterHelper::UTF8ToNative(fbxTexture->GetFileName());
    std::string extension = ghc::filesystem::path(absoluteFilename).extension().generic_string();
    Utils::ToLowerCase(extension);

    // name the texture with file name
    std::string textureName;
    auto foundTexture = FbxTextureToUniqueNameMap.find(fbxTexture);
    if (foundTexture != FbxTextureToUniqueNameMap.end()) {
        textureName = foundTexture->second;
    } else {
        textureName = ghc::filesystem::path(absoluteFilename).stem().generic_string();
        textureName = Utils::SanitizeObjectName(textureName);
        FbxTextureToUniqueNameMap[fbxTexture] = textureName;
    }

    std::string finalFilePath;
    if (Utils::ExistPath(absoluteFilename)) {
        // try opening from absolute path
        finalFilePath = absoluteFilename;
        std::vector<char> dataBinary;
        if (!finalFilePath.empty()) {
            std::ifstream fileStream(finalFilePath, std::ios_base::in | std::ios::binary);
            if (fileStream) {
                fileStream.unsetf(std::ios::skipws);
                fileStream.seekg(0, std::ios::end);
                dataBinary.resize(static_cast<size_t>(fileStream.tellg()));
                fileStream.seekg(0, std::ios::beg);
                fileStream.read(reinterpret_cast<char*>(dataBinary.data()), dataBinary.size());
                dataBinary.resize(static_cast<size_t>(fileStream.gcount()));
            }
        }

        if (dataBinary.size() > 0) {
            // UE_LOG(LogFbxMaterialImport, Verbose, TEXT("Loading texture file %s"), *FinalFilePath);

            // save texture settings if texture exist
            const std::string fileType = extension;

            // Unless the normal map setting is used during import,
            //	the user has to manually hit "reimport" then "recompress now" button
            if (bSetupAsNormalMap) {
                // todo
                // TextureFact->bFlipNormalMapGreenChannel = ImportOptions->bInvertNormalMap;
            }
            if (textureAssets.find(fbxTexture) == textureAssets.end()) {
                try {
                    exportTexture = std::make_shared<Assets::TextureAsset>();
                    exportTexture->fileType = fileType;
                    exportTexture->uniqueID = std::to_string(fbxTexture->GetUniqueID());
                    exportTexture->name = textureName;
                    exportTexture->raw = dataBinary;
                    bool isRepeatU = (fbxTexture->WrapModeU.Get() == FbxTexture::EWrapMode::eRepeat);
                    exportTexture->wrapU = isRepeatU ? Assets::TextureAsset::EWrapMode::Repeat : Assets::TextureAsset::EWrapMode::Clamp;
                    bool isRepeatV = (fbxTexture->WrapModeV.Get() == FbxTexture::EWrapMode::eRepeat);
                    exportTexture->wrapV = isRepeatV ? Assets::TextureAsset::EWrapMode::Repeat : Assets::TextureAsset::EWrapMode::Clamp;
                    exportTexture->bNeedPremultiplyAlpha = fbxTexture->PremultiplyAlpha.Get();
                    exportTexture->bUseMipMap = fbxTexture->UseMipMap.Get();
                    exportTexture->originalPath = absoluteFilename;
                    exportTexture->bEnableCompress = true;

                    ImageDecoder::ImportImage image;
                    ImageDecoder::EImageFormat format = ImageDecoder::EImageFormat::Invalid;
                    if (fileType == ".png") {
                        format = ImageDecoder::EImageFormat::PNG;
                    } else if (fileType == ".jpeg" || fileType == ".jpg") {
                        format = ImageDecoder::EImageFormat::JPEG;
                    } else if (fileType == ".bmp") {
                        format = ImageDecoder::EImageFormat::BMP;
                    } else if (fileType == ".ico") {
                        format = ImageDecoder::EImageFormat::ICO;
                    } else if (fileType == ".exr") {
                        format = ImageDecoder::EImageFormat::EXR;
                    } else if (fileType == ".pcx") {
                        format = ImageDecoder::EImageFormat::PCX;
                    } else if (fileType == ".tga") {
                        format = ImageDecoder::EImageFormat::TGA;
                    }

                    std::string warn;
                    ImageDecoder::Vector<char> warnChars;
                    if (format != ImageDecoder::EImageFormat::Invalid && ImageDecoder::DecodeImage(format, reinterpret_cast<uint8_t*>(dataBinary.data()), dataBinary.size(), true, warnChars, image)) {
                        warn = std::string(warnChars.begin(), warnChars.end());
                        if (warn.size()) {
                            LOG_INFO(warn);
                        }

                        exportTexture->decoded = std::vector<uint8_t>(image.data.begin(), image.data.end());
                        exportTexture->width = image.width;
                        exportTexture->height = image.height;
                        exportTexture->numMips = image.numMips;
                        exportTexture->bEnableCompress = false;

                        if ((image.source.type == ImageDecoder::EImageFormat::PNG || image.source.type == ImageDecoder::EImageFormat::JPEG) && image.source.RGBFormat == ImageDecoder::ERGBFormat::RGBA && image.source.bitDepth == 8) {
                            exportTexture->bIsCommonPNGorJPGFile = true;
                            exportTexture->bEnableCompress = true;
                            exportTexture->pixelFormat = Assets::TextureAsset::EPixelFormat::RGBA8;
                        } else {
                            if (image.textureformat == ImageDecoder::ETextureSourceFormat::RGBA8 || image.textureformat == ImageDecoder::ETextureSourceFormat::RGBA16F) {
                                if (image.textureformat == ImageDecoder::ETextureSourceFormat::RGBA8) {
                                    exportTexture->pixelFormat = Assets::TextureAsset::EPixelFormat::RGBA8;
                                } else {
                                    exportTexture->pixelFormat = Assets::TextureAsset::EPixelFormat::RGBA16F;
                                }
                            } else if (image.textureformat == ImageDecoder::ETextureSourceFormat::RGBA16) {
                                exportTexture = nullptr;
                            } else if (image.textureformat == ImageDecoder::ETextureSourceFormat::BGRA8) {
                                std::vector<uint8_t> data;
                                data.resize(image.data.size());
                                memcpy(data.data(), image.data.data(), data.size());
                                for (int i = 0; i < data.size() / 4; i++) {
                                    char tmp = data[4 * i];
                                    data[4 * i] = data[4 * i + 2];
                                    data[4 * i + 2] = tmp;
                                }
                                exportTexture->decoded = data;
                                exportTexture->pixelFormat = Assets::TextureAsset::EPixelFormat::RGBA8;
                            } else {
                                exportTexture = nullptr;
                            }
                        }
                    } else {
                        if (warn.size()) {
                            LOG_ERROR(warn);
                        }
                        exportTexture = nullptr;
                    }

                } catch (...) {
                    exportTexture = nullptr;
                }

                if (exportTexture) {
                    textureAssets.emplace(fbxTexture, exportTexture);
                }
            }
        }
    } else {
        LOG_ERROR(fmt::format("Failed to open image file at {}.", absoluteFilename));
    }

    return exportTexture;
}
}}  // namespace Fbx::Importer
