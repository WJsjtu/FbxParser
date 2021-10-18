#include "Scene.h"

namespace Fbx { namespace Importer {

bool Scene::CreateAndLinkExpressionForMaterialProperty(FbxSurfaceMaterial* fbxMaterial, std::shared_ptr<Assets::MaterialAsset> exportMaterial, const char* materialProperty, Assets::MaterialAsset::Parameter& materialInput, bool bSetupAsNormalMap, std::vector<std::string>& UVSet) {
    bool bCreated = false;
    FbxProperty fbxProperty = fbxMaterial->FindProperty(materialProperty);
    if (fbxProperty.IsValid()) {
        int unsupportedTextureCount = fbxProperty.GetSrcObjectCount<FbxLayeredTexture>();
        unsupportedTextureCount += fbxProperty.GetSrcObjectCount<FbxProceduralTexture>();
        if (unsupportedTextureCount > 0) {
            // UE_LOG(LogFbxMaterialImport, Warning, TEXT("Layered or procedural Textures are not supported (material
            // %s)"), UTF8_TO_TCHAR(FbxMaterial.GetName()));
        } else {
            int textureCount = fbxProperty.GetSrcObjectCount<FbxTexture>();
            if (textureCount > 0) {
                for (int textureIndex = 0; textureIndex < textureCount; textureIndex++) {
                    FbxFileTexture* FbxTexture = fbxProperty.GetSrcObject<FbxFileTexture>(textureIndex);

                    // create an texture asset
                    std::shared_ptr<Assets::TextureAsset> exportTexture = ImportTexture(FbxTexture, bSetupAsNormalMap);

                    if (exportTexture) {
                        float scaleU = static_cast<float>(FbxTexture->GetScaleU());
                        float scaleV = static_cast<float>(FbxTexture->GetScaleV());

                        // and link it to the material
                        std::shared_ptr<Assets::MaterialAsset::TextureInfo> textureInfo = std::make_shared<Assets::MaterialAsset::TextureInfo>();
                        materialInput.texture = textureInfo;
                        textureInfo->texture = exportTexture;
                        textureInfo->samplerType = bSetupAsNormalMap ? Assets::MaterialAsset::ETextureSamplerType::Mormal : Assets::MaterialAsset::ETextureSamplerType::Color;

                        // add/find UVSet and set it to the texture
                        FbxString UVSetName = FbxTexture->UVSet.Get();
                        std::string localUVSetName = ImporterHelper::UTF8ToNative(UVSetName.Buffer());
                        if (localUVSetName.empty()) {
                            localUVSetName = "UVmap_0";
                        }
                        auto UVFound = std::find(UVSet.begin(), UVSet.end(), localUVSetName);
                        int setIndex = UVFound == UVSet.end() ? -1 : static_cast<int>(UVFound - UVSet.begin());
                        if ((setIndex != 0 && setIndex != -1) || scaleU != 1.0f || scaleV != 1.0f) {
                            textureInfo->tilling.x = scaleU;
                            textureInfo->tilling.y = scaleV;
                        } else {
                            textureInfo->tilling.x = 1.0f;
                            textureInfo->tilling.y = 1.0f;
                        }
                        textureInfo->tilling.z = 0.0f;
                        textureInfo->tilling.w = 0.0f;

                        bCreated = true;
                    }
                }
            }
        }
    }

    return bCreated;
}

void Scene::FixupMaterial(FbxSurfaceMaterial* fbxMaterial, std::shared_ptr<Assets::MaterialAsset> exportMaterial) {
    // add a basic diffuse color if no texture is linked to diffuse
    if (exportMaterial->baseColor.texture == nullptr) {
        exportMaterial->baseColor.color = std::make_shared<Assets::MaterialAsset::ColorInfo>();
        FbxDouble3 diffuseColor;

        auto& colorExpression = exportMaterial->baseColor.color;
        bool bFoundDiffuseColor = true;
        if (fbxMaterial) {
            if (fbxMaterial->GetClassId().Is(FbxSurfacePhong::ClassId)) {
                diffuseColor = ((FbxSurfacePhong*)(fbxMaterial))->Diffuse.Get();
            } else if (fbxMaterial->GetClassId().Is(FbxSurfaceLambert::ClassId)) {
                diffuseColor = ((FbxSurfaceLambert*)(fbxMaterial))->Diffuse.Get();
            } else {
                bFoundDiffuseColor = false;
            }
        } else {
            bFoundDiffuseColor = false;
        }
        if (bFoundDiffuseColor) {
            colorExpression->value.x = (float)(diffuseColor[0]);
            colorExpression->value.y = (float)(diffuseColor[1]);
            colorExpression->value.z = (float)(diffuseColor[2]);
        } else {
            // use random color because there may be multiple materials, then they can be different
            colorExpression->value.x = 0.5f + (0.5f * Maths::Rand()) / static_cast<float>(RAND_MAX);
            colorExpression->value.y = 0.5f + (0.5f * Maths::Rand()) / static_cast<float>(RAND_MAX);
            colorExpression->value.z = 0.5f + (0.5f * Maths::Rand()) / static_cast<float>(RAND_MAX);
        }
        colorExpression->value.w = 1.0f;
    }
}

std::shared_ptr<Assets::MaterialAsset> Scene::CreateMaterial(FbxSurfaceMaterial* fbxMaterial, std::vector<std::string>& outUVSets, bool bForSkinnedMesh) {
    std::string finalMaterialName;
    {
        std::string materialFullName = ImporterHelper::MakeName(ImporterHelper::UTF8ToNative(fbxMaterial->GetName()));
        if (materialFullName.empty()) {
            materialFullName = "UnnamedMaterial";
        }
        materialFullName = Utils::SanitizeObjectName(materialFullName);
        finalMaterialName = materialFullName;
    }
    LOG_INFO(fmt::format("Creating material {} ...", finalMaterialName));

    // Check if we can use the specified base material to instance from it
    auto fbxImportOptions = Importer::GetInstance()->options;

    if (materialAssets.find(fbxMaterial) == materialAssets.end()) {
        // create an material asset
        std::shared_ptr<Assets::MaterialAsset> exportMaterial = std::make_shared<Assets::MaterialAsset>();
        exportMaterial->name = finalMaterialName;
        exportMaterial->uniqueID = std::to_string(fbxMaterial->GetUniqueID());
        exportMaterial->bForSkinnedMesh = bForSkinnedMesh;

        // textures and properties
        CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sDiffuse, exportMaterial->baseColor, false, outUVSets);
        CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sEmissive, exportMaterial->emissive, false, outUVSets);
        CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sSpecular, exportMaterial->specular, false, outUVSets);
        CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sSpecularFactor, exportMaterial->roughness, false, outUVSets);
        CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sShininess, exportMaterial->metallic, false, outUVSets);
        if (!CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sNormalMap, exportMaterial->normal, true, outUVSets)) {
            CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sBump, exportMaterial->normal, true,
                                                       outUVSets);  // no bump in unreal, use as normal map
        }
        if (CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sTransparentColor, exportMaterial->opacity, false, outUVSets)) {
            exportMaterial->blendMode = Assets::MaterialAsset::EBlendMode::Translucent;
            CreateAndLinkExpressionForMaterialProperty(fbxMaterial, exportMaterial, FbxSurfaceMaterial::sTransparencyFactor, exportMaterial->opacityMask, false, outUVSets);
        }
        FixupMaterial(fbxMaterial, exportMaterial);  // add random diffuse if none exists
        if (exportMaterial) {
            materialAssets.emplace(fbxMaterial, exportMaterial);
        }

        return exportMaterial;
    } else {
        return materialAssets[fbxMaterial];
    }
}  // namespace Importer

void Scene::FindOrImportMaterialsFromNode(FbxNode* fbxNode, std::vector<std::shared_ptr<Assets::MaterialAsset>>& outMaterials, std::vector<std::string>& UVSets, bool bForSkinnedMesh) {
    if (FbxMesh* meshNode = fbxNode->GetMesh()) {
        std::set<int> usedMaterialIndexes;

        for (int elementMaterialIndex = 0; elementMaterialIndex < meshNode->GetElementMaterialCount(); elementMaterialIndex++) {
            FbxGeometryElementMaterial* elementMaterial = meshNode->GetElementMaterial(elementMaterialIndex);
            switch (elementMaterial->GetMappingMode()) {
                case FbxLayerElement::eAllSame: {
                    if (elementMaterial->GetIndexArray().GetCount() > 0) {
                        usedMaterialIndexes.insert(elementMaterial->GetIndexArray()[0]);
                    }
                } break;
                case FbxLayerElement::eByPolygon: {
                    for (int materialIndex = 0; materialIndex < elementMaterial->GetIndexArray().GetCount(); materialIndex++) {
                        usedMaterialIndexes.insert(elementMaterial->GetIndexArray()[materialIndex]);
                    }
                } break;
            }
        }

        for (int materialIndex = 0, materialCount = fbxNode->GetMaterialCount(); materialIndex < materialCount; materialIndex++) {
            FbxSurfaceMaterial* fbxMaterial = fbxNode->GetMaterial(materialIndex);
            std::shared_ptr<Assets::MaterialAsset> materialImported = nullptr;

            // Only create the material used by the mesh element material
            if (fbxMaterial && usedMaterialIndexes.find(materialIndex) != usedMaterialIndexes.end()) {
                // Only create a new material if we are importing them and we could not find an existing one.
                materialImported = CreateMaterial(fbxMaterial, UVSets, bForSkinnedMesh);
            }

            // The fbxMaterial is not valid.
            outMaterials.push_back(materialImported);
        }
    } else {
        // Could not import the materials, no mesh found.
        for (int ii = 0; ii < fbxNode->GetMaterialCount(); ii++) {
            outMaterials.push_back(nullptr);
        }
    }
}

}}  // namespace Fbx::Importer
