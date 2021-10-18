#include "Resource.h"
namespace Exporter {

Material::Material(std::shared_ptr<Fbx::Assets::MaterialAsset> inMaterial) : Resource(""), material(inMaterial) {}

std::string Material::GetHash() { return material->uniqueID; }
std::string Material::GetExportPath() {
    std::string savePath = material->name + ".mat";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = material->name + "_" + std::to_string(++renameCount) + ".mat";
        }
    }
    return savePath;
}
std::string Material::ExportResource() {
    std::string mainTex = "";
    if (material->baseColor.texture) {
        mainTex = AddDependencies(std::make_shared<Texture>(material->baseColor.texture->texture));
    }
    std::string emissiveTex = "";
    if (material->emissive.texture) {
        emissiveTex = AddDependencies(std::make_shared<Texture>(material->emissive.texture->texture));
    }
    std::string specularTex = "";
    if (material->specular.texture) {
        specularTex = AddDependencies(std::make_shared<Texture>(material->specular.texture->texture));
    }
    std::string roughnessTex = "";
    if (material->roughness.texture) {
        roughnessTex = AddDependencies(std::make_shared<Texture>(material->roughness.texture->texture));
    }
    std::string metallicTex = "";
    if (material->metallic.texture) {
        metallicTex = AddDependencies(std::make_shared<Texture>(material->metallic.texture->texture));
    }
    std::string normalTex = "";
    if (material->normal.texture) {
        normalTex = AddDependencies(std::make_shared<Texture>(material->normal.texture->texture));
    }
    std::string opacityTex = "";
    if (material->opacity.texture) {
        opacityTex = AddDependencies(std::make_shared<Texture>(material->opacity.texture->texture));
    }
    std::string opacityMaskTex = "";
    if (material->opacityMask.texture) {
        opacityMaskTex = AddDependencies(std::make_shared<Texture>(material->opacityMask.texture->texture));
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    if (material->bForSkinnedMesh) {
        // BlinnPhong
        writer.String("effect");
        JSON_STRING_VALUE(writer, std::string("System::Effect::BlinnPhong"));
        writer.String("shaderParams");
        writer.StartObject();
        writer.String("_Color");
        writer.StartArray();
        if (material->baseColor.color) {
            writer.Double(material->baseColor.color->value.x);
            writer.Double(material->baseColor.color->value.y);
            writer.Double(material->baseColor.color->value.z);
            writer.Double(material->baseColor.color->value.w);
        } else {
            writer.Double(1);
            writer.Double(1);
            writer.Double(1);
            writer.Double(1);
        }
        writer.EndArray();
        writer.String("_SpecColor");
        writer.StartArray();
        if (material->specular.color) {
            writer.Double(material->specular.color->value.x);
            writer.Double(material->specular.color->value.y);
            writer.Double(material->specular.color->value.z);
        } else {
            writer.Double(0.5);
            writer.Double(0.5);
            writer.Double(0.5);
        }
        writer.Double(1);
        writer.EndArray();
        writer.String("_EmissionColor");
        writer.StartArray();
        if (material->emissive.color) {
            writer.Double(material->emissive.color->value.x);
            writer.Double(material->emissive.color->value.y);
            writer.Double(material->emissive.color->value.z);
            writer.Double(material->emissive.color->value.w);
        } else {
            writer.Double(1);
            writer.Double(1);
            writer.Double(1);
            writer.Double(1);
        }
        writer.EndArray();
        writer.String("_MainTex_ST");
        writer.StartArray();
        if (material->baseColor.texture) {
            writer.Double(material->baseColor.texture->tilling.x);
            writer.Double(material->baseColor.texture->tilling.y);
            writer.Double(material->baseColor.texture->tilling.z);
            writer.Double(material->baseColor.texture->tilling.w);
        } else {
            writer.Double(1);
            writer.Double(1);
            writer.Double(0);
            writer.Double(0);
        }
        writer.EndArray();
        writer.String("_AlbedoIntensity");
        writer.StartArray();
        writer.Double(1);
        writer.EndArray();
        writer.String("_Shininess");
        writer.StartArray();
        writer.Double(0.1);
        writer.EndArray();

        writer.String("_Cutoff");
        writer.StartArray();
        writer.Double(0.5);
        writer.EndArray();
        writer.EndObject();

        writer.String("textures");
        writer.StartObject();
        writer.String("_MainTex");
        JSON_STRING_VALUE(writer, (mainTex == "" ? "white" : mainTex));
        writer.String("_BumpMap");
        JSON_STRING_VALUE(writer, (normalTex == "" ? "white" : normalTex));
        writer.String("_EmissionMap");
        JSON_STRING_VALUE(writer, (emissiveTex == "" ? "white" : emissiveTex));
        writer.String("_SpecGlossMap");
        JSON_STRING_VALUE(writer, (specularTex == "" ? "white" : specularTex));
        writer.EndObject();

        writer.String("renderQueue");
        writer.Int(2000);

        writer.String("renderStates");
        writer.StartObject();
        writer.EndObject();

        writer.String("shaderDefinations");
        writer.StartObject();
        writer.String("USE_FOG");
        writer.Bool(true);
        if (normalTex != "") {
            writer.String("USE_NORMALMAP");
            writer.Bool(true);
        }
        if (opacityTex != "") {
            writer.String("USE_ALPHA_TEST");
            writer.Bool(true);
        }
        if (emissiveTex != "") {
            writer.String("USE_EMISSIONMAP");
            writer.Bool(true);
        }
        if (specularTex != "") {
            writer.String("USE_SPECMAP");
            writer.Bool(true);
        }
        writer.EndObject();
    } else {
        if (mainTex == "") {
            // Simple3D
            writer.String("effect");
            JSON_STRING_VALUE(writer, std::string("System::Effect::Simple3D"));
            writer.String("shaderParams");
            writer.StartObject();
            writer.String("_Color");
            writer.StartArray();
            if (material->baseColor.color) {
                writer.Double(material->baseColor.color->value.x);
                writer.Double(material->baseColor.color->value.y);
                writer.Double(material->baseColor.color->value.z);
                writer.Double(material->baseColor.color->value.w);
            } else {
                writer.Double(1);
                writer.Double(1);
                writer.Double(1);
                writer.Double(1);
            }
            writer.EndArray();
            writer.EndObject();

            writer.String("textures");
            writer.StartObject();
            writer.EndObject();

            writer.String("renderQueue");
            writer.Int(3000);

            writer.String("renderStates");
            writer.StartObject();
            writer.EndObject();

            writer.String("shaderDefinations");
            writer.StartObject();
            writer.EndObject();
        } else {
            // Effect3D
            writer.String("effect");
            JSON_STRING_VALUE(writer, std::string("System::Effect::Effect3D"));
            writer.String("shaderParams");
            writer.StartObject();
            writer.String("_MainTex_ST");
            writer.StartArray();
            if (material->baseColor.texture) {
                writer.Double(material->baseColor.texture->tilling.x);
                writer.Double(material->baseColor.texture->tilling.y);
                writer.Double(material->baseColor.texture->tilling.z);
                writer.Double(material->baseColor.texture->tilling.w);
            } else {
                writer.Double(1);
                writer.Double(1);
                writer.Double(0);
                writer.Double(0);
            }
            writer.EndArray();
            writer.String("_TintColor");
            writer.StartArray();
            if (material->baseColor.color) {
                writer.Double(material->baseColor.color->value.x);
                writer.Double(material->baseColor.color->value.y);
                writer.Double(material->baseColor.color->value.z);
                writer.Double(material->baseColor.color->value.w);
            } else {
                writer.Double(1);
                writer.Double(1);
                writer.Double(1);
                writer.Double(1);
            }
            writer.EndArray();
            writer.String("_Bright");
            writer.StartArray();
            writer.Double(1);
            writer.EndArray();
            writer.EndObject();

            writer.String("textures");
            writer.StartObject();
            writer.String("_MainTex");
            JSON_STRING_VALUE(writer, (mainTex == "" ? "white" : mainTex));
            writer.String("_MaskTex");
            JSON_STRING_VALUE(writer, (opacityMaskTex == "" ? "white" : opacityMaskTex));
            writer.EndObject();

            writer.String("renderQueue");
            writer.Int(3000);

            writer.String("renderStates");
            writer.StartObject();
            writer.EndObject();

            writer.String("shaderDefinations");
            writer.StartObject();
            writer.EndObject();
        }
    }

    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}
std::string Material::GetResourceType() { return "material"; }
}  // namespace Exporter