#include "Engine.h"
#include "Resource.h"
namespace Exporter {

Prefab::Prefab(std::shared_ptr<Fbx::Objects::Entity> inRoot, const std::string& inName) : Resource(""), name(inName), root(inRoot) { context = std::make_shared<Exporter::Context>(); }

std::string Prefab::GetHash() { return "prefab_" + root->name; }
std::string Prefab::GetExportPath() {
    std::string savePath = name + ".prefab";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = name + "_" + std::to_string(++renameCount) + ".prefab";
        }
    }
    return savePath;
}
std::string Prefab::ExportResource() {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("name");
    JSON_STRING_VALUE(writer, name);
    writer.String("type");
    JSON_STRING_VALUE(writer, std::string("3D"));

    auto entity = context->IterateGameObject(root);
    writer.String("gameObjectList");
    JSON_RAW_VALUE(writer, context->GetGameObjectListJSON(), rapidjson::Type::kArrayType);
    writer.String("componentList");
    JSON_RAW_VALUE(writer, context->GetComponentListJSON(), rapidjson::Type::kArrayType);

    writer.EndObject();
    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif

    for (auto dep : context->resourceList) {
        AddDependencies(dep);
    }
    return json;
}
std::string Prefab::GetResourceType() { return "prefab"; }
}  // namespace Exporter