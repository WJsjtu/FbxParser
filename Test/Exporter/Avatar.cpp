#include "Resource.h"
namespace Exporter {

std::string to_string(std::shared_ptr<Fbx::Assets::SkeletonAsset::Bone> bone) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("props");
    writer.StartObject();
    writer.String("name");
    JSON_STRING_VALUE(writer, bone->name);
    writer.String("translate");
    writer.StartArray();
    writer.Double(bone->transform.translation.x);
    writer.Double(bone->transform.translation.y);
    writer.Double(bone->transform.translation.z);
    writer.EndArray();
    writer.String("rotation");
    writer.StartArray();
    writer.Double(bone->transform.quaternion.x);
    writer.Double(bone->transform.quaternion.y);
    writer.Double(bone->transform.quaternion.z);
    writer.Double(bone->transform.quaternion.w);
    writer.EndArray();
    writer.String("scale");
    writer.StartArray();
    writer.Double(bone->transform.scale.x);
    writer.Double(bone->transform.scale.y);
    writer.Double(bone->transform.scale.z);
    writer.EndArray();
    writer.EndObject();
    writer.String("child");
    writer.StartArray();
    for (auto& child : bone->children) {
        JSON_RAW_VALUE(writer, to_string(child), rapidjson::Type::kObjectType);
    }
    writer.EndArray();
    writer.EndObject();

    std::string boneJSON = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(boneJSON);
#endif
    return boneJSON;
}

Avatar::Avatar(std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton) : Resource(""), skeleton(inSkeleton) {}

std::string Avatar::GetHash() { return skeleton->uniqueID; }

std::string Avatar::GetExportPath() {
    std::string savePath = skeleton->name + "_Avatar.avatar";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = skeleton->name + "_" + std::to_string(++renameCount) + "_Avatar.avatar";
        }
    }
    return savePath;
}

std::string Avatar::ExportResource() {
    std::string avatarJSON;
    if (skeleton->bones.size()) {
        std::shared_ptr<Fbx::Assets::SkeletonAsset::Bone> root = std::make_shared<Fbx::Assets::SkeletonAsset::Bone>();
        root->name = "RootNode";
        root->path = "/";
        root->uniqueID = "-1";
        root->children.push_back(skeleton->root);
        skeleton->root->parent = root;
        root->transform.translation = Fbx::Types::Vector3(0, 0, 0);
        root->transform.scale = Fbx::Types::Vector3(1, 1, 1);
        root->transform.quaternion = Fbx::Types::Vector4(0, 0, 0, 1);

        std::string treeJSON = to_string(root);
        skeleton->root->parent = nullptr;

        rapidjson::StringBuffer sb;
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
        writer.StartObject();
        writer.String("name");
        JSON_STRING_VALUE(writer, (skeleton->root->name + "_Avatar"));
        writer.String("rootNode");
        JSON_RAW_VALUE(writer, treeJSON, rapidjson::Type::kObjectType);
        writer.String("optimized");
        writer.Bool(true);
        writer.String("scaleFactor");
        writer.Double(1);
        writer.String("paths");
        writer.StartArray();
        writer.EndArray();
        writer.EndObject();

        avatarJSON = sb.GetString();
#ifdef _DEBUG
        PRETTIFY_JSON(avatarJSON);
#endif
    }
    return avatarJSON;
}

std::string Avatar::GetResourceType() { return "avatar"; }
}  // namespace Exporter