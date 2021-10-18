#include "Engine.h"
#include "Resource.h"
namespace Exporter {

std::shared_ptr<Entity> Entity::GetParent() { return _parent; }
void Entity::SetParent(std::shared_ptr<Entity> value) {
    auto thisPtr = shared_from_this();
    if (_parent != nullptr) {
        auto index = std::find(_parent->children.begin(), _parent->children.end(), thisPtr);
        if (index != _parent->children.end()) {
            _parent->children.erase(index);
        }
    }
    if (value != nullptr) {
        _parent = value;
        _parent->children.push_back(thisPtr);
    }
}

Entity::Entity(std::shared_ptr<Fbx::Objects::Entity> _go, int _arrayId) {
    gameObject = _go;
    // arrayId = -1;
    arrayId = _arrayId;
}

void Entity::AddChild(std::shared_ptr<Entity> entity) {
    auto thisPtr = shared_from_this();
    entity->SetParent(thisPtr);
}

int Entity::GetChildrenCount() { return children.size(); }

std::string Entity::ToJSON() {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("name");
    JSON_STRING_VALUE(writer, name);

    if (children.size() > 0) {
        writer.String("children");
        writer.StartArray();
        for (auto go : children) {
            writer.Int(go->arrayId);
        }
        writer.EndArray();
    }

    if (components.size() > 0) {
        writer.String("components");
        writer.StartArray();
        for (auto comp : components) {
            writer.Int(comp);
        }
        writer.EndArray();
    }

    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(active);
    writer.String("layer");
    writer.Int(0);

    writer.EndObject();

    writer.EndObject();
    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

std::string Context::GetGameObjectListJSON() {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartArray();
    int index = 0;
    for (auto gameObject : gameObjectList) {
        std::string result = gameObject->ToJSON();
        if (result == "") {
            LOG(Fbx::Utils::ELogLevel::Error, "Got empty JSON output.");
        } else {
            rapidjson::Document entityDoc;
            if (entityDoc.Parse(result.c_str()).HasParseError()) {
                continue;
            }
            entityDoc.AddMember("_index", index++, entityDoc.GetAllocator());
            {
                rapidjson::StringBuffer buffer;
                buffer.Clear();
                rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
                entityDoc.Accept(writer);
                result = buffer.GetString();
            }
            JSON_RAW_VALUE(writer, result, rapidjson::Type::kObjectType);
        }
    }
    writer.EndArray();
    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

std::string Context::GetComponentListJSON() {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartArray();
    int index = 0;
    for (auto component : componentList) {
        std::string result = component->GetJSON();
        if (result == "") {
            LOG(Fbx::Utils::ELogLevel::Error, "Got empty JSON ouput for component.");
        } else {
            rapidjson::Document entityDoc;
            if (entityDoc.Parse(result.c_str()).HasParseError()) {
                continue;
            }
            entityDoc.AddMember("_index", index++, entityDoc.GetAllocator());
            {
                rapidjson::StringBuffer buffer;
                buffer.Clear();
                rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
                entityDoc.Accept(writer);
                result = buffer.GetString();
            }
            JSON_RAW_VALUE(writer, result, rapidjson::Type::kObjectType);
        }
    }
    writer.EndArray();
    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

std::map<std::string, Context::ComponentConverter> Context::ComponentConverters;

std::shared_ptr<Entity> Context::MakeEntity(std::shared_ptr<Fbx::Objects::Entity> gameObject) {
    if (gameObject != nullptr && (entityMap.find(gameObject) != entityMap.end())) {
        return entityMap[gameObject];
    }

    auto entity = std::make_shared<Entity>(gameObject, static_cast<int>(gameObjectList.size()));
    if (gameObject != nullptr) {
        entityMap.emplace(gameObject, entity);
    }
    gameObjectList.push_back(entity);
    return entity;
}

void Context::RegisterComponentConverter(std::string priority, ComponentConverter entityConverter) { ComponentConverters.emplace(priority, entityConverter); }

std::shared_ptr<Entity> Context::IterateGameObject(std::shared_ptr<Fbx::Objects::Entity> go, std::shared_ptr<Entity> parent) {
    if (!bIsRoot) {
        bIsRoot = true;
        root = go;
    }

    auto entity = MakeEntity(go);
    entity->SetParent(parent);
    entity->name = go->name;

    entity->active = true;
    for (auto converterPair : ComponentConverters) {
        ComponentConverter& converter = converterPair.second;
        IterateGameObject(go, converter, parent);
    }
    return entity;
}

std::shared_ptr<Entity> Context::IterateGameObject(std::shared_ptr<Fbx::Objects::Entity> go, ComponentConverter converter, std::shared_ptr<Entity> parent) {
    if (!bIsRoot) {
        bIsRoot = true;
        root = go;
    }

    auto entity = MakeEntity(go);
    entity->SetParent(parent);
    entity->name = go->name;

    entity->active = true;

    auto thisPtr = shared_from_this();

    converter(go, entity, thisPtr);
    for (auto& child : go->children) {
        IterateGameObject(child, converter, entity);
    }
    return entity;
}

int Context::AddComponent(std::shared_ptr<Component> component, std::shared_ptr<Fbx::Objects::Component> inComponent) {
    auto thisPtr = shared_from_this();
    if (inComponent != nullptr && (componentMap.find(inComponent->id) != componentMap.end())) {
        // Debug.Log("" + nativeComponent.GetInstanceID());
        auto comp = componentMap[inComponent->id];
        comp->OnIterateTo(thisPtr);
        return comp->arrayId;
    } else {
        componentList.push_back(component);
        int num = componentList.size() - 1;
        component->arrayId = num;
        if (inComponent != nullptr) {
            // Debug.Log("" + nativeComponent.GetInstanceID());
            componentMap.emplace(inComponent->id, component);
        }
        component->OnIterateTo(thisPtr);
        return num;
    }
}

void Context::AddResource(const std::string& path) {
    if (path == "/" || path == "") {
        return;
    }
    if (std::find(resourceList.begin(), resourceList.end(), path) != resourceList.end()) {
        return;
    }
    resourceList.push_back(path);
}

std::string Component::GetJSON() { return json; }

void Component::OnIterateTo(std::shared_ptr<Context> context) { json = ToJSON(context); }

Component::Component() {}

Transform3D::Transform3D(std::shared_ptr<Fbx::Objects::Transform> inTransform) : Component(), transform(inTransform) {}

std::string Transform3D::GetTypeName() { return "Transform3D"; }

std::string Transform3D::ToJSON(std::shared_ptr<Context> context) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("type");
    JSON_STRING_VALUE(writer, GetTypeName());
    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(true);
    writer.String("position");
    writer.StartArray();
    writer.Double(transform->transform.translation.x);
    writer.Double(transform->transform.translation.y);
    writer.Double(transform->transform.translation.z);
    writer.EndArray();
    writer.String("scale");
    writer.StartArray();
    writer.Double(transform->transform.scale.x);
    writer.Double(transform->transform.scale.y);
    writer.Double(transform->transform.scale.z);
    writer.EndArray();
    writer.String("rotation");
    writer.StartArray();
    writer.Double(transform->transform.quaternion.x);
    writer.Double(transform->transform.quaternion.y);
    writer.Double(transform->transform.quaternion.z);
    writer.Double(transform->transform.quaternion.w);
    writer.EndArray();
    writer.EndObject();
    writer.EndObject();
    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

MeshRenderer::MeshRenderer(std::shared_ptr<Fbx::Objects::MeshRenderer> inMeshRenderer) : Component(), meshRenderer(inMeshRenderer) {}

std::string MeshRenderer::GetTypeName() { return "MeshRenderer"; }

std::string MeshRenderer::ToJSON(std::shared_ptr<Context> context) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("type");
    JSON_STRING_VALUE(writer, GetTypeName());
    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(true);

    writer.String("mesh");
    if (meshRenderer->mesh) {
        std::string meshPath = std::make_shared<Mesh>(meshRenderer->mesh)->Export();
        if (meshPath != "") {
            JSON_STRING_VALUE(writer, meshPath);
            context->AddResource(meshPath);
        } else {
            writer.Null();
        }
    } else {
        writer.Null();
    }

    writer.String("lightMap");
    writer.Null();

    writer.String("lightMapScaleOffset");
    writer.StartArray();
    writer.Double(1);
    writer.Double(1);
    writer.Double(0);
    writer.Double(0);
    writer.EndArray();

    writer.String("castShadow");
    writer.Bool(true);

    writer.String("receiveShadow");
    writer.Bool(true);

    writer.String("lightMapIndex");
    writer.Int(-1);

    writer.String("materials");
    writer.StartArray();
    if (meshRenderer->mesh) {
        for (auto& submesh : meshRenderer->mesh->subMeshes) {
            if (submesh->material) {
                std::string matPath = std::make_shared<Material>(submesh->material)->Export();
                if (matPath != "") {
                    JSON_STRING_VALUE(writer, matPath);
                    context->AddResource(matPath);
                } else {
                    writer.Null();
                }
            } else {
                writer.Null();
            }
        }
    }
    writer.EndArray();

    writer.EndObject();
    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}  // namespace Exporter

SkinnedMeshRenderer::SkinnedMeshRenderer(std::shared_ptr<Fbx::Objects::MeshRenderer> inSkinnedMeshRenderer) : Component(), skinnedMeshRenderer(inSkinnedMeshRenderer) {}

std::string SkinnedMeshRenderer::GetTypeName() { return "SkinnedMeshRenderer"; }

std::string SkinnedMeshRenderer::ToJSON(std::shared_ptr<Context> context) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();

    writer.String("type");
    JSON_STRING_VALUE(writer, GetTypeName());
    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(true);

    writer.String("mesh");
    if (skinnedMeshRenderer->mesh && skinnedMeshRenderer->skeleton) {
        std::string skMeshPath = std::make_shared<SkinnedMesh>(skinnedMeshRenderer->mesh, skinnedMeshRenderer->skeleton ? skinnedMeshRenderer->skeleton->skeleton : nullptr)->Export();
        if (skMeshPath != "") {
            JSON_STRING_VALUE(writer, skMeshPath);
            context->AddResource(skMeshPath);
        } else {
            writer.Null();
        }
    } else {
        writer.Null();
    }

    writer.String("lightMap");
    writer.Null();

    writer.String("lightMapScaleOffset");
    writer.StartArray();
    writer.Double(1);
    writer.Double(1);
    writer.Double(0);
    writer.Double(0);
    writer.EndArray();

    writer.String("castShadow");
    writer.Bool(true);

    writer.String("receiveShadow");
    writer.Bool(true);

    writer.String("lightMapIndex");
    writer.Int(-1);

    writer.String("skeleton");
    if (skinnedMeshRenderer->skeleton && (context->componentMap.find(skinnedMeshRenderer->skeleton->id) != context->componentMap.end())) {
        int skeletonComp = context->AddComponent(context->componentMap[skinnedMeshRenderer->skeleton->id], skinnedMeshRenderer->skeleton);
        if (skeletonComp >= 0) {
            writer.Int(skeletonComp);
        } else {
            writer.Null();
        }
    } else {
        writer.Null();
    }

    writer.String("materials");
    writer.StartArray();
    if (skinnedMeshRenderer->mesh) {
        for (auto& submesh : skinnedMeshRenderer->mesh->subMeshes) {
            if (submesh->material) {
                std::string matPath = std::make_shared<Material>(submesh->material)->Export();
                if (matPath != "") {
                    JSON_STRING_VALUE(writer, matPath);
                    context->AddResource(matPath);
                } else {
                    writer.Null();
                }
            } else {
                writer.Null();
            }
        }
    }
    writer.EndArray();

    writer.EndObject();
    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

Skeleton::Skeleton(std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton) : Component(), skeleton(inSkeleton) {}

std::string Skeleton::GetTypeName() { return "Skeleton"; }

std::string Skeleton::ToJSON(std::shared_ptr<Context> context) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("type");
    JSON_STRING_VALUE(writer, GetTypeName());
    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(true);
    writer.String("configuration");

    if (skeleton) {
        writer.StartObject();
        writer.String("type");
        JSON_STRING_VALUE(writer, std::string("Static"));
        writer.String("avatar");
        std::string avatarPath = std::make_shared<Avatar>(skeleton)->Export();
        context->AddResource(avatarPath);
        JSON_STRING_VALUE(writer, avatarPath);
        writer.String("remapping");
        writer.StartObject();
        writer.EndObject();
        writer.EndObject();
    } else {
        writer.Null();
    }

    writer.EndObject();
    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

Animation::Animation(std::shared_ptr<Fbx::Objects::Animation> inAnimation) : animation(inAnimation) {}

std::string Animation::GetTypeName() { return "Animation"; }

std::string Animation::ToJSON(std::shared_ptr<Context> context) {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("type");
    JSON_STRING_VALUE(writer, GetTypeName());
    writer.String("data");
    writer.StartObject();
    writer.String("active");
    writer.Bool(true);

    writer.String("skeleton");
    if (skeletonIndex >= 0) {
        writer.Int(skeletonIndex);
    } else {
        writer.Null();
    }

    writer.String("clips");
    writer.StartArray();
    for (auto& kvp : animation->animationClips) {
        if (!kvp.second) {
            continue;
        }
        writer.StartObject();
        writer.String("name");
        JSON_STRING_VALUE(writer, kvp.first);
        writer.String("clip");
        std::string clipPath = std::make_shared<AnimationClip>(kvp.second)->Export();
        JSON_STRING_VALUE(writer, clipPath);
        if (clipPath != "") {
            context->AddResource(clipPath);
        }
        writer.EndObject();
    }
    writer.EndArray();

    writer.String("autoPlay");
    writer.Bool(true);

    writer.String("speed");
    writer.Double(1);

    writer.EndObject();
    writer.EndObject();

    std::string json = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(json);
#endif
    return json;
}

void Converter3D(std::shared_ptr<Fbx::Objects::Entity> node, std::shared_ptr<Entity> entity, std::shared_ptr<Context> context) {
    if (node->transform) {
        auto inTrans = std::static_pointer_cast<Fbx::Objects::Transform>(node->transform);
        auto trans = std::make_shared<Transform3D>(inTrans);
        entity->components.push_back(context->AddComponent(trans, inTrans));
    }
    int skeletonIndex = -1;
    for (auto comp : node->components) {
        if (comp->GetTypeName() == Fbx::Objects::EComponentType::Transform) {
            auto inTrans = std::static_pointer_cast<Fbx::Objects::Transform>(comp);
            auto trans = std::make_shared<Transform3D>(inTrans);
            entity->components.push_back(context->AddComponent(trans, inTrans));
        } else if (comp->GetTypeName() == Fbx::Objects::EComponentType::Skeleton) {
            auto inSkeleton = std::static_pointer_cast<Fbx::Objects::Skeleton>(comp);
            if (inSkeleton->skeleton->bForSkinnedMesh) {
                auto skeleton = std::make_shared<Skeleton>(inSkeleton->skeleton);
                skeletonIndex = context->AddComponent(skeleton, inSkeleton);
                entity->components.push_back(skeletonIndex);
            } else {
                auto skeleton = std::make_shared<Skeleton>(nullptr);
                skeletonIndex = context->AddComponent(skeleton, inSkeleton);
                entity->components.push_back(skeletonIndex);
            }
        }
    }

    for (auto comp : node->components) {
        if (comp->GetTypeName() == Fbx::Objects::EComponentType::Animation) {
            auto inAnimation = std::static_pointer_cast<Fbx::Objects::Animation>(comp);
            if (inAnimation->animationClips.size()) {
                auto animation = std::make_shared<Animation>(inAnimation);
                animation->skeletonIndex = skeletonIndex;
                entity->components.push_back(context->AddComponent(animation, std::make_shared<Fbx::Objects::Component>()));
            }
        }
    }
}

void Converter3DMesh(std::shared_ptr<Fbx::Objects::Entity> node, std::shared_ptr<Entity> entity, std::shared_ptr<Context> context) {
    for (auto comp : node->components) {
        if (comp->GetTypeName() == Fbx::Objects::EComponentType::MeshRenderer) {
            auto inMeshrenderer = std::static_pointer_cast<Fbx::Objects::MeshRenderer>(comp);
            if (inMeshrenderer->mesh) {
                if (!inMeshrenderer->mesh->bForSkinnedMesh) {
                    auto meshrenderer = std::make_shared<MeshRenderer>(inMeshrenderer);
                    entity->components.push_back(context->AddComponent(meshrenderer, inMeshrenderer));
                } else {
                    auto meshrenderer = std::make_shared<SkinnedMeshRenderer>(inMeshrenderer);
                    entity->components.push_back(context->AddComponent(meshrenderer, inMeshrenderer));
                }
            }
        }
    }
}

}  // namespace Exporter
