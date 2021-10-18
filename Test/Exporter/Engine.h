#pragma once
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "FbxParser.h"
namespace Exporter {

class Entity : public std::enable_shared_from_this<Entity> {
public:
    std::string prefabPath;
    std::string unityAssetPath;
    std::string name;
    int arrayId;
    bool active = false;
    std::vector<int> components;
    std::shared_ptr<Fbx::Objects::Entity> gameObject;
    std::vector<std::shared_ptr<Entity>> children;

private:
    std::shared_ptr<Entity> _parent = nullptr;

public:
    std::shared_ptr<Entity> GetParent();
    void SetParent(std::shared_ptr<Entity> value);
    Entity(std::shared_ptr<Fbx::Objects::Entity> _go, int _arrayId);
    void AddChild(std::shared_ptr<Entity> entity);
    int GetChildrenCount();
    std::string ToJSON();
};

class Component;

class Context : public std::enable_shared_from_this<Context> {
public:
    std::vector<std::shared_ptr<Entity>> gameObjectList;
    std::vector<std::shared_ptr<Component>> componentList;
    std::vector<std::string> resourceList;

    std::map<std::shared_ptr<Fbx::Objects::Entity>, std::shared_ptr<Entity>> entityMap;
    std::map<std::string, std::shared_ptr<Component>> componentMap;

public:
    std::string GetGameObjectListJSON();
    std::string GetComponentListJSON();

private:
    typedef std::function<void(std::shared_ptr<Fbx::Objects::Entity>, std::shared_ptr<Entity>, std::shared_ptr<Context>)> ComponentConverter;

    static std::map<std::string, ComponentConverter> ComponentConverters;

    std::shared_ptr<Fbx::Objects::Entity> root = nullptr;

    bool bIsRoot = false;

    std::shared_ptr<Entity> MakeEntity(std::shared_ptr<Fbx::Objects::Entity> gameObject);

    std::shared_ptr<Entity> IterateGameObject(std::shared_ptr<Fbx::Objects::Entity> go, ComponentConverter converter, std::shared_ptr<Entity> parent = nullptr);

public:
    static void RegisterComponentConverter(std::string priority, ComponentConverter entityConverter);

    std::shared_ptr<Entity> IterateGameObject(std::shared_ptr<Fbx::Objects::Entity> go, std::shared_ptr<Entity> parent = nullptr);

    int AddComponent(std::shared_ptr<Component> component, std::shared_ptr<Fbx::Objects::Component> inComponent);

    void AddResource(const std::string& path);
};

class Component {
public:
    virtual std::string GetTypeName() = 0;
    virtual std::string ToJSON(std::shared_ptr<Context> context) = 0;
    int arrayId = -1;

    std::string GetJSON();

    void OnIterateTo(std::shared_ptr<Context> context);

protected:
    Component();

private:
    std::string json;
};

class Transform3D : public Component {
public:
    Transform3D(std::shared_ptr<Fbx::Objects::Transform> inTransform);
    virtual std::string GetTypeName() override;
    virtual std::string ToJSON(std::shared_ptr<Context> context) override;
    std::shared_ptr<Fbx::Objects::Transform> transform = nullptr;
};

class MeshRenderer : public Component {
public:
    MeshRenderer(std::shared_ptr<Fbx::Objects::MeshRenderer> meshRenderer);
    virtual std::string GetTypeName() override;
    virtual std::string ToJSON(std::shared_ptr<Context> context) override;
    std::shared_ptr<Fbx::Objects::MeshRenderer> meshRenderer = nullptr;
};

class SkinnedMeshRenderer : public Component {
public:
    SkinnedMeshRenderer(std::shared_ptr<Fbx::Objects::MeshRenderer> skinnedMeshRenderer);
    virtual std::string GetTypeName() override;
    virtual std::string ToJSON(std::shared_ptr<Context> context) override;
    std::shared_ptr<Fbx::Objects::MeshRenderer> skinnedMeshRenderer = nullptr;
};

class Skeleton : public Component {
public:
    Skeleton(std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton);
    virtual std::string GetTypeName() override;
    virtual std::string ToJSON(std::shared_ptr<Context> context) override;
    std::shared_ptr<Fbx::Assets::SkeletonAsset> skeleton = nullptr;
};

class Animation : public Component {
public:
    Animation(std::shared_ptr<Fbx::Objects::Animation> inAnimation);
    virtual std::string GetTypeName() override;
    virtual std::string ToJSON(std::shared_ptr<Context> context) override;
    std::shared_ptr<Fbx::Objects::Animation> animation = nullptr;
    int skeletonIndex;
};

void Converter3D(std::shared_ptr<Fbx::Objects::Entity> node, std::shared_ptr<Entity> entity, std::shared_ptr<Context> context);
void Converter3DMesh(std::shared_ptr<Fbx::Objects::Entity> node, std::shared_ptr<Entity> entity, std::shared_ptr<Context> context);

}  // namespace Exporter
