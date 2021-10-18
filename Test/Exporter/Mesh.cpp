#include "Resource.h"
namespace Exporter {

Mesh::Mesh(std::shared_ptr<Fbx::Assets::MeshAsset> inMesh) : Resource(""), mesh(inMesh) {}

std::string Mesh::GetHash() { return mesh->uniqueID; }
std::string Mesh::GetExportPath() {
    std::string savePath = mesh->name + ".mesh";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = mesh->name + "_" + std::to_string(++renameCount) + ".mesh";
        }
    }
    return savePath;
}
std::string Mesh::ExportResource() {
    std::string avatarPath;

    int size = 4 * 3;
    std::vector<std::string> layouts = {"POSITION"};
    if (mesh->bHasNormals) {
        size += 4 * 3;
        layouts.push_back("NORMAL");
    }
    if (mesh->bHasVertexColors) {
        size += 4 * 4;
        layouts.push_back("COLOR");
    }
    if (mesh->numTexCoords > 0) {
        size += 4 * 2;
        layouts.push_back("UV");
    }
    if (mesh->bHasTangents) {
        size += 4 * 4;
        layouts.push_back("TANGENT");
    }

    uint64_t vertexStart = 0;
    uint64_t vertexLength = 0;

    for (auto& submesh : mesh->subMeshes) {
        vertexLength += static_cast<uint64_t>(size) * static_cast<uint64_t>(submesh->vertices.size());
    }

    uint64_t indiceStart = vertexStart + vertexLength;
    uint64_t indiceLength = static_cast<uint64_t>(mesh->indexBuffer.size()) * (mesh->indiceFormat == Fbx::Assets::EMeshIndiceFormat::Bit16 ? 2 : 4);

    uint64_t bufferLength;
    {
        uint64_t bonePoseStart = indiceStart + indiceLength;

        if ((bonePoseStart % 4) != 0) {
            bonePoseStart += 4 - (bonePoseStart % 4);
        }
        bufferLength = bonePoseStart;
    }

    char* buffer = static_cast<char*>(malloc(bufferLength * sizeof(char)));

    memset(buffer, 0, bufferLength);

    float* current = (float*)buffer;
    for (auto& submesh : mesh->subMeshes) {
        for (uint64_t i = 0; i < submesh->vertices.size(); i++) {
            *current = submesh->vertices[i]->position.x;
            current++;
            *current = submesh->vertices[i]->position.y;
            current++;
            *current = submesh->vertices[i]->position.z;
            current++;
            if (mesh->bHasNormals) {
                *current = submesh->vertices[i]->normal.x;
                current++;
                *current = submesh->vertices[i]->normal.y;
                current++;
                *current = submesh->vertices[i]->normal.z;
                current++;
            }
            if (mesh->bHasVertexColors) {
                *current = submesh->vertices[i]->color.x;
                current++;
                *current = submesh->vertices[i]->color.y;
                current++;
                *current = submesh->vertices[i]->color.z;
                current++;
                *current = submesh->vertices[i]->color.w;
                current++;
            }
            if (mesh->numTexCoords > 0) {
                *current = (float)(submesh->vertices[i]->uv[0].x);
                current++;
                *current = (float)(submesh->vertices[i]->uv[0].y);
                current++;
            }
            if (mesh->bHasTangents) {
                *current = submesh->vertices[i]->tangent.x;
                current++;
                *current = submesh->vertices[i]->tangent.y;
                current++;
                *current = submesh->vertices[i]->tangent.z;
                current++;
                *current = 1;
                current++;
            }
        }
    }

    ASSERT(((char*)current - buffer) == indiceStart);

    for (uint64_t i = 0; i < mesh->indexBuffer.size(); i++) {
        if (mesh->indiceFormat == Fbx::Assets::EMeshIndiceFormat::Bit16) {
            uint16_t* _current = (uint16_t*)current;
            *_current = mesh->indexBuffer[(i % 3) == 0 ? i : ((i % 3) == 1 ? i + 1 : i - 1)];
            _current++;
            current = (float*)_current;
        } else {
            uint32_t* _current = (uint32_t*)current;
            *_current = mesh->indexBuffer[(i % 3) == 0 ? i : ((i % 3) == 1 ? i + 1 : i - 1)];
            _current++;
            current = (float*)_current;
        }
    }

    std::vector<char> content(buffer, buffer + bufferLength);
    free(buffer);

    auto binFile = std::make_shared<MeshBinAssetFile>("", mesh->name, content);
    std::string binFilePath = AddFile(binFile);

    std::string meshJSON;
    {
        rapidjson::StringBuffer sb;
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
        writer.StartObject();

        writer.String("file");
        writer.StartObject();
        writer.String("src");
        JSON_STRING_VALUE(writer, binFilePath);
        writer.EndObject();

        writer.String("data");
        writer.StartObject();

        writer.String("indiceFormat");
        JSON_STRING_VALUE(writer, std::string(mesh->indiceFormat == Fbx::Assets::EMeshIndiceFormat::Bit16 ? "BIT16" : "BIT32"));
        writer.String("vertexLayout");
        JSON_STRING_VALUE(writer, Join(layouts, ","));
        writer.String("vertexStart");
        writer.Uint64(vertexStart);
        writer.String("vertexLength");
        writer.Uint64(vertexLength);
        writer.String("indiceStart");
        writer.Uint64(indiceStart);
        writer.String("indiceLength");
        writer.Uint64(indiceLength);

        writer.String("capsule");
        writer.StartObject();
        writer.String("x");
        writer.Double(mesh->boundingSphere->center.x);
        writer.String("y");
        writer.Double(mesh->boundingSphere->center.y);
        writer.String("z");
        writer.Double(mesh->boundingSphere->center.z);
        writer.String("radius");
        writer.Double(mesh->boundingSphere->radius);
        writer.EndObject();

        writer.String("boundBox");
        writer.StartObject();
        writer.String("center");
        writer.StartArray();
        writer.Double(mesh->boundingBox->center.x);
        writer.Double(mesh->boundingBox->center.y);
        writer.Double(mesh->boundingBox->center.z);
        writer.EndArray();
        writer.String("size");
        writer.StartArray();
        writer.Double(mesh->boundingBox->extents.x);
        writer.Double(mesh->boundingBox->extents.y);
        writer.Double(mesh->boundingBox->extents.z);
        writer.EndArray();
        writer.EndObject();

        writer.String("subMeshs");
        writer.StartArray();
        for (int i = 0; i < mesh->subMeshes.size(); i++) {
            auto& submesh = mesh->subMeshes[i];
            writer.StartObject();
            writer.String("start");
            writer.Uint64(submesh->baseIndex);
            writer.String("length");
            writer.Uint64(submesh->numTriangles * 3);
            writer.EndObject();
        }
        writer.EndArray();

        writer.EndObject();
        writer.EndObject();
        meshJSON = sb.GetString();
#ifdef _DEBUG
        PRETTIFY_JSON(meshJSON);
#endif
    }
    return meshJSON;
}
std::string Mesh::GetResourceType() { return "mesh"; }

}  // namespace Exporter