#include <fstream>
#include <iostream>
#include <unordered_map>
#include "Resource.h"
#include <glm/vec4.hpp>  // glm::vec4
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>  // glm::mat4

namespace Exporter {

SkinnedMesh::SkinnedMesh(std::shared_ptr<Fbx::Assets::MeshAsset> inSkinnedMesh, std::shared_ptr<Fbx::Assets::SkeletonAsset> inSkeleton) : Resource(""), mesh(inSkinnedMesh), skeleton(inSkeleton) {}

std::string SkinnedMesh::GetHash() { return mesh->uniqueID; }
std::string SkinnedMesh::GetExportPath() {
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
std::string SkinnedMesh::ExportResource() {
    if (!skeleton) {
        return "";
    }

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
    size += 4 * 4;
    layouts.push_back("BLENDWEIGHT");
    size += 4 * 4;
    layouts.push_back("BLENDINDICES");
    if (mesh->bHasTangents) {
        size += 4 * 4;
        layouts.push_back("TANGENT");
    }

    uint64_t vertexStart = 0;
    uint64_t vertexLength = 0;
    uint64_t triangleCount = 0;

    for (auto& submesh : mesh->subMeshes) {
        vertexLength += static_cast<uint64_t>(size) * static_cast<uint64_t>(submesh->vertices.size());
    }

    uint64_t indiceStart = vertexStart + vertexLength;
    uint64_t indiceLength = static_cast<uint64_t>(mesh->indexBuffer.size()) * (mesh->indiceFormat == Fbx::Assets::EMeshIndiceFormat::Bit16 ? 2 : 4);

    uint64_t bonePoseStart = indiceStart + indiceLength;

    if ((bonePoseStart % 4) != 0) {
        bonePoseStart += 4 - (bonePoseStart % 4);
    }

    uint64_t bonePoseLength = static_cast<uint64_t>(mesh->activeBoneIndices.size()) * 4 * 4 * 4;

    uint64_t bufferLength = bonePoseStart + bonePoseLength;

    char* buffer = static_cast<char*>(malloc(bufferLength * sizeof(char)));

    memset(buffer, 0, bufferLength);

    std::unordered_map<uint16_t, uint16_t> boneLocalIndexToGlobalIndex;

    for (int i = 0; i < mesh->activeBoneIndices.size(); i++) {
        boneLocalIndexToGlobalIndex[mesh->activeBoneIndices[i]] = i;
    }

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
            auto boneWeights = submesh->vertices[i]->boneWeight;
            auto boneIndices = submesh->vertices[i]->boneIndex;
            while (boneWeights.size() < 4) {
                boneWeights.push_back(0);
                boneIndices.push_back(0);
            }
            if (boneIndices.size() > 4) {
                LOG(Fbx::Utils::ELogLevel::Warn, "Vertex influence is more than 4.");
                float weight = 0.0f;
                for (int i = 5; i < boneIndices.size(); i++) {
                    weight += boneWeights[i];
                }
                boneWeights[0] += weight;
            }

            {
                *current = boneWeights[0];
                current++;
                *current = boneWeights[1];
                current++;
                *current = boneWeights[2];
                current++;
                *current = boneWeights[3];
                current++;
            }
            {
                ASSERT(boneLocalIndexToGlobalIndex.find(boneIndices[0]) != boneLocalIndexToGlobalIndex.end());
                *current = boneLocalIndexToGlobalIndex[boneIndices[0]];
                current++;
                ASSERT(boneLocalIndexToGlobalIndex.find(boneIndices[1]) != boneLocalIndexToGlobalIndex.end());
                *current = boneLocalIndexToGlobalIndex[boneIndices[1]];
                current++;
                ASSERT(boneLocalIndexToGlobalIndex.find(boneIndices[2]) != boneLocalIndexToGlobalIndex.end());
                *current = boneLocalIndexToGlobalIndex[boneIndices[2]];
                current++;
                ASSERT(boneLocalIndexToGlobalIndex.find(boneIndices[3]) != boneLocalIndexToGlobalIndex.end());
                *current = boneLocalIndexToGlobalIndex[boneIndices[3]];
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

    current = (float*)(buffer + bonePoseStart);

    for (int i = 0; i < mesh->activeBoneIndices.size(); i++) {
        auto& mat = mesh->boneInverseMatrices[mesh->activeBoneIndices[i]];
        for (int j = 0; j < 16; j++) {
            *current = mat.data[j];
            current++;
        }
    }

    ASSERT(((char*)current - buffer) == bufferLength);

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
        writer.String("bonePoseStart");
        writer.Uint64(bonePoseStart);
        writer.String("bonePoseLength");
        writer.Uint64(bonePoseLength);

        ASSERT(skeleton->rootBone);

        glm::mat4 rootBoneInverse;
        bool rootBoneInverseFound = false;

        for (int i = 0; i < skeleton->bones.size(); i++) {
            if (skeleton->bones[i] == skeleton->rootBone) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) {
                        rootBoneInverse[j][k] = mesh->boneInverseMatrices[i].data[4 * j + k];
                    }
                }
                rootBoneInverseFound = true;
                break;
            }
        }
        ASSERT(rootBoneInverseFound);
        glm::vec4 boundingSphereOrigin = rootBoneInverse * glm::vec4(mesh->boundingSphere->center.x, mesh->boundingSphere->center.y, mesh->boundingSphere->center.z, 1);

        writer.String("capsule");
        writer.StartObject();
        writer.String("x");
        writer.Double(boundingSphereOrigin.x);
        writer.String("y");
        writer.Double(boundingSphereOrigin.y);
        writer.String("z");
        writer.Double(boundingSphereOrigin.z);
        writer.String("radius");
        writer.Double(mesh->boundingSphere->radius);
        writer.EndObject();

        glm::vec4 boundingBoxOrigin = rootBoneInverse * glm::vec4(mesh->boundingBox->center.x, mesh->boundingBox->center.y, mesh->boundingBox->center.z, 1);

        writer.String("boundBox");
        writer.StartObject();
        writer.String("center");
        writer.StartArray();
        writer.Double(boundingBoxOrigin.x);
        writer.Double(boundingBoxOrigin.y);
        writer.Double(boundingBoxOrigin.z);
        writer.EndArray();
        writer.String("size");
        writer.StartArray();
        writer.Double(mesh->boundingBox->extents.x);
        writer.Double(mesh->boundingBox->extents.y);
        writer.Double(mesh->boundingBox->extents.z);
        writer.EndArray();
        writer.EndObject();

        writer.String("rootBone");
        JSON_STRING_VALUE(writer, skeleton->rootBone->path);

        writer.String("bones");
        writer.StartArray();
        for (int i = 0; i < mesh->activeBoneIndices.size(); i++) {
            JSON_STRING_VALUE(writer, skeleton->bones[mesh->activeBoneIndices[i]]->path);
        }
        writer.EndArray();

        writer.String("subMeshs");
        writer.StartArray();
        for (int i = 0; i < mesh->subMeshes.size(); i++) {
            auto& subMesh = mesh->subMeshes[i];
            writer.StartObject();
            writer.String("start");
            writer.Uint64(subMesh->baseIndex);
            writer.String("length");
            writer.Uint64(subMesh->numTriangles * 3);
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
std::string SkinnedMesh::GetResourceType() { return "mesh"; }

}  // namespace Exporter