#pragma once
#include <vector>
#include "Importer/Mesh.h"
#include "FbxParser.h"
#include "FbxParser.private.h"
#include "fbxsdk.h"
#include "mikktspace.h"
#undef snprintf
namespace Fbx { namespace Builder {
struct MeshBuildSettings {
    bool bRecomputeNormals = true;
    bool bRecomputeTangents = true;

    /** If true, degenerate triangles will be removed. */
    bool bUseMikkTSpace = true;

    /** If true, we will use the surface area and the corner angle of the triangle as a ratio when computing the
     * normals. */
    bool bComputeWeightedNormals = true;

    /** If true, degenerate triangles will be removed. */
    bool bRemoveDegenerates = true;

    /** Required for PNT tessellation but can be slow. Recommend disabling for larger meshes. */
    bool bBuildAdjacencyBuffer = true;

    /** Threshold use to decide if two vertex position are equal. */
    float thresholdPosition = 0.00002f;

    /** Threshold use to decide if two normal, tangents or bi-normals are equal. */
    float thresholdTangentNormal = 0.00002f;

    /** Threshold use to decide if two UVs are equal. */
    float thresholdUV = 0.0009765625f;

    /** Threshold to compare vertex position equality when computing morph target deltas. */
    float morphThresholdPosition = 0.015f;

    /** Equality operator. */
    bool operator==(const MeshBuildSettings& Other) const;

    /** Inequality. */
    bool operator!=(const MeshBuildSettings& Other) const;
};

/**
 * Container to hold overlapping corners. For a vertex, lists all the overlapping vertices
 */
struct OverlappingCorners {
    OverlappingCorners() {}

    OverlappingCorners(const std::vector<glm::vec3>& inVertices, const std::vector<uint32_t>& inIndices, float comparisonThreshold);

    /* Resets, pre-allocates memory, marks all indices as not overlapping in preperation for calls to Add() */
    void Init(int numIndices);

    /* Add overlapping indices pair */
    void Add(int key, int value);

    /* Sorts arrays, converts sets to arrays for sorting and to allow simple iterating code, prevents additional adding
     */
    void FinishAdding();

    /**
     * @return array of sorted overlapping indices including input 'Key', empty array for indices that have no overlaps.
     */
    const std::vector<int>& FindIfOverlapping(int key) const;

private:
    std::vector<int> indexBelongsTo;
    std::vector<std::vector<int>> arrays;
    std::vector<std::set<int>> sets;
    std::vector<int> emptyArray;
    bool bFinishedAdding = false;
};

// Holder for skeletal data to be passed to MikkTSpace.
// Holds references to the wedge, face and points vectors that BuildSkeletalMesh is given.
// Holds reference to the calculated normals array, which will be fleshed out if they've been calculated.
// Holds reference to the newly created tangent and bitangent arrays, which MikkTSpace will fleshed out if required.
class MikkTSpace {
public:
    const std::vector<Importer::MeshImportData::MeshWedge>& wedges;  // Reference to wedge list.
    const std::vector<Importer::MeshImportData::MeshFace>& faces;    // Reference to face list.    Also contains normal/tangent/bitanget/UV coords for each vertex of the
                                                                     // face.
    const std::vector<glm::vec3>& points;                            // Reference to position list.
    bool bComputeNormals;                                            // Copy of bComputeNormals.
    std::vector<glm::vec3>& tangentsX;                               // Reference to newly created tangents list.
    std::vector<glm::vec3>& tangentsY;                               // Reference to newly created bitangents list.
    std::vector<glm::vec3>& tangentsZ;                               // Reference to computed normals, will be empty otherwise.

    MikkTSpace(const std::vector<Importer::MeshImportData::MeshWedge>& wedges, const std::vector<Importer::MeshImportData::MeshFace>& faces, const std::vector<glm::vec3>& points, bool bInComputeNormals, std::vector<glm::vec3>& vertexTangentsX, std::vector<glm::vec3>& vertexTangentsY,
               std::vector<glm::vec3>& vertexTangentsZ);
};

class IMeshBuildInputData {
public:
    IMeshBuildInputData(const std::vector<Importer::MeshImportData::MeshWedge>& inWedges, const std::vector<Importer::MeshImportData::MeshFace>& inFaces, const std::vector<glm::vec3>& inPoints, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences,
                        const std::vector<int>& inPointToOriginalMap, const MeshBuildSettings& inBuildOptions);

    uint32_t GetWedgeIndex(uint32_t faceIndex, uint32_t triIndex);

    uint32_t GetVertexIndex(uint32_t wedgeIndex);

    uint32_t GetVertexIndex(uint32_t faceIndex, uint32_t triIndex);

    glm::vec3 GetVertexPosition(uint32_t wedgeIndex);

    glm::vec3 GetVertexPosition(uint32_t faceIndex, uint32_t triIndex);

    glm::dvec2 GetVertexUV(uint32_t faceIndex, uint32_t triIndex, uint32_t UVIndex);

    uint32_t GetFaceSmoothingGroups(uint32_t faceIndex);

    uint32_t GetNumFaces();

    uint32_t GetNumWedges();

    SMikkTSpaceInterface* GetMikkTInterface();

    void* GetMikkTUserData();

    void ValidateTangentArraySize();

public:
    const MeshBuildSettings& buildOptions;

    // process temparory data structure
    std::vector<glm::vec3> tangentX;
    std::vector<glm::vec3> tangentY;
    std::vector<glm::vec3> tangentZ;

    SMikkTSpaceInterface mikkTInterface;
    MikkTSpace mikkTUserData;

    const std::vector<Importer::MeshImportData::MeshWedge>& wedges;
    const std::vector<Importer::MeshImportData::MeshFace>& faces;
    const std::vector<glm::vec3>& points;
    const std::vector<Importer::MeshImportData::VertexInfluence>& influences;
    const std::vector<int>& pointToOriginalMap;
};

struct SubMeshVertex {
    glm::vec3 position;
    glm::vec3 tangentX;             // Tangent, U-direction
    glm::vec3 tangentY;             // Binormal, V-direction
    glm::vec3 tangentZ;             // Normal
    glm::dvec2 UVs[MAX_TEXCOORDS];  // UVs
    glm::vec4 color;                // VertexColor
    uint16_t influenceBones[MAX_TOTAL_INFLUENCES];
    float influenceWeights[MAX_TOTAL_INFLUENCES];
};

struct SubMeshVertexWithWedgeIdx : public SubMeshVertex {
    uint32_t pointWedgeIdx;
};

class SubMeshAsset {
public:
    SubMeshAsset() : materialIndex(0), baseIndex(0), numTriangles(0), baseVertexIndex(0), numVertices(0), originalDataSectionIndex(-1), chunkedParentSectionIndex(-1) {}

public:
    /** Material (texture) used for this section. */
    uint16_t materialIndex;

    /** The offset of this section's indices in the LOD's index buffer. */
    uint32_t baseIndex;

    /** The number of triangles in this section. */
    uint32_t numTriangles;

    /** The offset into the LOD's vertex buffer of this section's vertices. */
    uint32_t baseVertexIndex;

    /** The soft vertices of this section. */
    std::vector<SubMeshVertex> vertices;

    /** Number of vertices in this section (size of SoftVertices array). Available in non-editor builds. */
    int numVertices;

    /*
     * This represent the original section index in the imported data. The original data is chunk per material,
     * we use this index to store user section modification. The user cannot change a BONE chunked section data,
     * since the BONE chunk can be per-platform. Do not use this value to index the Sections array, only the user
     * section data should be index by this value.
     */
    int originalDataSectionIndex;

    /*
     * If this section was produce because of BONE chunking, the parent section index will be valid.
     * If the section is not the result of skin vertex chunking, this value will be INDEX_NONE.
     * Use this value to know if the section was BONE chunked:
     * if(ChunkedParentSectionIndex != INDEX_NONE) will be true if the section is BONE chunked
     */
    int chunkedParentSectionIndex;
};

template <class T = SubMeshAsset>
class MeshAsset {
    static_assert(std::is_base_of<SubMeshAsset, T>::value, "MeshAsset sections must derive from SubMeshAsset");

public:
    /** Sections. */
    std::vector<T> sections;

    uint32_t numVertices;

    /** Index buffer, covering all sections */
    std::vector<uint32_t> indexBuffer;

    /** Mapping from final mesh vertex index to raw import vertex index. Needed for vertex animation, which only stores
     * positions for import verts. */
    std::vector<int> meshToImportVertexMap;

    /** The max index in MeshToImportVertexMap, ie. the number of imported (raw) verts. */
    int maxImportVertex;

    /** Imported raw mesh data. Optional, only the imported mesh LOD has this, generated LOD or old asset will be null.
     */
    std::vector<uint32_t> rawPointIndices;
};

template <class T = SubMeshAsset, class S = MeshAsset<T>>
class Mesh {
    static_assert(std::is_base_of<MeshAsset<T>, S>::value, "Mesh must derive from MeshAsset");

public:
    FbxMesh* fbxMesh;
    std::string name;
    std::vector<std::shared_ptr<Assets::MaterialAsset>> materials;
    bool bHasVertexColors = false;  // If true there are vertex colors in the imported file
    bool bHasNormals = false;       // If true there are normals in the imported file
    bool bHasTangents = false;      // If true there are tangents in the imported file
    uint32_t numTexCoords;          // The number of texture coordinate sets
    uint32_t maxMaterialIndex;      // The max material index found on a triangle
    std::shared_ptr<Maths::BoxSphereBounds> boundingSphere;
    std::shared_ptr<Maths::Box> boundingBox;
    std::shared_ptr<S> asset;
};

class OriginSubMesh {
public:
    /** The material index with which this chunk should be rendered. */
    int materialIndex = 0;
    /** The original section index for which this chunk was generated. */
    int originalSectionIndex = 0;
    /** The vertices associated with this chunk. */
    std::vector<SubMeshVertexWithWedgeIdx> vertices;
    /** The indices of the triangles in this chunk. */
    std::vector<uint32_t> indices;
    /** The parent original section index for which this chunk was generated. INDEX_NONE for parent the value of the
     * parent for BONE child chunked*/
    int parentChunkSectionIndex = -1;
};

class MeshBuildInputData : public IMeshBuildInputData {
public:
    MeshBuildInputData(MeshAsset<>& inModel, const std::vector<Importer::MeshImportData::VertexInfluence>& inInfluences, const std::vector<Importer::MeshImportData::MeshWedge>& inWedges, const std::vector<Importer::MeshImportData::MeshFace>& inFaces, const std::vector<glm::vec3>& inPoints,
                       const std::vector<int>& inPointToOriginalMap, const MeshBuildSettings& inBuildOptions);

    std::vector<std::shared_ptr<OriginSubMesh>> chunks;

    MeshAsset<>& model;
};

class MeshBuilder {
public:
    bool Build(std::shared_ptr<Mesh<>> mesh, std::shared_ptr<Importer::MeshImportData> meshImportData, const MeshBuildSettings& options);

    bool BuildMesh(MeshAsset<>& model, const std::string& meshName, const std::vector<Importer::MeshImportData::VertexInfluence>& influences, const std::vector<Importer::MeshImportData::MeshWedge>& wedges, const std::vector<Importer::MeshImportData::MeshFace>& faces,
                   const std::vector<glm::vec3>& points, const std::vector<int>& pointToOriginalMap, const MeshBuildSettings& buildOptions);

private:
    bool GenerateRenderableMesh(MeshBuildInputData& inBuildData);

    std::vector<std::shared_ptr<OverlappingCorners>> overlappingCorners;
};

std::shared_ptr<Assets::MeshAsset> ConvertToMesh(std::shared_ptr<Mesh<>> mesh);
}}  // namespace Fbx::Builder