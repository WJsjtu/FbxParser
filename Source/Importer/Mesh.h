#pragma once
#include <fbxsdk.h>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#undef snprintf
#include "FbxParser.private.h"

namespace Fbx { namespace Importer {

/**
 * Container and importer for skinned mesh (FBX file) data
 **/
class MeshImportData {
public:
    struct MeshWedge {
        MeshWedge(uint32_t maxTexCoord = Configuration::MaxTexCoord);
        uint32_t iVertex;             // Vertex index.
        std::vector<glm::dvec2> uvs;  // UVs.
        glm::u8vec4 color;            // Vertex color.
    };

    struct MeshFace {
        // Textured Vertex indices.
        uint32_t iWedge[3];
        // Source Material (= texture plus unique flags) index.
        uint16_t meshMaterialIndex;

        glm::vec3 tangentX[3];
        glm::vec3 tangentY[3];
        glm::vec3 tangentZ[3];

        // 32-bit flag for smoothing groups.
        uint32_t smoothingGroups;
    };

    // A bone: an orientation, and a position, all relative to their parent.
    struct JointPosition {
        Maths::Transform transform;

        // For collision testing / debug drawing...
        float length;
        float xSize;
        float ySize;
        float zSize;
    };

    // Textured triangle.
    struct Triangle {
        // Point to three vertices in the vertex list.
        uint32_t wedgeIndex[3];
        // Materials can be anything.
        uint16_t materialIndex;
        // 32-bit flag for smoothing groups.
        uint32_t smoothingGroups;

        glm::vec3 tangentX[3];
        glm::vec3 tangentY[3];
        glm::vec3 tangentZ[3];

        Triangle& operator=(const Triangle& Other) {
            this->materialIndex = Other.materialIndex;
            this->smoothingGroups = Other.smoothingGroups;
            this->wedgeIndex[0] = Other.wedgeIndex[0];
            this->wedgeIndex[1] = Other.wedgeIndex[1];
            this->wedgeIndex[2] = Other.wedgeIndex[2];
            this->tangentX[0] = Other.tangentX[0];
            this->tangentX[1] = Other.tangentX[1];
            this->tangentX[2] = Other.tangentX[2];

            this->tangentY[0] = Other.tangentY[0];
            this->tangentY[1] = Other.tangentY[1];
            this->tangentY[2] = Other.tangentY[2];

            this->tangentZ[0] = Other.tangentZ[0];
            this->tangentZ[1] = Other.tangentZ[1];
            this->tangentZ[2] = Other.tangentZ[2];
            return *this;
        }
    };

    struct VertexInfluence {
        float weight;
        uint32_t vertexIndex;
        uint16_t boneIndex;
    };

    // Raw data material.
    struct RawMaterial {
        /** The actual material created on import or found among existing materials, this member is not serialize,
         * importer can found back the material */
        std::shared_ptr<Assets::MaterialAsset> material;
        /** The material name found by the importer */
        std::string materialImportName;
    };

    // Raw data bone.
    struct Bone {
        FbxNode* source;
        std::string name;
        int32_t numChildren;         // children // only needed in animation ?
        int32_t parentIndex;         // 0/NULL if this is the root bone.
        JointPosition bonePosition;  // reference position
    };

    // Raw data bone influence.
    struct RawBoneInfluence  // just weight, vertex, and Bone, sorted later....
    {
        float weight;
        int32_t vertexIndex;
        int32_t boneIndex;
    };

    // Vertex with texturing info, akin to Hoppe's 'Wedge' concept - import only.
    struct Vertex {
        Vertex(uint32_t maxTexCoord = Configuration::MaxTexCoord);
        uint32_t vertexIndex = 0;     // Index to a vertex.
        std::vector<glm::dvec2> uvs;  // Scaled to BYTES, rather...-> Done in digestion phase, on-disk size doesn't
        // matter here.
        glm::vec4 color;             // Vertex colors
        uint16_t materialIndex = 0;  // At runtime, this one will be implied by the face that's pointing to us.

        bool operator==(const Vertex& other) const {
            bool equal = true;

            equal &= (vertexIndex == other.vertexIndex);
            equal &= (materialIndex == other.materialIndex);
            equal &= (color == other.color);

            bool bUVsEqual = true;
            for (uint32_t UVIdx = 0; UVIdx < Configuration::MaxTexCoord; UVIdx++) {
                if (uvs[UVIdx] != other.uvs[UVIdx]) {
                    bUVsEqual = false;
                    break;
                }
            }

            equal &= bUVsEqual;

            return equal;
        }
    };

    // Points: regular FVectors (for now..)
    struct Point {
        glm::vec3 point;  // Change into packed integer later IF necessary, for 3x size reduction...
    };

public:
    std::string name = "";
    std::vector<MeshImportData::RawMaterial> materials;
    std::vector<glm::vec3> points;
    std::vector<MeshImportData::Vertex> wedges;
    std::vector<MeshImportData::Triangle> faces;
    std::vector<FbxNode*> sortedLinks;
    std::vector<MeshImportData::Bone> bones;
    std::vector<MeshImportData::RawBoneInfluence> influences;
    std::vector<int> pointToRawMap;  // Mapping from current point index to the original import point index
    uint32_t numTexCoords;           // The number of texture coordinate sets
    uint32_t maxMaterialIndex = 0;   // The max material index found on a triangle
    bool bHasVertexColors = false;   // If true there are vertex colors in the imported file
    bool bHasNormals = false;        // If true there are normals in the imported file
    bool bHasTangents = false;       // If true there are tangents in the imported file
    bool bUseT0AsRefPose = false;    // If true, then the pose at time=0 will be used instead of the ref pose
    bool bDiffPose = false;          // If true, one of the bones has a different pose at time=0 vs the ref pose

    // Morph targets imported(i.e. FBX) data. The name is the morph target name
    std::vector<MeshImportData> morphTargets;
    std::vector<std::set<uint32_t>> morphTargetModifiedPoints;
    std::vector<std::string> morphTargetNames;

    std::shared_ptr<Maths::BoxSphereBounds> boundingSphere = nullptr;
    std::shared_ptr<Maths::Box> boundingBox = nullptr;

    MeshImportData() : numTexCoords(0), maxMaterialIndex(0), bHasVertexColors(false), bHasNormals(false), bHasTangents(false), bUseT0AsRefPose(false), bDiffPose(false) {}

    /**
     * Removes all import data
     */
    void Empty() {
        materials.clear();
        points.clear();
        wedges.clear();
        faces.clear();
        bones.clear();
        sortedLinks.clear();
        influences.clear();
        pointToRawMap.clear();
    }
};
}}  // namespace Fbx::Importer