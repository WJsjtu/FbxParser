#include "FbxParser.h"
#include "FbxParser.private.h"
#include <random>
#include <filesystem>
#include "Importer/Scene.h"

namespace Fbx {
namespace Configuration {
uint32_t MaxTexCoord = 4;
uint32_t MaxFrameRate = 120;
uint32_t DefaultFrameRate = 60;
float VectorComparsionThreshold = 1.e-12f;
float PointComparsionThreshold = 1.e-18f;
float UVComparsionThreshold = 0.000009765625f;
}  // namespace Configuration

AssertException::AssertException(const std::string& msg) : message(msg) {}

const char* AssertException::what() const throw() { return message.c_str(); }

namespace Types {

Vector2::Vector2(float inX, float inY) : x(inX), y(inY) {}

Vector3::Vector3(float inX, float inY, float inZ) : x(inX), y(inY), z(inZ) {}

Vector4::Vector4(float inX, float inY, float inZ, float inW) : x(inX), y(inY), z(inZ), w(inW) {}

Vector2& ConvertFromGLM(Vector2& vec1, const glm::vec2& vec2) {
    vec1.x = vec2.x;
    vec1.y = vec2.y;
    return vec1;
}

Vector3& ConvertFromGLM(Vector3& vec1, const glm::vec3& vec2) {
    vec1.x = vec2.x;
    vec1.y = vec2.y;
    vec1.z = vec2.z;
    return vec1;
}

Vector4& ConvertFromGLM(Vector4& vec1, const glm::vec4& vec2) {
    vec1.x = vec2.x;
    vec1.y = vec2.y;
    vec1.z = vec2.z;
    vec1.w = vec2.w;
    return vec1;
}

Vector4& ConvertFromGLM(Vector4& vec1, const glm::quat& vec2) {
    vec1.x = vec2.x;
    vec1.y = vec2.y;
    vec1.z = vec2.z;
    vec1.w = vec2.w;
    return vec1;
}

Matrix4& ConvertFromGLM(Matrix4& mat1, const glm::mat4& mat2) {
    for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
            mat1.data[4 * j + k] = mat2[j][k];
        }
    }
    return mat1;
}
}  // namespace Types

namespace Objects {

int ComponentID = 0;

Component::Component() : id(std::to_string(ComponentID++)) {}

EComponentType Component::GetTypeName() { return EComponentType::Component; }

EComponentType Transform::GetTypeName() { return EComponentType::Transform; }

EComponentType Skeleton::GetTypeName() { return EComponentType::Skeleton; }

EComponentType MeshRenderer::GetTypeName() { return EComponentType::MeshRenderer; }

EComponentType Animation::GetTypeName() { return EComponentType::Animation; }

}  // namespace Objects

namespace Utils {

std::function<void(ELogLevel, std::string)> GlobalLogger = [](ELogLevel, std::string) {};

DefaultLogInstance::DefaultLogInstance(const std::string& logFilePath) {
    auto stdcout_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
    stdcout_sink->set_level(spdlog::level::debug);
    stdcout_sink->set_pattern("[%Y-%m-%d %H:%M:%S][thread %t][%^%l%$] %v");

    auto stdcerr_sink = std::make_shared<spdlog::sinks::stderr_sink_mt>();
    stdcerr_sink->set_level(spdlog::level::err);
    stdcerr_sink->set_pattern("[%Y-%m-%d %H:%M:%S][thread %t][%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logFilePath, 16 * 1024 * 1024, 0);
    file_sink->set_level(spdlog::level::debug);
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S][thread %t][%^%l%$] %v");

    logger = std::shared_ptr<spdlog::logger>{new spdlog::logger("multi_sink", {stdcout_sink, stdcerr_sink, file_sink})};
    logger->set_level(spdlog::level::debug);
}

void DefaultLogInstance::Log(ELogLevel level, std::string message) {
    if (!logger) {
        return;
    }
    switch (level) {
        case Fbx::Utils::ELogLevel::Info: logger->info(message); break;
        case Fbx::Utils::ELogLevel::Error: logger->error(message); break;
        case Fbx::Utils::ELogLevel::Warn: logger->warn(message); break;
        case Fbx::Utils::ELogLevel::Critical: logger->critical(message); break;
        case Fbx::Utils::ELogLevel::Debug: logger->debug(message); break;
        default: break;
    }
}

std::map<std::string, std::shared_ptr<DefaultLogInstance>> Loggers;

std::function<void(ELogLevel, std::string)> GetDefaultLogger(const std::string& logFilePath) {
    if (Loggers.find(logFilePath) == Loggers.end()) {
        Loggers.emplace(logFilePath, std::make_shared<DefaultLogInstance>(logFilePath));
    }
    return std::bind(&DefaultLogInstance::Log, Loggers[logFilePath], std::placeholders::_1, std::placeholders::_2);
}

std::function<void(ELogLevel, std::string)> GetGlobalLogger() { return GlobalLogger; }

void SetGlobalLogger(std::function<void(ELogLevel, std::string)> logger) { GlobalLogger = logger; }

}  // namespace Utils

namespace Maths {

float TruncToFloat(float f) { return truncf(f); }

template <>
bool IsNearlyEqual<glm::vec3>(const glm::vec3& A, const glm::vec3& B, float errorTolerance) {
    return (Maths::Abs<float>(A.x - B.x) <= errorTolerance) && (Maths::Abs<float>(A.y - B.y) <= errorTolerance) && (Maths::Abs<float>(A.z - B.z) <= errorTolerance);
}

template <>
bool IsNearlyEqual<glm::quat>(const glm::quat& A, const glm::quat& B, float errorTolerance) {
    return (Maths::Abs<float>(A.x - B.x) <= errorTolerance) && (Maths::Abs<float>(A.y - B.y) <= errorTolerance) && (Maths::Abs<float>(A.z - B.z) <= errorTolerance) && (Maths::Abs<float>(A.w - B.w) <= errorTolerance);
}

template <>
bool IsNearlyZero<glm::vec3>(const glm::vec3& vec, float tolerance) {
    return Maths::Abs(vec.x) <= tolerance && Maths::Abs(vec.y) <= tolerance && Maths::Abs(vec.z) <= tolerance;
}

bool ContainsNaN(const glm::vec3& vec) { return glm::isnan(vec.x) || glm::isnan(vec.y) || glm::isnan(vec.z); }

bool ContainsNaN(const glm::quat& vec) { return glm::isnan(vec.x) || glm::isnan(vec.y) || glm::isnan(vec.z) || glm::isnan(vec.w); }

/**
 * Returns signed fractional part of a float.
 * @param Value	Floating point value to convert
 * @return		A float between >=0 and < 1 for nonnegative input. A float between >= -1 and < 0 for negative
 * input.
 */
float Fractional(float value) { return value - TruncToFloat(value); }

int RoundToInt(double value) { return (int)(round(value)); }

// Use the Euclidean method to find the GCD
int GreatestCommonDivisor(int a, int b) {
    while (b != 0) {
        int t = b;
        b = a % b;
        a = t;
    }
    return a;
}

// LCM = a/gcd * b
// a and b are the number we want to find the lcm
int LeastCommonMultiplier(int a, int b) {
    int currentGcd = GreatestCommonDivisor(a, b);
    return currentGcd == 0 ? 0 : (a / currentGcd) * b;
}

std::random_device dev;
std::mt19937 rng(dev());

float Rand() {
    std::uniform_real_distribution<float> dist(0.0, 1.0);
    return dist(rng);
}

float Acos(float value) { return acosf((value < -1.f) ? -1.f : ((value < 1.f) ? value : 1.f)); }

void GetMatrixScaledAxes(glm::mat4 mat, glm::vec3& x, glm::vec3& y, glm::vec3& z) {
    x.x = mat[0][0];
    x.y = mat[0][1];
    x.z = mat[0][2];
    y.x = mat[1][0];
    y.y = mat[1][1];
    y.z = mat[1][2];
    z.x = mat[2][0];
    z.y = mat[2][1];
    z.z = mat[2][2];
}

glm::vec3 Transform::ExtractScaling(glm::mat4& mat, float tolerance) {
    glm::vec3 scale3D(0, 0, 0);

    // For each row, find magnitude, and if its non-zero re-scale so its unit length.
    const float squareSum0 = (mat[0][0] * mat[0][0]) + (mat[0][1] * mat[0][1]) + (mat[0][2] * mat[0][2]);
    const float squareSum1 = (mat[1][0] * mat[1][0]) + (mat[1][1] * mat[1][1]) + (mat[1][2] * mat[1][2]);
    const float squareSum2 = (mat[2][0] * mat[2][0]) + (mat[2][1] * mat[2][1]) + (mat[2][2] * mat[2][2]);

    if (squareSum0 > tolerance) {
        float Scale0 = sqrt(squareSum0);
        scale3D[0] = Scale0;
        float InvScale0 = 1.f / Scale0;
        mat[0][0] *= InvScale0;
        mat[0][1] *= InvScale0;
        mat[0][2] *= InvScale0;
    } else {
        scale3D[0] = 0;
    }

    if (squareSum1 > tolerance) {
        float Scale1 = sqrt(squareSum1);
        scale3D[1] = Scale1;
        float InvScale1 = 1.f / Scale1;
        mat[1][0] *= InvScale1;
        mat[1][1] *= InvScale1;
        mat[1][2] *= InvScale1;
    } else {
        scale3D[1] = 0;
    }

    if (squareSum2 > tolerance) {
        float Scale2 = sqrt(squareSum2);
        scale3D[2] = Scale2;
        float InvScale2 = 1.f / Scale2;
        mat[2][0] *= InvScale2;
        mat[2][1] *= InvScale2;
        mat[2][2] *= InvScale2;
    } else {
        scale3D[2] = 0;
    }

    return scale3D;
}

void Transform::SetFromMatrix(const glm::mat4& inMatrix) {
    glm::mat4 mat = inMatrix;

    // Get the 3D scale from the matrix
    scale = ExtractScaling(mat);

    // If there is negative scaling going on, we handle that here
    if (glm::determinant(inMatrix) < 0.f) {
        // Assume it is along X and modify transform accordingly.
        // It doesn't actually matter which axis we choose, the 'appearance' will be the same
        scale.x *= -1;
    }

    rotation = glm::quat(mat);
    translation = glm::vec3(inMatrix[3][0], inMatrix[3][1], inMatrix[3][2]);

    // Normalize rotation
    rotation = glm::normalize(rotation);
}

glm::mat4 Transform::ToMatrixWithScale() const {
    glm::mat4 r = glm::toMat4(rotation);
    glm::mat4 result = r;
    result[0][0] *= scale.x;
    result[0][1] *= scale.x;
    result[0][2] *= scale.x;

    result[1][0] *= scale.y;
    result[1][1] *= scale.y;
    result[1][2] *= scale.y;

    result[2][0] *= scale.z;
    result[2][1] *= scale.z;
    result[2][2] *= scale.z;

    result[3][0] = translation.x;
    result[3][1] = translation.y;
    result[3][2] = translation.z;

    result[0][3] = result[1][3] = result[2][3] = 0;
    result[3][3] = 1;
    return result;
}

void Transform::NormalizeRotation() {
    rotation = glm::normalize(rotation);
    ASSERT(!Maths::ContainsNaN(rotation));
}

Box::Box() {
    min = max = glm::vec3(0, 0, 0);
    bIsValid = false;
}

Box::Box(const glm::vec3& inMin, const glm::vec3& inMax) : min(inMin), max(inMax), bIsValid(1) {}

Box::Box(const std::vector<glm::vec3>& points) {
    min = max = glm::vec3(0, 0, 0);
    bIsValid = true;
    for (int i = 0; i < points.size(); i++) {
        *this += points[i];
    }
}

bool Box::operator==(const Box& other) const { return (min == other.min) && (max == other.max); }

Box& Box::operator+=(const glm::vec3& other) {
    if (bIsValid) {
        min.x = Maths::Min(min.x, other.x);
        min.y = Maths::Min(min.y, other.y);
        min.z = Maths::Min(min.z, other.z);

        max.x = Maths::Max(max.x, other.x);
        max.y = Maths::Max(max.y, other.y);
        max.z = Maths::Max(max.z, other.z);
    } else {
        min = max = other;
        bIsValid = true;
    }

    return *this;
}

Box Box::operator+(const glm::vec3& other) const { return Box(*this) += other; }

Box Box::operator+(const Box& other) const { return Box(*this) += other; }

Box& Box::operator+=(const Box& other) {
    if (bIsValid && other.bIsValid) {
        min.x = Maths::Min(min.x, other.min.x);
        min.y = Maths::Min(min.y, other.min.y);
        min.z = Maths::Min(min.z, other.min.z);

        max.x = Maths::Max(max.x, other.max.x);
        max.y = Maths::Max(max.y, other.max.y);
        max.z = Maths::Max(max.z, other.max.z);
    } else if (other.bIsValid) {
        *this = other;
    }

    return *this;
}

glm::vec3& Box::operator[](int index) {
    ASSERT((index >= 0) && (index < 2));

    if (index == 0) {
        return min;
    }

    return max;
}

Box Box::ExpandBy(float w) const { return Box(min - glm::vec3(w, w, w), max + glm::vec3(w, w, w)); }

Box Box::ExpandBy(const glm::vec3& v) const { return Box(min - v, max + v); }

Box Box::ExpandBy(const glm::vec3& neg, const glm::vec3& pos) const { return Box(min - neg, max + pos); }

Box Box::ShiftBy(const glm::vec3& offset) const { return Box(min + offset, max + offset); }

Box Box::MoveTo(const glm::vec3& destination) const {
    const glm::vec3 Offset = destination - GetCenter();
    return Box(min + Offset, max + Offset);
}

glm::vec3 Box::GetCenter() const { return glm::vec3((min + max) * 0.5f); }

void Box::GetCenterAndExtents(glm::vec3& center, glm::vec3& extents) const {
    extents = GetExtent();
    center = min + extents;
}

glm::vec3 Box::GetClosestPointTo(const glm::vec3& point) const {
    // start by considering the point inside the box
    glm::vec3 ClosestPoint = point;

    // now clamp to inside box if it's outside
    if (point.x < min.x) {
        ClosestPoint.x = min.x;
    } else if (point.x > max.x) {
        ClosestPoint.x = max.x;
    }

    // now clamp to inside box if it's outside
    if (point.y < min.y) {
        ClosestPoint.y = min.y;
    } else if (point.y > max.y) {
        ClosestPoint.y = max.y;
    }

    // Now clamp to inside box if it's outside.
    if (point.z < min.z) {
        ClosestPoint.z = min.z;
    } else if (point.z > max.z) {
        ClosestPoint.z = max.z;
    }

    return ClosestPoint;
}

glm::vec3 Box::GetExtent() const { return 0.5f * (max - min); }

glm::vec3 Box::GetSize() const { return (max - min); }

float Box::GetVolume() const { return ((max.x - min.x) * (max.y - min.y) * (max.z - min.z)); }

Box Box::BuildAABB(const glm::vec3& origin, const glm::vec3& extent) {
    Box NewBox(origin - extent, origin + extent);
    return NewBox;
}

BoxSphereBounds::BoxSphereBounds() : origin(glm::vec3(0)), boxExtent(glm::vec3(0)), sphereRadius(0.f) {}

BoxSphereBounds::BoxSphereBounds(const Box& box) {
    box.GetCenterAndExtents(origin, boxExtent);
    sphereRadius = glm::length(boxExtent);
}

}  // namespace Maths

namespace Utils {

bool IsNumeric(const std::string& str) {
    bool result = false;
    try {
        std::stod(str);
        result = true;
    } catch (const std::exception&) {
    }
    return result;
}

void SanitizeInvalidCharsInline(std::string& inText, const char* invalidChars) {
    const char* InvalidChar = invalidChars ? invalidChars : "";
    while (*InvalidChar) {
        for (auto& Character : inText) {
            Character = Character == *InvalidChar ? '_' : Character;
        }
        ++InvalidChar;
    }
}

std::string SanitizeInvalidChars(const std::string& inText, const char* invalidChars) {
    std::string SanitizedText = inText;
    SanitizeInvalidCharsInline(SanitizedText, invalidChars);
    return SanitizedText;
}

bool ExistPath(const std::string& path) {
    try {
        return std::filesystem::exists(path);
    } catch (std::filesystem::filesystem_error error) {
        LOG_DEBUG(std::string("[std::filesystem::filesystem_error]: ") + error.what());
        return false;
    } catch (...) {
        return false;
    }
}

bool EnsurePath(const std::string& path) {
    std::string dir_path = std::filesystem::path(path).parent_path().generic_string();
    try {
        if (!ExistPath(dir_path) || !std::filesystem::is_directory(dir_path)) {
            return std::filesystem::create_directories(dir_path);
        }
        return true;
    } catch (std::filesystem::filesystem_error error) {
        LOG_DEBUG(std::string("[std::filesystem::filesystem_error]: ") + error.what());
        return false;
    } catch (...) {
        return false;
    }
}

std::string SanitizeObjectName(const std::string& inObjectName) { return SanitizeInvalidChars(inObjectName, INVALID_OBJECTNAME_CHARACTERS); }

#ifdef _MSC_VER
#include <Windows.h>
std::string ANSItoUTF8(std::string& strAnsi) {
#ifdef _DEBUG
    return strAnsi;
#else
    UINT nLen = MultiByteToWideChar(CP_ACP, NULL, strAnsi.data(), -1, NULL, NULL);
    WCHAR* wszBuffer = new WCHAR[nLen + 1];
    nLen = MultiByteToWideChar(CP_ACP, NULL, strAnsi.data(), -1, wszBuffer, nLen);
    wszBuffer[nLen] = 0;
    nLen = WideCharToMultiByte(CP_UTF8, NULL, wszBuffer, -1, NULL, NULL, NULL, NULL);
    CHAR* szBuffer = new CHAR[nLen + 1];
    nLen = WideCharToMultiByte(CP_UTF8, NULL, wszBuffer, -1, szBuffer, nLen, NULL, NULL);
    szBuffer[nLen] = 0;
    strAnsi = std::string(szBuffer);
    delete[] wszBuffer;
    delete[] szBuffer;
    return strAnsi;
#endif
}
#endif

bool InsensitiveCaseEquals(const std::string& a, const std::string& b) {
    unsigned int sz = a.size();
    if (b.size() != sz) return false;
    for (unsigned int i = 0; i < sz; i++)
        if (tolower(a[i]) != tolower(b[i])) return false;
    return true;
}

void ToLowerCase(std::string& data) {
    std::transform(data.begin(), data.end(), data.begin(), [](unsigned char c) { return std::tolower(c); });
}

std::string CleanIllegalChar(const std::string& str, bool heightLevel) {
    std::string result = str;
    for (int i = 0; i < result.length(); i++) {
        if (result.at(i) == '<' || result.at(i) == '>' || result.at(i) == '|' || result.at(i) == '\"' || result.at(i) == '?' || result.at(i) == '*' || result.at(i) == '#') {
            result.at(i) = '_';
        }
    }
    if (heightLevel) {
        for (int i = 0; i < result.length(); i++) {
            if (result.at(i) == '/' || result.at(i) == ':') {
                result.at(i) = '_';
            }
        }
    }
    return result;
}
}  // namespace Utils

std::pair<std::shared_ptr<FbxInfo>, std::shared_ptr<Objects::Entity>> ParseFbx(const std::string& filePath, std::shared_ptr<Options> options) {
    auto iptr = Fbx::Importer::Importer::GetInstance();
    auto info = iptr->GetFileSceneInfo(filePath, options);
    if (info) {
        auto scene = iptr->ImportScene(info, options);
        if (scene) {
            scene->root->name = std::filesystem::path(filePath).stem().generic_string();
            std::shared_ptr<FbxInfo> fbxInfo = std::make_shared<FbxInfo>();
            fbxInfo->fileVersion = info->FbxFileVersion;
            fbxInfo->fileCreator = info->FbxFileCreator;
            fbxInfo->lastSavedVendor = info->FbxLastSavedVendor;
            fbxInfo->lastSavedAppName = info->FBXLastSavedAppName;
            fbxInfo->lastSavedAppVersion = info->FBXLastSavedAppVersion;
            fbxInfo->materialCount = info->totalMaterialNum;
            fbxInfo->textureCount = info->totalTextureNum;
            fbxInfo->geometryCount = info->totalGeometryNum;
            fbxInfo->nonSkinnedMeshCount = info->nonSkinnedMeshNum;
            fbxInfo->skinnedMeshCount = info->skinnedMeshNum;
            fbxInfo->bHasAnimation = info->bHasAnimation;
            fbxInfo->framerate = info->originalFbxFramerate;
            fbxInfo->time = info->totalTime;
            std::string unitSystem = info->originalFileUnitSystem.GetScaleFactorAsString(false).Buffer();
            fbxInfo->unitSystem = unitSystem;
            return std::make_pair(fbxInfo, scene->root);
        }
    }
    return std::make_pair(nullptr, nullptr);
}

}  // namespace Fbx