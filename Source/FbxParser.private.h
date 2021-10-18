#pragma once
#include <glm/ext/matrix_clip_space.hpp>  // glm::perspective
#include <glm/ext/matrix_transform.hpp>   // glm::translate, glm::rotate, glm::scale
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext/scalar_constants.hpp>  // glm::pi
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/mat4x4.hpp>  // glm::mat4
#include <glm/vec3.hpp>    // glm::vec3
#include <glm/vec4.hpp>    // glm::vec4
#include <ostream>
#include <sstream>
#include "FbxParser.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

namespace Fbx {
namespace Types {

Vector2& ConvertFromGLM(Vector2& vec1, const glm::vec2& vec2);

Vector3& ConvertFromGLM(Vector3& vec1, const glm::vec3& vec2);

Vector4& ConvertFromGLM(Vector4& vec1, const glm::vec4& vec2);

Vector4& ConvertFromGLM(Vector4& vec1, const glm::quat& vec2);

Matrix4& ConvertFromGLM(Matrix4& mat1, const glm::mat4& mat2);

}  // namespace Types
namespace Maths {
/**
 * Converts a float to an integer value with truncation towards zero.
 * @param f		Floating point value to convert
 * @return		Truncated integer value.
 */
float TruncToFloat(float f);

/** Returns higher value in a generic way */
template <class T>
constexpr T Max(const T A, const T B) {
    return (A >= B) ? A : B;
}

/** Returns lower value in a generic way */
template <class T>
constexpr T Min(const T A, const T B) {
    return (A <= B) ? A : B;
}

/** Computes absolute value in a generic way */
template <class T>
constexpr T Abs(const T A) {
    return (A >= (T)0) ? A : -A;
}

/** Returns highest of 3 values */
template <class T>
T Max3(const T A, const T B, const T C) {
    return Max(Max(A, B), C);
}

/** Returns lowest of 3 values */
template <class T>
T Min3(const T A, const T B, const T C) {
    return Min(Min(A, B), C);
}

/**
 *	Checks if two floating point numbers are nearly equal.
 *	@param A				First number to compare
 *	@param B				Second number to compare
 *	@param ErrorTolerance	Maximum allowed difference for considering them as 'nearly equal'
 *	@return					true if A and B are nearly equal
 */
template <class T>
bool IsNearlyEqual(const T& A, const T& B, float errorTolerance = 1.e-8f) {
    return Abs<T>(A - B) <= errorTolerance;
}

template <>
bool IsNearlyEqual<glm::vec3>(const glm::vec3& A, const glm::vec3& B, float errorTolerance);

template <>
bool IsNearlyEqual<glm::quat>(const glm::quat& A, const glm::quat& B, float errorTolerance);

/**
 *	Checks if a floating point number is nearly zero.
 *	@param Value			Number to compare
 *	@param ErrorTolerance	Maximum allowed difference for considering Value as 'nearly zero'
 *	@return					true if Value is nearly zero
 */
template <class T>
bool IsNearlyZero(const T& value, float errorTolerance = 1.e-8f) {
    return Maths::Abs(value) <= errorTolerance;
}

template <>
bool IsNearlyZero<glm::vec3>(const glm::vec3& vec, float tolerance);

bool ContainsNaN(const glm::vec3& vec);

bool ContainsNaN(const glm::quat& vec);

/**
 * Returns signed fractional part of a float.
 * @param Value	Floating point value to convert
 * @return		A float between >=0 and < 1 for nonnegative input. A float between >= -1 and < 0 for negative
 * input.
 */
float Fractional(float value);

int RoundToInt(double value);

// Use the Euclidean method to find the GCD
int GreatestCommonDivisor(int a, int b);

// LCM = a/gcd * b
// a and b are the number we want to find the lcm
int LeastCommonMultiplier(int a, int b);

template <class T>
bool IsNaN(const T value) {
    return std::isnan(value);
}

float Rand();

float Acos(float value);

void GetMatrixScaledAxes(glm::mat4 mat, glm::vec3& x, glm::vec3& y, glm::vec3& z);

class Transform {
public:
    glm::vec3 translation = glm::vec3(0, 0, 0);
    glm::vec3 scale = glm::vec3(0, 0, 0);
    glm::quat rotation = glm::quat(1, 0, 0, 0);

    static glm::vec3 ExtractScaling(glm::mat4& mat, float tolerance = 1.e-8f);

    void SetFromMatrix(const glm::mat4& inMatrix);

    glm::mat4 ToMatrixWithScale() const;

    void NormalizeRotation();
};

/**
 * Implements an axis-aligned box.
 *
 * Boxes describe an axis-aligned extent in three dimensions. They are used for many different things in the
 * Engine and in games, such as bounding volumes, collision detection and visibility calculation.
 */
class Box {
public:
    /** Holds the box's minimum point. */
    glm::vec3 min;

    /** Holds the box's maximum point. */
    glm::vec3 max;

    /** Holds a flag indicating whether this box is valid. */
    bool bIsValid;

public:
    Box();

    /**
     * Creates and initializes a new box from the specified extents.
     *
     * @param inMin The box's minimum point.
     * @param inMax The box's maximum point.
     */
    Box(const glm::vec3& inMin, const glm::vec3& inMax);

    /**
     * Creates and initializes a new box from an array of points.
     *
     * @param points Array of Points to create for the bounding volume.
     */
    Box(const std::vector<glm::vec3>& points);

public:
    /**
     * Compares two boxes for equality.
     *
     * @return true if the boxes are equal, false otherwise.
     */
    bool operator==(const Box& other) const;

    /**
     * Adds to this bounding box to include a given point.
     *
     * @param other the point to increase the bounding volume to.
     * @return Reference to this bounding box after resizing to include the other point.
     */
    Box& operator+=(const glm::vec3& other);

    /**
     * Gets the result of addition to this bounding volume.
     *
     * @param other The other point to add to this.
     * @return A new bounding volume.
     */
    Box operator+(const glm::vec3& other) const;

    /**
     * Gets the result of addition to this bounding volume.
     *
     * @param other The other volume to add to this.
     * @return A new bounding volume.
     */
    Box operator+(const Box& other) const;

    /**
     * Adds to this bounding box to include a new bounding volume.
     *
     * @param Other the bounding volume to increase the bounding volume to.
     * @return Reference to this bounding volume after resizing to include the other bounding volume.
     */
    Box& operator+=(const Box& Other);

    /**
     * Gets reference to the min or max of this bounding volume.
     *
     * @param index the index into points of the bounding volume.
     * @return a reference to a point of the bounding volume.
     */
    glm::vec3& operator[](int index);

public:
    /**
     * Increases the box size.
     *
     * @param w The size to increase the volume by.
     * @return A new bounding box.
     */
    Box ExpandBy(float w) const;

    /**
     * Increases the box size.
     *
     * @param vV The size to increase the volume by.
     * @return A new bounding box.
     */
    Box ExpandBy(const glm::vec3& v) const;

    /**
     * Increases the box size.
     *
     * @param neg The size to increase the volume by in the negative direction (positive values move the bounds
     * outwards)
     * @param pos The size to increase the volume by in the positive direction (positive values move the bounds
     * outwards)
     * @return A new bounding box.
     */
    Box ExpandBy(const glm::vec3& neg, const glm::vec3& pos) const;

    /**
     * Shifts the bounding box position.
     *
     * @param offset The vector to shift the box by.
     * @return A new bounding box.
     */
    Box ShiftBy(const glm::vec3& offset) const;

    /**
     * Moves the center of bounding box to new destination.
     *
     * @param The destination point to move center of box to.
     * @return A new bounding box.
     */
    Box MoveTo(const glm::vec3& destination) const;

    /**
     * Gets the center point of this box.
     *
     * @return The center point.
     * @see GetCenterAndExtents, GetExtent, GetSize, GetVolume
     */
    glm::vec3 GetCenter() const;

    /**
     * Gets the center and extents of this box.
     *
     * @param center[out] Will contain the box center point.
     * @param Extents[out] Will contain the extent around the center.
     * @see GetCenter, GetExtent, GetSize, GetVolume
     */
    void GetCenterAndExtents(glm::vec3& center, glm::vec3& extents) const;

    /**
     * Calculates the closest point on or inside the box to a given point in space.
     *
     * @param point The point in space.
     * @return The closest point on or inside the box.
     */
    glm::vec3 GetClosestPointTo(const glm::vec3& point) const;

    /**
     * Gets the extents of this box.
     *
     * @return The box extents.
     * @see GetCenter, GetCenterAndExtents, GetSize, GetVolume
     */
    glm::vec3 GetExtent() const;

    /**
     * Gets the size of this box.
     *
     * @return The box size.
     * @see GetCenter, GetCenterAndExtents, GetExtent, GetVolume
     */
    glm::vec3 GetSize() const;

    /**
     * Gets the volume of this box.
     *
     * @return The box volume.
     * @see GetCenter, GetCenterAndExtents, GetExtent, GetSize
     */
    float GetVolume() const;

public:
    /**
     * Utility function to build an AABB from Origin and Extent
     *
     * @param origin The location of the bounding box.
     * @param extent Half size of the bounding box.
     * @return A new axis-aligned bounding box.
     */
    static Box BuildAABB(const glm::vec3& origin, const glm::vec3& extent);
};

class BoxSphereBounds {
public:
    BoxSphereBounds();

    /**
     * Creates and initializes a new instance the given Box.
     *
     * The sphere radius is taken from the extent of the box.
     *
     * @param Box The bounding box.
     */
    BoxSphereBounds(const Box& box);

    /** Holds the origin of the bounding box and sphere. */
    glm::vec3 origin;

    /** Holds the extent of the bounding box. */
    glm::vec3 boxExtent;

    /** Holds the radius of the bounding sphere. */
    float sphereRadius;
};

}  // namespace Maths
namespace Utils {
class DefaultLogInstance {
public:
    DefaultLogInstance(const std::string& logFilePath);
    void Log(ELogLevel level, std::string message);
    std::shared_ptr<spdlog::logger> logger = nullptr;
};

bool IsNumeric(const std::string& str);

void static SanitizeInvalidCharsInline(std::string& inText, const char* invalidChars);

std::string static SanitizeInvalidChars(const std::string& inText, const char* invalidChars);

bool ExistPath(const std::string& path);

bool EnsurePath(const std::string& path);

std::string SanitizeObjectName(const std::string& inObjectName);

std::string CleanIllegalChar(const std::string& str, bool heightLevel);

bool InsensitiveCaseEquals(const std::string& a, const std::string& b);

void ToLowerCase(std::string& data);

/*! note: delimiter cannot contain NUL characters
 */
template <typename Range, typename Value = typename Range::value_type>
std::string Join(Range const& elements, const char* const delimiter) {
    std::ostringstream os;
    auto b = begin(elements), e = end(elements);

    if (b != e) {
        std::copy(b, prev(e), std::ostream_iterator<Value>(os, delimiter));
        b = prev(e);
    }
    if (b != e) {
        os << *b;
    }

    return os.str();
}

/*! note: imput is assumed to not contain NUL characters
 */
template <typename Input, typename Output, typename Value = typename Output::value_type>
void Split(char delimiter, Output& output, Input const& input) {
    using namespace std;
    for (auto cur = begin(input), beg = cur;; cur++) {
        if (cur == end(input) || *cur == delimiter || !*cur) {
            output.insert(output.end(), Value(beg, cur));
            if (cur == end(input) || !*cur)
                break;
            else
                beg = next(cur);
        }
    }
}
#ifdef _MSC_VER
std::string ANSItoUTF8(std::string& strAnsi);
#endif
}  // namespace Utils
}  // namespace Fbx

namespace std {
template <>
struct less<glm::vec3> {
    bool operator()(const glm::vec3& left, const glm::vec3& right) const { return (left.x < right.x) ? true : ((left.y < right.y) ? true : (left.z < right.z)); }
};
}  // namespace std

#define ANIMATION_SMALL_NUMBER (1.e-7f)

/** These characters cannot be used in object names */
#define INVALID_OBJECTNAME_CHARACTERS ("\"' ,/.:|&!~\n\r\t@#(){}[]=;^%$`")

/** These characters cannot be used in ObjectPaths, which includes both the package path and part after the first . */
#define INVALID_OBJECTPATH_CHARACTERS ("\"' ,|&!~\n\r\t@#(){}[]=;^%$`")

#define INVALID_UNIQUE_ID 0xFFFFFFFFFFFFFFFF

#define GeneratedLODNameSuffix "_GeneratedLOD_"

#define ASSERT(expr)                                                                                                                                           \
    if (!(expr)) {                                                                                                                                             \
        throw Fbx::AssertException("Assert failed, file: " + std::string(__FILE__) + ", line: " + std::to_string(__LINE__) + ", expr: " + std::string(#expr)); \
    }

#ifdef _MSC_VER
#define LOG_DEBUG(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Debug, Fbx::Utils::ANSItoUTF8(std::string(str))); }
#define LOG_WARN(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Warn, Fbx::Utils::ANSItoUTF8(std::string(str))); }
#define LOG_INFO(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Info, Fbx::Utils::ANSItoUTF8(std::string(str))); }
#define LOG_ERROR(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Error, Fbx::Utils::ANSItoUTF8(std::string(str))); }
#define LOG_CRITICAL(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Critical, Fbx::Utils::ANSItoUTF8(std::string(str))); }
#else
#define LOG_DEBUG(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Debug, std::string(str)); }
#define LOG_WARN(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Warn, std::string(str)); }
#define LOG_INFO(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Info, std::string(str)); }
#define LOG_ERROR(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Error, std::string(str)); }
#define LOG_CRITICAL(str) \
    { Fbx::Utils::GetGlobalLogger()(Fbx::Utils::ELogLevel::Critical, std::string(str)); }
#endif