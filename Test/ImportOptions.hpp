#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
static const std::string ImportOptionsJsonSchema = "{\"type\":\"object\",\"properties\":{\"convertAxis\":{\"type\":\"boolean\"},\"convertUnit\":{\"type\":\"boolean\"},\"animation\":{\"type\":\"object\",\"properties\":{\"importAnimations\":{\"type\":\"boolean\"},\"animationLengthImportType\":{\"enum\":[0,1],\"type\":\"number\"},\"resample\":{\"type\":\"boolean\"},\"resampleRate\":{\"type\":\"number\"},\"bImportMorph\":{\"type\":\"boolean\"},\"doNotImportCurveWithZero\":{\"type\":\"boolean\"},\"importCustomAttribute\":{\"type\":\"boolean\"},\"importBoneTracks\":{\"type\":\"boolean\"},\"preserveLocalTransform\":{\"type\":\"boolean\"}}},\"mesh\":{\"type\":\"object\",\"properties\":{\"preserveSmoothingGroups\":{\"type\":\"boolean\"},\"importMorphTargets\":{\"type\":\"boolean\"},\"thresholdPosition\":{\"type\":\"number\"},\"thresholdTangentNormal\":{\"type\":\"number\"},\"thresholdUV\":{\"type\":\"number\"},\"morphThresholdPosition\":{\"type\":\"number\"},\"computeNormal\":{\"enum\":[0,1,2],\"type\":\"number\"},\"computeTangent\":{\"enum\":[0,1,2],\"type\":\"number\"}}}},\"$schema\":\"http://json-schema.org/draft-07/schema#\"}";


