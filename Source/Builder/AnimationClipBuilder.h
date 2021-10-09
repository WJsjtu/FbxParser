#pragma once
#include <vector>
#include "Importer/Animation.h"
#include "FbxParser.private.h"
#include "FbxParser.h"
namespace Fbx { namespace Builder {

struct AnimationBuildSettings {
    double posReductionThreshold = 0.5;
    double scaleReductionThreshold = 0.5;
    double rotReductionThreshold = 0.5;
};

class AnimationClipBuilder {
public:
    bool Build(std::shared_ptr<Assets::AnimationClipAsset> asset, std::shared_ptr<Importer::AnimationImportData> animationImportData, const AnimationBuildSettings& options);
};

}}  // namespace Fbx::Builder