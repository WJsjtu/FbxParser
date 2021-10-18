#include "Resource.h"
namespace Exporter {
AnimationClip::AnimationClip(std::shared_ptr<Fbx::Assets::AnimationClipAsset> inAnimationClip) : Resource(""), animationClip(inAnimationClip) {}

std::string AnimationClip::GetHash() { return animationClip->uniqueID; }

std::string AnimationClip::GetExportPath() {
    std::string savePath = animationClip->name + ".animationclip";
    if (!isPathLocked) {
        isPathLocked = true;
        int renameCount = 0;
        while (!ExportStore::IsFilePathValid(savePath)) {
            savePath = animationClip->name + "_" + std::to_string(++renameCount) + ".animationclip";
        }
    }
    return savePath;
}

std::string AnimationClip::ExportResource() {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    writer.StartObject();
    writer.String("keyframeData");
    writer.StartObject();

    writer.String("bone");
    writer.StartObject();
    writer.String("sampleList");
    writer.StartArray();
    for (auto& curve : animationClip->curves) {
        writer.StartObject();
        writer.String("pathIndex");
        writer.Uint(curve->index);
        writer.String("type");
        writer.Uint(static_cast<uint32_t>(curve->type));
        writer.String("keyframeCount");
        writer.Uint(curve->frameCount);
        writer.EndObject();
    }
    writer.EndArray();
    writer.String("contentList");
    writer.StartArray();
    for (auto& frame : animationClip->frames) {
        writer.StartObject();
        writer.String("frameId");
        writer.Uint(frame->frameIndex);
        writer.String("value");
        writer.Double(frame->value);
        writer.String("inTangent");
        writer.Double(frame->inTangent);
        writer.String("outTangent");
        writer.Double(frame->outTangent);
        writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();

    writer.String("meta");
    writer.StartObject();
    writer.String("sampleList");
    writer.StartArray();
    writer.EndArray();
    writer.String("contentList");
    writer.StartArray();
    writer.EndArray();
    writer.EndObject();

    writer.String("eventList");
    writer.StartArray();
    writer.EndArray();

    writer.String("frameRate");
    writer.Uint(animationClip->frameRate);

    writer.String("totalFrameCount");
    writer.Uint(animationClip->frameCount);

    writer.EndObject();

    writer.String("data");
    writer.StartObject();
    writer.String("name");
    JSON_STRING_VALUE(writer, animationClip->name);
    writer.String("paths");
    writer.StartArray();
    for (auto& path : animationClip->paths) {
        JSON_STRING_VALUE(writer, path);
    }
    writer.EndArray();
    writer.String("wrapMode");
    JSON_STRING_VALUE(writer, std::string("Default"));
    writer.String("startTime");
    writer.Double(0);
    writer.String("stopTime");
    writer.Double(animationClip->length);
    writer.String("loopTime");
    writer.Bool(false);
    writer.String("cycleOffset");
    writer.Double(0);

    writer.EndObject();
    writer.EndObject();

    std::string animationClipJSON = sb.GetString();
#ifdef _DEBUG
    PRETTIFY_JSON(animationClipJSON);
#endif

    return animationClipJSON;
}

std::string AnimationClip::GetResourceType() { return "animationclip"; }

}  // namespace Exporter