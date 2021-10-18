enum NormalImportOptions {
  ImportOrComputeNormal = 0,
  ForceComputeNormal,
  IgnoreNormal,
};

enum TangentImportOptions {
  ImportOrComputeTangent = 0,
  ForceComputeTangent,
  IgnoreTangent,
};

enum EFBXAnimationLengthImportType {
  /** This option imports animation frames based on what is defined at the time of export */
  FBXALIT_ExportedTime,
  /** Will import the range of frames that have animation. Can be useful if the exported range is longer than the actual animation in the FBX file */
  FBXALIT_AnimatedKey,
};

export interface IImportOptions {
  convertAxis?: boolean; // true
  convertUnit?: boolean; // true
  animation?: {
    importAnimations?: boolean; // true
    animationLengthImportType?: EFBXAnimationLengthImportType; // EFBXAnimationLengthImportType.FBXALIT_ExportedTime
    resample?: boolean; // true
    resampleRate?: number; // 0
    bImportMorph?: boolean; // true
    doNotImportCurveWithZero?: boolean; // true
    importCustomAttribute?: boolean; // true
    importBoneTracks?: boolean; // true
    preserveLocalTransform?: boolean; // false
  },
  mesh?: {
    preserveSmoothingGroups?: boolean; // true
    importMorphTargets?: boolean; // true
    thresholdPosition?: number; // 0.00002
    thresholdTangentNormal?: number; // 0.00002
    thresholdUV?: number; // 0.0009765625
    morphThresholdPosition?: number; // 0.015
    computeNormal?: NormalImportOptions; // NormalImportOptions.ForceComputeNormal
    computeTangent?: TangentImportOptions; // TangentImportOptions.ForceComputeTangent
  }
}