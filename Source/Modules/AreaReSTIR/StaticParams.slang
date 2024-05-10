__exported import Rendering.Materials.TexLODTypes;

/** Translation of defines set by the host to compile-time constants used to
    configure the path tracer without overhead from dynamic control flow.
    This will eventually be replaced by specialization constants in Slang.
*/

#ifndef SAMPLES_PER_PIXEL
#error StaticParams.slang is missing defines
#endif

static const bool kAdjustShadingNormals = ADJUST_SHADING_NORMALS;
static const bool kUseAlphaTest = USE_ALPHA_TEST;
static const bool kUseNormalMapping = USE_NORMAL_MAPPING;
