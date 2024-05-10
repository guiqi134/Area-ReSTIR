add_library(AreaReSTIR STATIC)
# add_plugin(AreaReSTIR)

target_sources(AreaReSTIR PRIVATE
    EvalContext.slang
    EvaluateFinalSamples.cs.slang
    FinalSample.slang
    GenerateLightTiles.cs.slang
    InitialResampling.cs.slang
    Lights.slang
    LightSample.slang
    LocalEmissiveTriangle.slang
    Params.slang
    ReflectTypes.cs.slang
    Resampling.slang
    Reservoir.slang
    AreaReSTIR.cpp
    AreaReSTIR.h
    AreaReSTIR.slang
    SpatialResampling.cs.slang
    SurfaceData.slang
    TemporalResampling.cs.slang
    UpdateEmissiveTriangles.cs.slang
    LoadShadingData.slang
    PixelAreaSampleData.slang
    TemporalResampling_FloatMotion.cs.slang
    SimpleCameraData.slang
    StaticParams.slang
    TemporalMSAATracePrimaryRays.cs.slang
)

target_copy_shaders(AreaReSTIR Modules/AreaReSTIR)

target_link_libraries(AreaReSTIR
    PUBLIC
        Falcor
)

target_include_directories(AreaReSTIR
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set_target_properties(AreaReSTIR
    PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        LIBRARY_OUTPUT_DIRECTORY ${FALCOR_RUNTIME_OUTPUT_DIRECTORY}
)

target_source_group(AreaReSTIR "Modules")

validate_headers(AreaReSTIR)
