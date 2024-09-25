import Scene.Scene;

Texture2D<float4> gPosW;
Texture2D<float4> gNormW;
RWTexture2D<float2> gPosNormalFwidth;
RWTexture2D<float2> gLinearZ;

cbuffer CB
{
    uint2 gFrameDim;
}

float3 computeDdxPosW(float3 posW, float3 normW, float2 invFrameDim)
{
    float3 projRight = normalize(cross(normW, cross(normW, gScene.camera.data.cameraV)));
    float distanceToHit = length(posW - gScene.camera.data.posW);
    float2 ddNdc = float2(2.f, -2.f) * invFrameDim;
    float distRight = distanceToHit * ddNdc.x / dot(normalize(gScene.camera.data.cameraV), projRight);
    return distRight * projRight;
}

float3 computeDdyPosW(float3 posW, float3 normW, float2 invFrameDim)
{
    float3 projUp = normalize(cross(normW, cross(normW, gScene.camera.data.cameraU)));
    float distanceToHit = length(posW - gScene.camera.data.posW);
    float2 ddNdc = float2(2.f, -2.f) * invFrameDim;
    float distUp = distanceToHit * ddNdc.y / dot(normalize(gScene.camera.data.cameraU), projUp);
    return distUp * projUp;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    if (any(pixel >= gFrameDim)) return;
    uint2 rightPixel = pixel + uint2(1, 0);
    uint2 bottomPixel = pixel + uint2(0, 1);

    // Compute position filter width
    float3 ddxPosW = (gPosW[rightPixel] - gPosW[pixel]).xyz;
    float3 ddyPosW = (gPosW[bottomPixel] - gPosW[pixel]).xyz;
    float FwidthPosW = length(abs(ddxPosW) + abs(ddy(ddyPosW)));

    // Compute normal filter width
    float3 ddxNormW = (gNormW[rightPixel] - gNormW[pixel]).xyz;
    float3 ddyNormW = (gNormW[bottomPixel] - gNormW[pixel]).xyz;
    float FwidthNormW = length(abs(ddxNormW) + abs(ddy(ddyNormW)));

    // Compute linear z
    float4 curPosH = mul(gScene.camera.data.viewProjMatNoJitter, float4(gPosW[pixel].xyz, 1.f));
    float curLinearZ = curPosH.w;
    float4 curPosH_dx = mul(gScene.camera.data.viewProjMatNoJitter, float4(gPosW[pixel].xyz + ddxPosW, 1.f));
    float4 curPosH_dy = mul(gScene.camera.data.viewProjMatNoJitter, float4(gPosW[pixel].xyz + ddxPosW, 1.f));
    float ddxLinearZ = abs(curPosH_dx.w - curLinearZ);
    float ddyLinearZ = abs(curPosH_dy.w - curLinearZ);
    float dLinearZ = max(ddxLinearZ, ddyLinearZ);
    gLinearZ[pixel] = float2(curLinearZ, dLinearZ);

    gPosNormalFwidth[pixel] = float2(FwidthPosW, FwidthNormW);
}
