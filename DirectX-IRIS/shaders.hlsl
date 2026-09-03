// IRIS Hyperreal Earth — DirectX12 HLSL
// Ported from index-hyperreal-directx.html:350-400 ShaderMaterial (GLSL -> HLSL)
// This is the *true* DirectX shader for native hyperrealism

Texture2D dayTexture : register(t0);
Texture2D nightTexture : register(t1);
Texture2D bumpTexture : register(t2);
Texture2D specularTexture : register(t3); // water mask
Texture2D cloudTexture : register(t4);
SamplerState samAniso : register(s0); // 16x anisotropic

cbuffer SceneCB : register(b0)
{
    float4x4 mWorldViewProj;
    float4x4 mWorld;
    float3 sunDirection;
    float time;
    float3 cameraPos;
    float exposure;
};

// Vertex
struct VSInput { float3 pos : POSITION; float2 uv : TEXCOORD0; float3 normal : NORMAL; };
struct PSInput { float4 pos : SV_Position; float2 uv : TEXCOORD0; float3 worldPos : TEXCOORD1; float3 normal : TEXCOORD2; };

PSInput VS(VSInput vin)
{
    PSInput o;
    float4 worldPos = mul(float4(vin.pos,1), mWorld);
    o.worldPos = worldPos.xyz;
    o.pos = mul(float4(vin.pos,1), mWorldViewProj);
    o.uv = vin.uv;
    o.normal = normalize(mul(vin.normal, (float3x3)mWorld));
    return o;
}

// ACES Filmic (DirectX games)
float3 ACES(float3 x){
    float a=2.51, b=0.03, c=2.43, d=0.59, e=0.14;
    return saturate((x*(a*x+b))/(x*(c*x+d)+e));
}

float4 PS(PSInput pin) : SV_Target
{
    float3 day = pow(dayTexture.Sample(samAniso, pin.uv).rgb, 0.85) * 1.55;
    float3 night = pow(nightTexture.Sample(samAniso, pin.uv).rgb, 0.88) * 1.90;
    float bump = bumpTexture.Sample(samAniso, pin.uv).r;
    float specMask = specularTexture.Sample(samAniso, pin.uv).r; // 1=water
    float cloud = cloudTexture.Sample(samAniso, pin.uv).r;
    float cloudAlpha = cloud * 0.95;

    // cloud shadows parallax
    float2 cloudUvOffset = pin.uv + sunDirection.xy * 0.008;
    float cloudShadow = cloudTexture.Sample(samAniso, cloudUvOffset).r;
    day *= (1 - cloudShadow*0.38);
    day = lerp(day, float3(1,1,1), cloudAlpha*0.42);

    float landFactor = 1 - smoothstep(0.2, 0.55, specMask);
    night *= landFactor * 1.2;

    float3 N = normalize(pin.normal);
    float3 bumpN = normalize(N + (bump - 0.5)*0.65 * float3(0.8,0.8,0.4));
    N = bumpN;

    float3 sunDir = normalize(sunDirection);
    float NdotL = dot(N, sunDir);
    float dayMix = smoothstep(-0.32, 0.34, NdotL);
    float nightMix = smoothstep(0.18, -0.32, NdotL);

    float3 viewDir = normalize(cameraPos - pin.worldPos);
    float fres = pow(1 - saturate(dot(N, viewDir)), 3.2);

    // DirectX PBR Ocean specular
    float3 halfDir = normalize(sunDir + viewDir);
    float specPower = lerp(18, 98, specMask);
    float specIntensity = lerp(0.08, 1.0, specMask);
    float NdotH = saturate(dot(N, halfDir));
    float spec = pow(NdotH, specPower) * 1.85 * specIntensity * dayMix;
    spec *= (1 - cloudShadow*0.55);
    float oceanFres = pow(1 - saturate(dot(N, viewDir)), 5.0)*specMask*0.55*dayMix;

    float3 col = lerp(night*1.75, day, dayMix);
    col += (bump - 0.5)*0.11*dayMix;
    col += fres * float3(0.22,0.58,1.02)*0.78*dayMix;
    col += float3(1,0.98,0.92)*spec;
    col += float3(0.28,0.62,1.0)*oceanFres;
    col = lerp(col, night*2.05, nightMix*0.42);
    float terminator = exp(-pow(NdotL*8,2))*0.18;
    col += float3(0.15,0.45,1.0)*terminator*dayMix*nightMix*4.0;

    col *= 1.28;
    col = ACES(col * exposure);
    col = lerp(col, col*float3(1.08,1.02,0.92), dayMix*0.12);
    col = pow(col, 0.98);
    return float4(col,1);
}

// Atmosphere — Rayleigh + Mie (vertex similar, pixel computes scatter)
float4 AtmoPS(PSInput pin) : SV_Target
{
    float3 N = normalize(pin.normal);
    float3 V = normalize(cameraPos - pin.worldPos);
    float NdotV = saturate(dot(N,V));
    float NdotL = saturate(dot(N, normalize(sunDirection)));
    float rayleigh = pow(1 - NdotV, 3.8) * (0.6 + NdotL*0.55);
    float mie = pow(saturate(dot(V, normalize(sunDirection))), 42) * 0.72;
    float3 rayleighCol = float3(0.18,0.48,1.0)* rayleigh *1.1;
    float3 mieCol = float3(1,0.82,0.45)* mie;
    float alpha = rayleigh*0.92 + mie*0.58;
    return float4(rayleighCol+mieCol, alpha*0.95);
}
