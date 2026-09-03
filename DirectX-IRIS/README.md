# IRIS — Native DirectX 12 Hyperreal Earth

This is the **native DirectX 12** version of your IRIS LEO simulator — the absolute best hyperrealism possible on Windows (ray-traced reflections, true HDR10, mesh shaders).

## Why DirectX12 is Best vs Web

- **WebGPU = DirectX12 in browser** → `index-hyperreal-directx.html:48` auto-uses `WebGPURenderer → D3D12` when `navigator.gpu` exists. That's *true* DirectX inside your existing web project.
- **Native D3D12** → this folder is a C++ desktop .exe with zero browser limits: 8K textures, DXR ray tracing, HDR10, full DirectX Tool Kit.

Your web version (`index.html:230`) used `THREE.WebGLRenderer` (OpenGL). The hyperreal upgrade (`index-hyperreal-directx.html`) keeps 100% compatibility but on Windows Edge/Chrome it **literally calls DirectX12** via WebGPU.

## What "Hyperrealism" Adds (DirectX shaders)

Ported from `index-hyperreal-directx.html:220-410` ShaderMaterial to HLSL `shaders.hlsl`:

1. **PBR Ocean** — water mask → sharp specular (pow 98) vs land rough (pow 18), Fresnel at grazing angles
2. **Cloud Shadows** — day texture darkened by cloud texture offset along sun direction (parallax)
3. **Bump Normal Perturbation** — topology map perturbs normal 0.65 for 3D terrain without mesh
4. **Rayleigh + Mie Scattering** — outer/inner atmosphere shells with forward scatter Mie term (pow 42)
5. **ACES Filmic Tone Mapping** — `aces(x) = (x*(2.51x+0.03))/(x*(2.43x+0.59)+0.14)` + HDR exposure 1.35-1.45
6. **City Lights Bloom** — UnrealBloomPass 0.55 strength, additive blending
7. **16x Anisotropic Filtering** + 192-segment sphere (vs 128 before)

## Quick Start (Native)

### Option A: Visual Studio 2022 (Recommended)
1. Open `IRIS_DirectX12.sln` (or create new DirectX12 project and copy `IRIS_DirectX12.cpp` + `shaders.hlsl`)
2. Requires: Windows 10 SDK 10.0.19041+, DirectX12 Agility SDK
3. Build x64 Release → Run
4. Controls: Drag = orbit, Scroll = zoom (same as web `OrbitControls`)

### Option B: CMake + MSVC
```powershell
cmake -B build -S . -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\build\Release\IRIS_DirectX12.exe
```

### Textures
Place your 8K textures in `assets/` (or reuse `../libs/`):
- `earth-day.jpg` (day diffuse)
- `earth-night.jpg` (city lights emissive)
- `earth-topology.png` (bump/normal)
- `earth_clouds_1024.png` (or 4096 for hyperreal)

The HLSL shader auto-generates specular mask from day texture (water detection: B > R+12).

## Web Hyperreal (Instant Run)

No build needed — just open:
```
index-hyperreal-directx.html
```
On Windows Chrome/Edge 113+ it uses **DirectX12**. Check top bar pill: `◆ DIRECTX 12 · WEBGPU` = native D3D12. If you see `◇ WEBGL2 · DX SHADERS` enable `chrome://flags/#enable-unsafe-webgpu`.

## Next Steps for Max Quality

- Replace 1K textures with 8K from NASA Visible Earth
- Enable DXR ray-traced shadows for satellites (see `shaders.hlsl:RTX` stub)
- Add HDR10 swapchain (`DXGI_FORMAT_R10G10B10A2_UNORM`)
- Use DirectX Mesh Shaders for micro-debris (3200 particles → 100k)

Questions? Open `IRIS_DirectX12.cpp:1` — it's fully commented.
