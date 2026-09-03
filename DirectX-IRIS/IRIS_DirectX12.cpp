// IRIS — Native DirectX12 Hyperreal Earth
// Minimal D3D12 scaffold. Copy into Visual Studio DirectX12 template.
// For full hyperreal parity with index-hyperreal-directx.html, implement sphere generation & texture loading via DirectXTK12 (DDSTextureLoader).

#include <windows.h>
#include <wrl.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <string>
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using namespace DirectX;
using Microsoft::WRL::ComPtr;

// Window
HWND g_hWnd;
const UINT WIDTH = 1920, HEIGHT = 1080;

// D3D12 core
ComPtr<ID3D12Device> device;
ComPtr<IDXGISwapChain3> swapChain;
ComPtr<ID3D12CommandQueue> cmdQueue;
ComPtr<ID3D12GraphicsCommandList> cmdList;
ComPtr<ID3D12CommandAllocator> cmdAlloc;
ComPtr<ID3D12DescriptorHeap> rtvHeap;
ComPtr<ID3D12Resource> renderTargets[2];
ComPtr<ID3D12Fence> fence;
HANDLE fenceEvent;
UINT64 fenceValue = 0;
UINT frameIndex;
UINT rtvDescriptorSize;

// Shaders — compile shaders.hlsl
// VS: VS  PS: PS  AtmoPS: AtmoPS
// Sampler: 16x anisotropic (D3D12_FILTER_ANISOTROPIC, MaxAnisotropy 16) — matches JS tex.anisotropy=16

struct SceneCB {
    XMMATRIX mWorldViewProj;
    XMMATRIX mWorld;
    XMFLOAT3 sunDirection;
    float time;
    XMFLOAT3 cameraPos;
    float exposure; // 1.35 ACES
};

// Earth sphere generation (192 segments, matches JS 192,192)
struct Vertex { XMFLOAT3 pos; XMFLOAT2 uv; XMFLOAT3 normal; };
std::vector<Vertex> CreateSphere(float radius, int latSegments, int lonSegments){
    std::vector<Vertex> v;
    for(int lat=0; lat<=latSegments; ++lat){
        float theta = lat * XM_PI / latSegments;
        float sinT = sinf(theta), cosT = cosf(theta);
        for(int lon=0; lon<=lonSegments; ++lon){
            float phi = lon * 2*XM_PI / lonSegments;
            float sinP = sinf(phi), cosP = cosf(phi);
            Vertex vert;
            vert.normal = XMFLOAT3(cosP*sinT, cosT, sinP*sinT);
            vert.pos = XMFLOAT3(vert.normal.x*radius, vert.normal.y*radius, vert.normal.z*radius);
            vert.uv = XMFLOAT2((float)lon/lonSegments, (float)lat/latSegments);
            v.push_back(vert);
        }
    }
    return v;
}
// Index buffer for triangle list would be generated similarly...

// LEO shells — same defs as JS: 100,160,450,800,2000 km scaled by R_EARTH
// R_EARTH = 1 unit = 6371 km
float toScene(float km){ return km/6371.0f; }

// Main loop placeholder — real app would have OrbitControls reimplemented via mouse drag -> camera matrices
// Camera same as JS: position (0,1.9,3.2), target (0,0,0), FOV 45deg

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM w, LPARAM l){
    if(msg==WM_DESTROY){ PostQuitMessage(0); return 0; }
    return DefWindowProc(hWnd, msg, w, l);
}

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR, int nCmd){
    // 1. Create window
    WNDCLASS wc={}; wc.lpfnWndProc=WndProc; wc.hInstance=hInst; wc.lpszClassName=L"IRIS_DX12";
    RegisterClass(&wc);
    g_hWnd = CreateWindow(wc.lpszClassName, L"IRIS — Hyperreal DirectX12 LEO Earth", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, WIDTH, HEIGHT, nullptr,nullptr,hInst,nullptr);
    ShowWindow(g_hWnd, nCmd);

    // 2. Create D3D12 device & swapchain (boilerplate omitted for brevity — use DirectXTK12 sample)
    //    Important settings for hyperrealism:
    //    - Back buffer format: DXGI_FORMAT_R10G10B10A2_UNORM for HDR10 (or R8G8B8A8_UNORM + ACES)
    //    - Depth: D32_FLOAT
    //    - Sampler: D3D12_FILTER_ANISOTROPIC MaxAnisotropy=16
    //    - Blend: additive for atmosphere (SRC_ALPHA, ONE)

    MessageBox(g_hWnd,
        L"IRIS DirectX12 scaffold created.\n\n"
        L"To run hyperreal Earth:\n"
        L"1. Open this file in Visual Studio 2022 DirectX12 template\n"
        L"2. Add shaders.hlsl (VS/PS/AtmoPS)\n"
        L"3. Load textures from ../libs/ (earth-day.jpg etc) via DDSTextureLoader\n"
        L"4. Create sphere (192,192) and 5 LEO shells (100-2000km)\n"
        L"5. Set exposure=1.35 and ACES tonemapping in PS\n\n"
        L"Web hyperreal already works: open ../index-hyperreal-directx.html in Edge/Chrome (WebGPU → DirectX12).",
        L"IRIS — DirectX12", MB_OK);

    // Minimal message pump
    MSG msg={};
    while(msg.message!=WM_QUIT){
        if(PeekMessage(&msg,nullptr,0,0,PM_REMOVE)){ TranslateMessage(&msg); DispatchMessage(&msg); }
        else {
            // TODO: Update SceneCB (sunDirection, cameraPos, time), render Earth + shells + atmosphere
            // Present with HDR
        }
    }
    return 0;
}
