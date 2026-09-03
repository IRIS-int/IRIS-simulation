cmake_minimum_required(VERSION 3.20)
project(IRIS_DirectX12 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
add_executable(IRIS_DirectX12 IRIS_DirectX12.cpp)
target_link_libraries(IRIS_DirectX12 d3d12 dxgi d3dcompiler)
# Requires Windows SDK 10.0.19041+ and DirectX Agility SDK
# Textures: copy ../libs/*.jpg/png to build folder or set assets path
