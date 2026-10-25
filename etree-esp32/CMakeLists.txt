# etree-esp32 - build with Visual Studio: File -> Open -> Folder, pick this folder,
# choose the "x64-Release" configuration, Build -> Build All.
cmake_minimum_required(VERSION 3.20)
project(etree-esp32 CXX)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
  message(FATAL_ERROR "etree-esp32 must be built as 64-bit (x64): libqemu-xtensa.dll is 64-bit")
endif()

set(CMAKE_CXX_STANDARD 20)              # C++/WinRT (bt_pc.cpp)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(etree-esp32 main.cpp bt.cpp bt_pc.cpp bt.h bt_internal.h btlink.h esp32link.h qemu_esp32_api.h)
target_compile_definitions(etree-esp32 PRIVATE _CRT_SECURE_NO_WARNINGS)
# static C runtime: users don't need the VC++ redistributable
set_property(TARGET etree-esp32 PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
