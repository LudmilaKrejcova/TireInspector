cmake_minimum_required(VERSION 3.14)
project(TireInspector)

include(FetchContent)

# Fetch local SDL3
FetchContent_Declare(
    SDL3
    SOURCE_DIR "C:/libs/SDL-release-3.2.24" 
)
FetchContent_MakeAvailable(SDL3)

# Fetch local geGL
FetchContent_Declare(
    geGL
    SOURCE_DIR "C:/libs/geGL-master"
)
FetchContent_MakeAvailable(geGL)

# Use the prebuilt shared OpenCV libraries directly
set(OpenCV_STATIC OFF)
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)
# ---------------------------------------------------------
# 1. DEFINE THE TARGET FIRST
# ---------------------------------------------------------
set(sourceFiles D3D.cpp)
add_executable(TireInspector ${sourceFiles})
target_compile_features(TireInspector PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(TireInspector PRIVATE /utf-8)
endif()

# ---------------------------------------------------------
# 2. CONFIGURE THE TARGET
# ---------------------------------------------------------
# Add include directories
target_include_directories(TireInspector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(TireInspector PRIVATE C:/libs/glm-master)

# Link libraries (SDL3 instead of SDL2)
target_link_libraries(TireInspector PRIVATE SDL3::SDL3 geGL::geGL ${OpenCV_LIBS})

# ---------------------------------------------------------
# 3. POST-BUILD ACTIONS
# ---------------------------------------------------------
# Copy SDL3 DLL to the executable folder
add_custom_command(TARGET TireInspector POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:SDL3-shared>
    $<TARGET_FILE_DIR:TireInspector>
)

# Automatically copy the OpenCV world DLL to the executable folder
add_custom_command(TARGET TireInspector POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:opencv_world>
    $<TARGET_FILE_DIR:TireInspector>
)