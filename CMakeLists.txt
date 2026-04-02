cmake_minimum_required(VERSION 3.15)
project(libera-lab VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- libera-core (submodule) ----
add_subdirectory(libs/libera-core)

# ---- GLFW (windowing) ----
include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ---- Dear ImGui (UI) ----
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# Build ImGui as a library with GLFW + OpenGL3 backends
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)

# OpenGL
if (APPLE)
    find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
    target_link_libraries(imgui_lib PUBLIC ${OPENGL_FRAMEWORK})
elseif (WIN32)
    target_link_libraries(imgui_lib PUBLIC opengl32)
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui_lib PUBLIC OpenGL::GL)
endif()

# ---- Libera Lab executable ----
add_executable(libera-lab
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ILDAParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/RobotoMedium.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/RobotoBold.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/ForkAwesome.cpp
)

# ---- macOS app bundle ----
if (APPLE)
    set(MACOSX_BUNDLE_ICON icons/libera-lab.icns)
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${MACOSX_BUNDLE_ICON} PROPERTIES
        MACOSX_PACKAGE_LOCATION "Resources"
    )
    target_sources(libera-lab PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/${MACOSX_BUNDLE_ICON})

    set_target_properties(libera-lab PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME "Libera Lab"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.sebleedelisle.libera-lab"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_ICON_FILE "libera-lab.icns"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist.in"
    )

    # Copy patterns into the app bundle Resources folder
    file(GLOB ILDA_PATTERNS "${CMAKE_CURRENT_SOURCE_DIR}/patterns/*.ild")
    set_source_files_properties(${ILDA_PATTERNS} PROPERTIES
        MACOSX_PACKAGE_LOCATION "Resources/patterns"
    )
    target_sources(libera-lab PRIVATE ${ILDA_PATTERNS})
else()
    # Windows icon
    if (WIN32)
        target_sources(libera-lab PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/icons/libera-lab.rc)
    endif()

    # Copy patterns next to the executable
    file(GLOB ILDA_PATTERNS "${CMAKE_CURRENT_SOURCE_DIR}/patterns/*.ild")
    foreach(PATTERN_FILE ${ILDA_PATTERNS})
        get_filename_component(PATTERN_NAME ${PATTERN_FILE} NAME)
        configure_file(${PATTERN_FILE}
            ${CMAKE_CURRENT_BINARY_DIR}/patterns/${PATTERN_NAME} COPYONLY)
    endforeach()
endif()

target_include_directories(libera-lab PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(libera-lab PRIVATE libera-core imgui_lib)

if (MSVC)
    target_compile_options(libera-lab PRIVATE /W4 /permissive- /FS)
else()
    target_compile_options(libera-lab PRIVATE -Wall -Wextra -Wpedantic)
endif()
