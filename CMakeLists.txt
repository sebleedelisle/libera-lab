cmake_minimum_required(VERSION 3.15)

# Pull version from git tag (e.g. v0.3.3 → 0.3.3), fallback to 0.0.0
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_TAG_RESULT
)
if(GIT_TAG_RESULT EQUAL 0 AND GIT_TAG MATCHES "^v?([0-9]+\\.[0-9]+\\.[0-9]+)")
    set(PROJECT_VERSION_FROM_GIT "${CMAKE_MATCH_1}")
else()
    set(PROJECT_VERSION_FROM_GIT "0.0.0")
endif()

project(libera-lab VERSION ${PROJECT_VERSION_FROM_GIT} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    add_compile_options("$<$<CONFIG:Release>:/Zi>")
    add_link_options("$<$<CONFIG:Release>:/DEBUG:FULL>")
endif()

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
    GIT_TAG        v1.92.7-docking
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
    find_library(APPKIT_FRAMEWORK AppKit REQUIRED)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/LiberaApp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LiberaPluginsWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/OscilloscopeWindow.cpp
    $<$<PLATFORM_ID:Darwin>:${CMAKE_CURRENT_SOURCE_DIR}/LiberaFileDialog_macos.mm>
    $<$<NOT:$<PLATFORM_ID:Darwin>>:${CMAKE_CURRENT_SOURCE_DIR}/LiberaFileDialog_other.cpp>
    ${CMAKE_CURRENT_SOURCE_DIR}/ILDAParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/RobotoMedium.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/RobotoBold.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/ForkAwesome.cpp
)

if (APPLE)
    target_link_libraries(libera-lab PRIVATE ${APPKIT_FRAMEWORK})
endif()

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
target_compile_definitions(libera-lab PRIVATE APP_VERSION="${PROJECT_VERSION}")

if (MSVC)
    target_compile_options(libera-lab PRIVATE /W4 /permissive- /FS)
else()
    target_compile_options(libera-lab PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- Clear app settings on build ----
if (APPLE)
    set(_settings_dir "$ENV{HOME}/Library/Application Support/LiberaLab")
elseif (WIN32)
    set(_settings_dir "$ENV{APPDATA}/LiberaLab")
else()
    if (DEFINED ENV{XDG_CONFIG_HOME})
        set(_settings_dir "$ENV{XDG_CONFIG_HOME}/LiberaLab")
    else()
        set(_settings_dir "$ENV{HOME}/.config/LiberaLab")
    endif()
endif()
add_custom_target(clear-settings ALL
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${_settings_dir}"
    COMMENT "Clearing LiberaLab settings folder"
)
add_dependencies(libera-lab clear-settings)
