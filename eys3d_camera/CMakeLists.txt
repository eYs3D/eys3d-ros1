cmake_minimum_required(VERSION 3.10)
project(eys3d_camera VERSION 1.0.0)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

# ===== Per-ABI dispatch + native scheduling tune =====
#
# The driver targets two ABIs, matching the prebuilt eSPDI .so set:
#   - x86_64  → libeSPDI_X86_64.so,    SSE2 baseline (ABI-mandated)
#   - aarch64 → libeSPDI_NVIDIA_64.so, NEON baseline (ABI-mandated)
#
# SIMD path selection is done at compile time via the `__aarch64__` /
# `__x86_64__` macros (see src/simd_kernels.cpp); no CMake-side branching.
#
#   -fno-math-errno -fno-trapping-math
#     Semantically safe on this driver (no errno-after-libm reads, no
#     SIGFPE handler installed); enable broader folding and vectorisation.
#
#   -mtune=native           (native builds only)
#     Adjusts GCC's scheduler model for the host CPU. The instruction set
#     is unchanged, so the binary stays portable across the ABI. Skipped
#     when cross-compiling, where the host CPU does not reflect the target.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fno-math-errno -fno-trapping-math)
    if(NOT CMAKE_CROSSCOMPILING)
        add_compile_options(-mtune=native)
    endif()
endif()

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    diagnostic_msgs
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_generation
    nodelet
    pluginlib
    roscpp
    roslib
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    topic_tools
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# The self-calibration action. ROS 1 generates messages in-package, so the
# action lives here rather than in a separate interfaces package.
add_action_files(
    DIRECTORY action
    FILES SelfCal.action
)
generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs
)

# Runtime image/filter controls, served over dynamic_reconfigure.
generate_dynamic_reconfigure_options(
    cfg/EYS3DCamera.cfg
)

# OpenMP — parallelises the depth → XYZ reprojection across image rows.
# Bundled with GCC's runtime (libgomp); no separate apt package needed.
# The reprojection falls back to a serial loop when it is not detected.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found: parallel point-cloud reprojection enabled")
else()
    message(STATUS "OpenMP not found: point-cloud reprojection will run single-threaded")
endif()

# eSPDI ships in-package under eSPDI/ — header plus prebuilt .so for
# x86_64 and aarch64. The package is self-contained; no external SDK
# installation is required.
set(ESPDI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/eSPDI")

if(NOT EXISTS "${ESPDI_ROOT}/eSPDI.h")
    message(FATAL_ERROR
        "eSPDI headers not found at ${ESPDI_ROOT}. "
        "The SDK should ship in-package under eSPDI/.")
endif()

# Target architecture, not host — cross-compiling from x86_64 to aarch64
# (e.g. for a Jetson image) should pick the aarch64 .so.
#
# EYS3D_ARCH is the directory name the in-package trees use (uname -m:
# x86_64 / aarch64), normalised from whatever alias the toolchain reports.
# The eSPDI .so, the selfk archive, and the OpenCL loader stub all key off
# it, so it is resolved once here rather than per-block — an arch derived
# inside an optional block would be empty whenever that block is skipped.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR
   CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(EYS3D_ARCH "x86_64")
    set(ESPDI_LIB_NAME "libeSPDI_X86_64.so")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR
       CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(EYS3D_ARCH "aarch64")
    set(ESPDI_LIB_NAME "libeSPDI_NVIDIA_64.so")
else()
    message(FATAL_ERROR
        "Unsupported target arch '${CMAKE_SYSTEM_PROCESSOR}'. "
        "eSPDI ships prebuilt libs only for x86_64 and aarch64.")
endif()

# Resolve the versioned SDK binary (libeSPDI_<arch>.so.<x.y.z.w>). The tree
# carries no symlinks, so source archives stay buildable. COMPARE NATURAL
# needs CMake 3.18; 18.04 ships 3.10 and 20.04 ships 3.16.
file(GLOB _espdi_libs "${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
if(NOT _espdi_libs)
    message(FATAL_ERROR "libeSPDI not found: ${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    list(SORT _espdi_libs COMPARE NATURAL)
else()
    list(SORT _espdi_libs)
    list(LENGTH _espdi_libs _espdi_lib_count)
    if(_espdi_lib_count GREATER 1)
        message(WARNING
            "Several libeSPDI binaries in ${ESPDI_ROOT}; CMake ${CMAKE_VERSION} "
            "cannot order them by version, so the newest may not be picked.")
    endif()
endif()
list(GET _espdi_libs -1 ESPDI_LIB)
message(STATUS "${PROJECT_NAME}: linking ${ESPDI_LIB}")

# Emit DT_RPATH (rather than the default DT_RUNPATH) so dlopen still
# resolves linked libraries when LD_LIBRARY_PATH is unset. lld does not
# accept the flag, so skip it there.
if(CMAKE_CXX_COMPILER_LINKER_ID STREQUAL "LLD")
    set(EYS3D_CAMERA_RPATH_FLAGS "")
else()
    set(EYS3D_CAMERA_RPATH_FLAGS "-Wl,--disable-new-dtags")
endif()

# APC error-code name table, generated from the SDK header so runtime
# error messages carry the define name (APC_NOT_SUPPORT_RES) instead of
# a bare rc. Regenerates automatically when the SDK header changes.
find_package(PythonInterp REQUIRED)
set(ESPDI_ERROR_INC "${CMAKE_CURRENT_BINARY_DIR}/generated/espdi_error_names.inc")
add_custom_command(
    OUTPUT "${ESPDI_ERROR_INC}"
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND ${PYTHON_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_espdi_error_names.py"
            "${ESPDI_ROOT}/eSPDI_def.h" "${ESPDI_ERROR_INC}"
    DEPENDS "${ESPDI_ROOT}/eSPDI_def.h"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_espdi_error_names.py"
    COMMENT "Generating APC error-code name table"
)
add_custom_target(gen_espdi_error_names DEPENDS "${ESPDI_ERROR_INC}")
# The table is written at build time, but include_directories() below is
# evaluated at configure time and a non-existent entry is an error there.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_nodelet
    CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        diagnostic_msgs
        diagnostic_updater
        dynamic_reconfigure
        geometry_msgs
        message_runtime
        nodelet
        pluginlib
        roscpp
        roslib
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_ros
        topic_tools
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ESPDI_ROOT}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# eYs3D self-calibration (selfk) core, bundled in-package under selfk/: the
# public header plus a static archive per architecture, linked into the
# driver. Set -DEYS3D_WITH_SELFCAL=OFF to build without it.
option(EYS3D_WITH_SELFCAL "Build with self-calibration (selfk) support" ON)
if(EYS3D_WITH_SELFCAL)
    set(SELFK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/selfk")
    # The archives ship under the same normalised arch directory as the
    # eSPDI .so — see EYS3D_ARCH above.
    set(SELFK_PREBUILT
        "${SELFK_DIR}/lib/${EYS3D_ARCH}/libeys3d_selfk_${EYS3D_ARCH}.a")
    # Build rules that only apply where the sources they build are present.
    file(GLOB _selfk_local "${CMAKE_CURRENT_SOURCE_DIR}/cmake/local/selfk_*.cmake")
    if(_selfk_local)
        foreach(_rule IN LISTS _selfk_local)
            include("${_rule}")
        endforeach()
    elseif(EXISTS "${SELFK_PREBUILT}")
        add_library(eys3d_selfk STATIC IMPORTED)
        set_target_properties(eys3d_selfk PROPERTIES
            IMPORTED_LOCATION "${SELFK_PREBUILT}"
            INTERFACE_INCLUDE_DIRECTORIES "${SELFK_DIR}/include"
            # The archive was built against the in-package eSPDI; the final
            # .so must resolve those symbols.
            INTERFACE_LINK_LIBRARIES "${ESPDI_LIB}")
    else()
        message(FATAL_ERROR
            "EYS3D_WITH_SELFCAL is ON but no self-calibration archive for "
            "${CMAKE_SYSTEM_PROCESSOR} was found (${SELFK_PREBUILT}). Set "
            "-DEYS3D_WITH_SELFCAL=OFF to build without it, or ask eYs3D for an "
            "archive built for this architecture.")
    endif()
endif()

# The driver core, built once as a nodelet. The standalone node below is a
# thin main() around the nodelet loader, so both entry points run the exact
# same code path.
set(EYS3D_CAMERA_SOURCES
    src/camera_nodelet.cpp
    src/camera_nodelet_control.cpp
    src/espdi_device.cpp
    src/hole_filling.cpp
    src/register_settings.cpp
    src/selfcal_manager.cpp
    src/simd_kernels.cpp
    src/spatial_filter.cpp
    src/temporal_filter.cpp
    src/video_modes.cpp
    src/zd_lookup.cpp
)

add_library(${PROJECT_NAME}_nodelet ${EYS3D_CAMERA_SOURCES})
add_dependencies(${PROJECT_NAME}_nodelet
    gen_espdi_error_names
    ${PROJECT_NAME}_gencfg
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${ESPDI_LIB}    # also exports libjpeg-turbo 2.0.4 (tjDecompress2 etc.)
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
)
if(EYS3D_WITH_SELFCAL)
    target_link_libraries(${PROJECT_NAME}_nodelet eys3d_selfk)
    target_compile_definitions(${PROJECT_NAME}_nodelet PRIVATE EYS3D_WITH_SELFCAL)
    target_include_directories(${PROJECT_NAME}_nodelet PRIVATE ${SELFK_DIR}/include)
endif()
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_nodelet OpenMP::OpenMP_CXX)
endif()

# BUILD_RPATH is set explicitly: CMake 3.10 does not derive it from the
# linked library's location, leaving the devel-space nodelet unable to
# dlopen libeSPDI. INSTALL_RPATH covers the install tree, where the SDK
# sits next to the library.
set_target_properties(${PROJECT_NAME}_nodelet PROPERTIES
    LINK_FLAGS "${EYS3D_CAMERA_RPATH_FLAGS}"
    BUILD_RPATH "${ESPDI_ROOT}"
    INSTALL_RPATH "\$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH FALSE
)

add_executable(${PROJECT_NAME}_node src/camera_node_main.cpp)
add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    LINK_FLAGS "${EYS3D_CAMERA_RPATH_FLAGS}"
    INSTALL_RPATH "\$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# catkin_make configures every package in the workspace in a single CMake
# pass, so target names share one global namespace: an unprefixed
# "perf_monitor" would collide with any other package defining the same
# target and fail the whole workspace at configure time. OUTPUT_NAME keeps
# `rosrun eys3d_camera perf_monitor` working.
add_executable(${PROJECT_NAME}_perf_monitor src/perf_monitor.cpp)
set_target_properties(${PROJECT_NAME}_perf_monitor PROPERTIES
    OUTPUT_NAME perf_monitor
)
add_dependencies(${PROJECT_NAME}_perf_monitor
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_perf_monitor ${catkin_LIBRARIES})

# Rewrites the bundled RViz layout for a renamed camera and execs rviz. No
# link libraries: it reads argv and a file, and calls no ROS API.
add_executable(${PROJECT_NAME}_rviz_launcher src/rviz_launcher.cpp)
set_target_properties(${PROJECT_NAME}_rviz_launcher PROPERTIES
    OUTPUT_NAME rviz_launcher
)

install(TARGETS ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_perf_monitor
                ${PROJECT_NAME}_rviz_launcher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# The SDK runtime is installed beside the driver library so the $ORIGIN
# RPATH above resolves it.
install(FILES "${ESPDI_LIB}"
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# The SDK is linked with NEEDED libOpenCL.so.1; ship the loader stub it
# expects, keeping the SDK's $ORIGIN-relative neighbour layout.
install(FILES "${ESPDI_ROOT}/OpenCL/${EYS3D_ARCH}/lib/libOpenCL.so.1"
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Standard catkin launch-file check: parses every launch file and verifies
# every referenced package, node type and file resolves. Run with
# `catkin_make run_tests_eys3d_camera`.
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
    roslaunch_add_file_check(launch/examples)
endif()

install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg config launch meshes rviz udev urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
