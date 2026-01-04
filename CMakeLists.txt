cmake_minimum_required(VERSION 3.14)
project(tvmc VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================
# BiBi-Sync Library
# ============================================

# Path to bibi-sync-rust build output
set(BIBI_SYNC_ROOT "${CMAKE_SOURCE_DIR}/../bibi-sync-rust")
set(BIBI_SYNC_INCLUDE "${BIBI_SYNC_ROOT}/include")
set(BIBI_SYNC_LIB_DIR "${BIBI_SYNC_ROOT}/target/release")

# Check if the library exists
if(APPLE)
    set(BIBI_SYNC_LIB_NAME "libbibi_sync.dylib")
elseif(UNIX)
    set(BIBI_SYNC_LIB_NAME "libbibi_sync.so")
elseif(WIN32)
    set(BIBI_SYNC_LIB_NAME "bibi_sync.dll")
endif()

if(NOT EXISTS "${BIBI_SYNC_LIB_DIR}/${BIBI_SYNC_LIB_NAME}")
    message(FATAL_ERROR 
        "BiBi-Sync library not found at ${BIBI_SYNC_LIB_DIR}/${BIBI_SYNC_LIB_NAME}\n"
        "Please build bibi-sync-rust first:\n"
        "  cd ${BIBI_SYNC_ROOT} && cargo build --release"
    )
endif()

# Create imported target for BiBi-Sync
add_library(bibi_sync SHARED IMPORTED)
set_target_properties(bibi_sync PROPERTIES
    IMPORTED_LOCATION "${BIBI_SYNC_LIB_DIR}/${BIBI_SYNC_LIB_NAME}"
    INTERFACE_INCLUDE_DIRECTORIES "${BIBI_SYNC_INCLUDE}"
)

# ============================================
# libInterpolate (for PWM conversion)
# ============================================
find_package(libInterpolate REQUIRED)

# ============================================
# Thruster Config Library
# ============================================
add_library(thruster_config
    src/thruster-config/thruster_config.cpp
)

target_include_directories(thruster_config PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/config
)

# ============================================
# TVMC Executable
# ============================================
add_executable(tvmc
    src/motion-controller/controller.cpp
    src/reporters/thrust.cpp
    src/PID-Controller/PID_controller.cpp
)

target_include_directories(tvmc PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${BIBI_SYNC_INCLUDE}
)

target_link_libraries(tvmc PRIVATE
    bibi_sync
    thruster_config
    pthread
)

# ============================================
# PWM Reporter Executable
# ============================================
add_executable(pwmc
    src/reporters/pwm.cpp
)

target_include_directories(pwmc PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${BIBI_SYNC_INCLUDE}
)

target_link_libraries(pwmc PRIVATE
    bibi_sync
    thruster_config
    libInterpolate::Interpolate
    pthread
)

# ============================================
# Installation
# ============================================
install(TARGETS tvmc pwmc
    RUNTIME DESTINATION bin
)

install(DIRECTORY config/
    DESTINATION share/tvmc/config
    FILES_MATCHING 
    PATTERN "*.json"
    PATTERN "*.csv"
)

# ============================================
# Print Configuration
# ============================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "TVMC Build Configuration")
message(STATUS "========================================")
message(STATUS "  BiBi-Sync library: ${BIBI_SYNC_LIB_DIR}/${BIBI_SYNC_LIB_NAME}")
message(STATUS "  BiBi-Sync include: ${BIBI_SYNC_INCLUDE}")
message(STATUS "========================================")
message(STATUS "")
