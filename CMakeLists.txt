cmake_minimum_required(VERSION 3.14)
project(radar_coverage_merge 
    VERSION 1.0.0
    DESCRIPTION "Radar coverage area merge with polygon boolean operations"
    LANGUAGES CXX
)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# ============================================================================
# 依赖: Clipper2
# ============================================================================

# 方式1: 尝试查找系统安装的 Clipper2
find_package(Clipper2 QUIET)

if(NOT Clipper2_FOUND)
    message(STATUS "Clipper2 not found, fetching from GitHub...")
    
    # 方式2: 使用 FetchContent 自动下载
    include(FetchContent)
    FetchContent_Declare(
        clipper2
        GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
        GIT_TAG        Clipper2_1.3.0
        SOURCE_SUBDIR  CPP
    )
    
    # 配置 Clipper2 选项
    set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
    set(CLIPPER2_USINGZ "OFF" CACHE STRING "" FORCE)
    
    FetchContent_MakeAvailable(clipper2)
    
    set(CLIPPER2_LIBRARY Clipper2)
else()
    message(STATUS "Found Clipper2: ${Clipper2_DIR}")
    set(CLIPPER2_LIBRARY Clipper2::Clipper2)
endif()

# ============================================================================
# 库: radar_coverage
# ============================================================================

add_library(radar_coverage INTERFACE)
target_include_directories(radar_coverage INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(radar_coverage INTERFACE ${CLIPPER2_LIBRARY})

# ============================================================================
# 可执行文件: 示例程序
# ============================================================================

add_executable(radar_coverage_demo src/main.cpp)
target_link_libraries(radar_coverage_demo PRIVATE radar_coverage)

# ============================================================================
# 测试 (可选)
# ============================================================================

option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    enable_testing()
    
    # 使用 Google Test
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
    
    add_executable(radar_coverage_test tests/test_polygon_boolean.cpp)
    target_link_libraries(radar_coverage_test PRIVATE 
        radar_coverage 
        GTest::gtest_main
    )
    
    include(GoogleTest)
    gtest_discover_tests(radar_coverage_test)
endif()

# ============================================================================
# 安装
# ============================================================================

include(GNUInstallDirs)

install(TARGETS radar_coverage radar_coverage_demo
    EXPORT radar_coverage_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# ============================================================================
# 打印配置信息
# ============================================================================

message(STATUS "")
message(STATUS "=== Radar Coverage Merge Configuration ===")
message(STATUS "  Version:      ${PROJECT_VERSION}")
message(STATUS "  Build type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Clipper2:     ${CLIPPER2_LIBRARY}")
message(STATUS "  Tests:        ${BUILD_TESTS}")
message(STATUS "")
