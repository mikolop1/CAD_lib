cmake_minimum_required(VERSION 3.10)
project(CADLib VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files
set(CADLIB_SOURCES
    src/Point3D.cpp
    src/Vector3D.cpp
    src/Matrix4x4.cpp
    src/Geometry.cpp
)

# Header files
set(CADLIB_HEADERS
    include/Point3D.h
    include/Vector3D.h
    include/Matrix4x4.h
    include/Geometry.h
)

# Create static library
add_library(CADLib STATIC ${CADLIB_SOURCES} ${CADLIB_HEADERS})

# Set target properties
set_target_properties(CADLib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${CADLIB_HEADERS}"
)

# Include directories for the library
target_include_directories(CADLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optional: Find Qt if available (for future Qt integration)
find_package(Qt6 COMPONENTS Core QUIET)
if(Qt6_FOUND)
    message(STATUS "Qt6 found - Qt integration available")
    target_link_libraries(CADLib PUBLIC Qt6::Core)
    target_compile_definitions(CADLib PUBLIC CADLIB_QT_ENABLED)
else()
    find_package(Qt5 COMPONENTS Core QUIET)
    if(Qt5_FOUND)
        message(STATUS "Qt5 found - Qt integration available")
        target_link_libraries(CADLib PUBLIC Qt5::Core)
        target_compile_definitions(CADLib PUBLIC CADLIB_QT_ENABLED)
    else()
        message(STATUS "Qt not found - building without Qt integration")
    endif()
endif()

# Build examples option
option(BUILD_EXAMPLES "Build example programs" ON)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Install targets
install(TARGETS CADLib
    EXPORT CADLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/CADLib
)

# Install export
install(EXPORT CADLibTargets
    FILE CADLibTargets.cmake
    NAMESPACE CADLib::
    DESTINATION lib/cmake/CADLib
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CADLibConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_file(cmake/CADLibConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/CADLibConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CADLibConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CADLibConfigVersion.cmake"
    DESTINATION lib/cmake/CADLib
)
