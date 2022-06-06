cmake_minimum_required(VERSION 3.1)
project(hsolo)

# Needed to prevent cmath errors on Windows
add_definitions(-D_USE_MATH_DEFINES)

# Set output directory locations
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/lib")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/lib")

# Configure cmake build types
SET (CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17")
ENDIF() 

# Decide which targets to build
# Only set targets if they are not defined by the command line
IF(NOT DEFINED BUILD_STATIC_LIBRARY)
  set(BUILD_STATIC_LIBRARY ON)
ENDIF()
IF(NOT DEFINED BUILD_EXAMPLE_EXECUTABLE)
  set(BUILD_EXAMPLE_EXECUTABLE ON)
ENDIF()
IF(NOT DEFINED BUILD_PYTHON_WRAPPER)
  set(BUILD_PYTHON_WRAPPER OFF)
ENDIF()
IF(NOT DEFINED BUILD_MATLAB_MEX)
  set(BUILD_MATLAB_MEX OFF)
ENDIF()

# Configure source paths
SET(SOURCE_DIR "src")
SET(SOURCES "${SOURCE_DIR}/HSolo.cpp"
"${SOURCE_DIR}/HSoloUtils.cpp")

# Add Eigen3 requirement
#find_package(Eigen3 REQUIRED)
IF(NOT DEFINED EIGEN3_INCLUDE_DIR)
  find_package(Eigen3 REQUIRED)
ENDIF()
find_package( OpenCV 4.4 REQUIRED )


include_directories("include" SYSTEM ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# Build the static library
IF(BUILD_STATIC_LIBRARY)
  add_library(${PROJECT_NAME} STATIC ${SOURCES})
ENDIF(BUILD_STATIC_LIBRARY)

# Build the example executable
IF(BUILD_EXAMPLE_EXECUTABLE)
  # Generate a test executable
  SET(EXAMPLE_SOURCES ${SOURCES}
  "${SOURCE_DIR}/hsolo_example.cpp")

  include_directories(${OpenCV_INCLUDE_DIRS})

  add_executable("${PROJECT_NAME}_example" ${EXAMPLE_SOURCES})
  target_link_libraries( "${PROJECT_NAME}_example" ${OpenCV_LIBS} )

ENDIF(BUILD_EXAMPLE_EXECUTABLE)

# Build the Python wrapper
IF(BUILD_PYTHON_WRAPPER)
  # Generate python module
  if(NOT TARGET pybind11)
    add_subdirectory(lib/pybind11)
   endif()
  pybind11_add_module(_pyhsolo MODULE ${SOURCES} "${SOURCE_DIR}/python_bindings.cpp")
  target_link_libraries( _pyhsolo PRIVATE ${OpenCV_LIBS} )

set_target_properties(_pyhsolo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/pyhsolo"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/pyhsolo"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/pyhsolo"
)

ENDIF(BUILD_PYTHON_WRAPPER)

# Build the Matlab wrapper
IF(BUILD_MATLAB_MEX)
   SET(MEX_SOURCES ${SOURCES}
      "matlab/findHomographyHSoloMEX.cpp")
   find_package(Matlab)
   matlab_add_mex(NAME findHomographyHSoloMEX SRC ${MEX_SOURCES})
   target_link_libraries( findHomographyHSoloMEX  ${OpenCV_LIBS} )

set_target_properties(findHomographyHSoloMEX
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/matlab"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/matlab"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/matlab"
)
ENDIF(BUILD_MATLAB_MEX)





