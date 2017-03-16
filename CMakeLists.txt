project(Ori)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

find_package(TinyXML REQUIRED)
include_directories(${TinyXML_INCLUDE_DIRS})

set(Ori_DEFINITIONS ${Ori_DEFINITIONS} ${TinyXML_DEFINITIONS})
set(Ori_LIBRARIES Ori ${TinyXML_LIBRARY})

option(USE_PBRT "Use Pbrt ray-tracing lib" OFF)
if(USE_PBRT)
    find_package(PBRT REQUIRED)
    if(PBRT_FOUND)
        MESSAGE(STATUS "Found PBRT")
        INCLUDE_DIRECTORIES(${PBRT_INCLUDE_DIRS})
        set(Ori_DEFINITIONS ${Ori_DEFINITIONS} -dUSE_PBRT)
        set(Ori_LIBRARIES ${Ori_LIBRARIES} ${PBRT_LIBRARIES})
    else(PBRT_FOUND)
        message(STATUS "Optionnal lib PBRT not Found")
    endif(PBRT_FOUND)
    find_package(OpenCV REQUIRED)
    message(STATUS "Found OpenCV")
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LINK_DIRECTORIES})
set(Ori_LIBRARIES ${Ori_LIBRARIES} ${OpenCV_LIBRARIES})
endif(USE_PBRT)

add_definitions(${Ori_DEFINITIONS})

set(CPPS
    src/Ori.cpp
    src/ExtrinsicModel.cpp
    src/ConicModel.cpp
    src/SphericModel.cpp
    src/DistortionPolynom.cpp
    src/xml.cpp
)

set(HPPS
    src/Ori.hpp
    src/ExtrinsicModel.hpp
    src/IntrinsicModel.hpp
    src/ConicModel.hpp
    src/SphericModel.hpp
    src/DistortionPolynom.hpp
    src/xml.hpp
)

add_library(Ori ${CPPS} ${HPPS})
add_executable(testOri test/main.cpp)
target_link_libraries(testOri ${Ori_LIBRARIES})
if(USE_PBRT)
    add_executable(testPBRT test/mainPBRT.cpp)
    target_link_libraries(testPBRT ${Ori_LIBRARIES})
endif(USE_PBRT)

##############################################################################
# Install
##############################################################################
set(Ori_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
install(FILES ${HPPS} DESTINATION "${Ori_INCLUDE_DIR}/Ori")

install(TARGETS Ori
  EXPORT Ori-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install (EXPORT Ori-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
configure_file(OriConfig.cmake.in ${CMAKE_BINARY_DIR}/OriConfig.cmake.in2 @ONLY)
configure_file(${CMAKE_BINARY_DIR}/OriConfig.cmake.in2 ${CMAKE_BINARY_DIR}/OriConfig.cmake @ONLY)
install (FILES ${CMAKE_BINARY_DIR}/OriConfig.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)

