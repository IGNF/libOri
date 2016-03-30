project(Ori)
cmake_minimum_required(VERSION 2.6)
find_package(TinyXML REQUIRED)
include_directories(${TinyXML_INCLUDE_DIRS})

set(Ori_DEFINITIONS ${TinyXML_DEFINITIONS})
set(Ori_LIBRARIES Ori ${TinyXML_LIBRARY})

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

