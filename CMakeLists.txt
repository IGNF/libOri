project(Ori)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

set(Ori_LIBRARIES Ori)
set(Ori_DEFINITIONS "")

#find_package(TinyXML)
#if(TinyXML_FOUND)
#  message(STATUS "TinyXML found")
#  include_directories(${TinyXML_INCLUDE_DIRS})
#  set(Ori_DEFINITIONS ${Ori_DEFINITIONS} ${TinyXML_DEFINITIONS} "-DHAVE_XML=1")
#  set(Ori_LIBRARIES ${Ori_LIBRARIES}  ${TinyXML_LIBRARY})
#endif()

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(TINYXML tinyxml)
  if(TINYXML_FOUND)
    message(STATUS "Tinyxml found")
    include_directories(${TINYXML_INCLUDE_DIRS})
    set(Ori_DEFINITIONS ${Ori_DEFINITIONS} ${TINYXML_DEFINITIONS} "-DHAVE_XML=1")
    set(Ori_LIBRARIES ${Ori_LIBRARIES} ${TINYXML_LIBRARIES})
  endif(TINYXML_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  if(JSONCPP_FOUND)
    message(STATUS "JSONCPP found")
    include_directories(${JSONCPP_INCLUDE_DIRS})
    set(Ori_DEFINITIONS ${Ori_DEFINITIONS} ${JSONCPP_DEFINITIONS} "-DHAVE_JSON=1")
    set(Ori_LIBRARIES ${Ori_LIBRARIES} ${JSONCPP_LIBRARIES})
  endif(JSONCPP_FOUND)
endif(PKG_CONFIG_FOUND)

find_package(PBRT)
if(PBRT_FOUND)
	MESSAGE(STATUS "PBRT found")
	INCLUDE_DIRECTORIES(${PBRT_INCLUDE_DIRS})
	set(Ori_DEFINITIONS ${Ori_DEFINITIONS} "-DHAVE_PBRT=1")
	set(Ori_LIBRARIES ${Ori_LIBRARIES} ${PBRT_LIBRARIES})
endif(PBRT_FOUND)

find_package(OpenCV)
if(OpenCV_FOUND)
	message(STATUS "OpenCV found")
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_directories(${OpenCV_LINK_DIRECTORIES})
	set(Ori_DEFINITIONS ${Ori_DEFINITIONS} "-DHAVE_OPENCV=1")
	set(Ori_LIBRARIES ${Ori_LIBRARIES} ${OpenCV_LIBRARIES})
endif(OpenCV_FOUND)

add_definitions(${Ori_DEFINITIONS})

set(CPPS
    src/Ori.cpp
    src/ExtrinsicModel.cpp
    src/IntrinsicModel.cpp
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
if(PBRT_FOUND AND OpenCV_FOUND)
    add_executable(testPBRT test/mainPBRT.cpp test/PlyHeaderParser.h test/PlyHeaderParser.cpp )
    target_link_libraries(testPBRT ${Ori_LIBRARIES})
endif(PBRT_FOUND AND OpenCV_FOUND)

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
