# - Try to find PBRT
# Once done this will define
#  
#  PBRT_FOUND        - system has PBRT
#  PBRT_INCLUDE_DIR  - the PBRT include directory
#  PBRT_LIBRARY      - Link these to use PBRT
#   

IF (PBRT_INCLUDE_DIRS)
  # Already in cache, be silent
  SET(PBRT_FIND_QUIETLY TRUE)
ENDIF (PBRT_INCLUDE_DIRS)

FIND_PATH( PBRT_INCLUDE_DIR core/pbrt.h
           PATHS "/usr/include" "/usr/local/include" "C:/libs/pbrt/include" .. ../..
           PATH_SUFFIXES /pbrt-v2/src /library/pbrt-v2/src)     

if( WIN32 )

 FIND_LIBRARY( PBRT_LIBRARY
               NAMES pbrt.lib
               PATHS "C:/libs/pbrt/lib")
               
 # Store the library dir. May be used for linking to dll!
 GET_FILENAME_COMPONENT( PBRT_LIBRARY_DIR ${PBRT_LIBRARY} PATH )
 
else (WIN32)

 FIND_LIBRARY( PBRT_LIBRARY
               NAMES PBRT pbrt
               PATHS "/usr/lib" "/usr/local/lib" .. ../..
               PATH_SUFFIXES /pbrt-v2/src/objs /library/pbrt-v2/src/objs)

endif( WIN32)


IF (PBRT_INCLUDE_DIR AND PBRT_LIBRARY)
  SET(PBRT_LIBRARIES ${PBRT_LIBRARY} Half IlmImf IlmImfUtil IlmThread Iex Imath pthread)
  SET(PBRT_INCLUDE_DIRS ${PBRT_INCLUDE_DIR} ${PBRT_INCLUDE_DIR}/core)
  SET(PBRT_FOUND TRUE)
ELSE (PBRT_INCLUDE_DIR AND PBRT_LIBRARY)
  SET(PBRT_FOUND FALSE)
ENDIF (PBRT_INCLUDE_DIR AND PBRT_LIBRARY)
