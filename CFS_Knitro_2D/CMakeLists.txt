# make a directory build in KTR, go there and run “cmake ..”.
# may need to specify generator in Windows:
# e.g.: for Visual Studio 2013 on x64, run cmake -G "Visual Studio 12 Win64" ..
# need separate build folders for Win32 and Win64
# The makefile will be created there. It uses KNITRODIR environment variable.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

PROJECT(CFS_Knitro)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES($ENV{KNITRODIR}/include)

SET(KTR_RELEASE 1021)
SET(KTR_RELEASE_LINUX 10.2.1)
SET(KNITRO_LIB libknitro.so.${KTR_RELEASE_LINUX})

IF(APPLE)
  SET(KNITRO_LIB "libknitro.dylib")
ENDIF(APPLE)

IF(WIN32)
  SET(KNITRO_LIB "knitro${KTR_RELEASE}.lib")
ENDIF(WIN32)

#ADD_EXECUTABLE( MyQP src/MyQP.cpp )
#TARGET_LINK_LIBRARIES(MyQP $ENV{KNITRODIR}/lib/${KNITRO_LIB}  ${CMAKE_DL_LIBS})

ADD_EXECUTABLE( MyCFS src/MyCFS.cpp )
TARGET_LINK_LIBRARIES(MyCFS $ENV{KNITRODIR}/lib/${KNITRO_LIB}  ${CMAKE_DL_LIBS})

ADD_EXECUTABLE( AutoTestCFS src/AutoTestCFS.cpp )
TARGET_LINK_LIBRARIES(AutoTestCFS $ENV{KNITRODIR}/lib/${KNITRO_LIB}  ${CMAKE_DL_LIBS})

ADD_EXECUTABLE( AutoTestNLNC src/AutoTestNLNC.cpp )
TARGET_LINK_LIBRARIES(AutoTestNLNC $ENV{KNITRODIR}/lib/${KNITRO_LIB}  ${CMAKE_DL_LIBS})

ADD_EXECUTABLE( MyNLNC src/MyNLNC.cpp )
TARGET_LINK_LIBRARIES(MyNLNC $ENV{KNITRODIR}/lib/${KNITRO_LIB}  ${CMAKE_DL_LIBS})
