# Dependencies
# PCL, OpenCV2.4, g2o, CSparse
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # for Ubuntu16.04

FIND_PACKAGE(OpenCV 2.4.9 REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O REQUIRED)

FIND_PACKAGE(CSparse REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

# add include files and library files
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

# build library slambase
ADD_LIBRARY(slambase slamBase.cpp)
TARGET_LINK_LIBRARIES(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})

# build library parameter reader
ADD_LIBRARY(parameterReader parameterReader.cpp)
TARGET_LINK_LIBRARIES(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES})

# build slam
ADD_EXECUTABLE(slam slam.cpp)
TARGET_LINK_LIBRARIES(slam slambase
                           parameterReader
                           ${OpenCV_LIBS} 
                           ${PCL_LIBRARIES}
                           g2o_core g2o_types_slam3d g2o_solver_csparse 
                           g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})
