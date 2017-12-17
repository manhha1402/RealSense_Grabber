project(realsense_object)
cmake_minimum_required(VERSION 2.8.11)
# build with c++11
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/") #find libraries

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/") #find tinyxml.h

#Librealsense 
find_library(Realsense_LIBS realsense2)



set(CPP_FILES main.cpp realsense.cpp)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
${TinyXML_INCLUDE_DIRS}
common/include
)
add_definitions(${PCL_DEFINITIONS})
set(project_CPP_FILES
        common/src/features.cpp
        common/src/filters.cpp
        common/src/registrator.cpp
        common/src/util.cpp
        common/src/segmentation.cpp
        common/src/aruco_marker.cpp
        common/src/loader.cpp
        common/src/matrix_utils.cpp
        common/src/realsense.cpp
        common/src/RobustMatcher.cpp
        common/src/config.cpp
)
set(pi_tag_CPP_FILES
        common/src/AbstractFiducialModel.cpp
        common/src/FiducialModelPi.cpp
        common/src/ellipserefine.cpp
)
add_library(pi_tag ${pi_tag_CPP_FILES})
target_link_libraries(pi_tag
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen_INCLUDE_DIRS}
)
set_target_properties(pi_tag PROPERTIES COMPILE_FLAGS "-D__LINUX__")

#Executable for realsense camera
add_executable(realsense src/main.cpp ${project_CPP_FILES} )
target_link_libraries(realsense pi_tag
                        ${Realsense_LIBS} )
set_target_properties(realsense PROPERTIES COMPILE_FLAGS "-D__LINUX__")
#add executable for creating model
add_executable(model src/main_model.cpp ${project_CPP_FILES} )
target_link_libraries(model ${OpenCV_LIBS} ${TinyXML_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_INCLUDE_DIRS}
                             ${Realsense_LIBS} ${DEPENDENCIES})



