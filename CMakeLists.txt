cmake_minimum_required(VERSION 3.10)

project(charuco_pose_est)

find_package( OpenCV 4.0 REQUIRED )

#find_package( yaml-cpp 0.6 REQUIRED )

if ( NOT OpenCV_FOUND )
    message( FATAL_ERROR "Could NOT find OpenCV version 4.0 or higher" )
else ()
    message(STATUS "***FOUND*** OpenCV in directory ${OpenCV_DIR}.")
endif ()

find_package( Matlab REQUIRED MX_LIBRARY )

#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

if ( NOT Matlab_FOUND )
    message( FATAL_ERROR "MATLAB root directory not found. Failed to build MATLAB bindings.")
else ()
    message(STATUS "***FOUND*** MATLAB root directory in ${Matlab_ROOT_DIR}.")
endif ()

# ArUco pixel detection
matlab_add_mex(
    NAME ArucoPixDect
    SRC src/mex_aruco_pixel_detection.cpp
    LINK_TO ${OpenCV_LIBS}
    R2018a
    )

target_include_directories(ArucoPixDect PRIVATE "include")

set_target_properties( ArucoPixDect
    PROPERTIES
    COMPILE_FLAGS "-fvisibility=default"
    ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    )


# ChArUco pose estimation
matlab_add_mex(
    NAME CharucoPosEst
    SRC src/mex_charuco_pose_est.cpp
    LINK_TO ${OpenCV_LIBS}
    R2018a
    )

target_include_directories(CharucoPosEst PRIVATE "include")

set_target_properties( CharucoPosEst
    PROPERTIES
    COMPILE_FLAGS "-fvisibility=default"
    ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    )

# ArUco pose estimation
#matlab_add_mex(
#    NAME ArucoPosEst
#    SRC mex_aruco_pose_est.cpp
#    LINK_TO ${OpenCV_LIBS} yaml-cpp
#    )

#target_include_directories(CharucoPosEst PRIVATE "include")

#set_target_properties( ArucoPosEst
#    PROPERTIES
#    COMPILE_FLAGS "-fvisibility=default"
#    ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
#    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
#    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
#    )

