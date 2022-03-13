cmake_minimum_required(VERSION 3.10)

project(charuco_pose_est)

find_package( OpenCV 4.0 REQUIRED
    PATHS / )

if ( NOT OpenCV_FOUND )
    message( FATAL_ERROR "Could NOT find OpenCV version 4.0 or higher" )
else ()
    message(STATUS "***FOUND*** OpenCV in directory ${OpenCV_DIR}.")
endif ()

#find_package( yaml-cpp REQUIRED )

#if ( NOT yaml-cpp_FOUND )
#    message( FATAL_ERROR "Could NOT find yaml-cpp" )
#else ()
#    message(STATUS "***FOUND*** yaml-cpp in directory ${yaml-cpp_DIR}.")
#endif ()

#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

find_package( Matlab COMPONENTS MX_LIBRARY )

if ( NOT Matlab_FOUND )
    message( FATAL_ERROR "MATLAB root directory not found. Failed to build MATLAB bindings.")
else ()
    message(STATUS "***FOUND*** MATLAB root directory in ${Matlab_ROOT_DIR}.")
endif ()


matlab_add_mex(
    NAME ArucoPixDect
    SRC src/mex_aruco_pixel_detection.cpp src/helper_functions.cpp
    LINK_TO ${OpenCV_LIBS}
    )

target_include_directories(ArucoPixDect PRIVATE "include")


set_target_properties( ArucoPixDect
    PROPERTIES
    COMPILE_FLAGS "-fvisibility=default"
    ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    )



matlab_add_mex(
    NAME CharucoPosEst
    SRC src/mex_charuco_pose_est.cpp src/helper_functions.cpp
    LINK_TO ${OpenCV_LIBS}
    )

target_include_directories(CharucoPosEst PRIVATE "include")

set_target_properties( CharucoPosEst
    PROPERTIES
    COMPILE_FLAGS "-fvisibility=default"
    ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
    )




#if (Matlab_FOUND)




##    matlab_add_mex(
##        NAME ArucoPosEst
##        SRC mex_aruco_pose_est.cpp
##        LINK_TO ${OpenCV_LIBS} ${yaml-cpp_LIBS}
##        )


##    set_target_properties( ArucoPosEst
##        PROPERTIES
##        COMPILE_FLAGS "-fvisibility=default"
##        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
##        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
##        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
##        )


#    message(STATUS "***BUILD LOCATION*** Mex functions can be found in ${PROJECT_SOURCE_DIR}/bin")
#else ()

#    set(BUILD_MATLAB_BINDINGS OFF)
#endif ()
