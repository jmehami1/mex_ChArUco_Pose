cmake_minimum_required(VERSION 3.10)

project(charuco_pose_est)

find_package( Matlab COMPONENTS MX_LIBRARY )
find_package( OpenCV REQUIRED )

#include_directories("${PROJECT_SOURCE_DIR}/bin" )
#link_directories( /home/jesse/Documents/mexopencv-3.2.0/lib )

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/BIN")
#add_subdirectory("${PROJECT_SOURCE_DIR}/bin")
#set(
#    CMAKE_RUNTIME_OUTPUT_DIRECTORY
#    ${PROJECT_SOURCE_DIR}/bin
#    )

if (Matlab_FOUND)
    message(STATUS "MATLAB root directory found: ${Matlab_ROOT_DIR}.")
    matlab_add_mex(
        NAME CharucoPosEst
        SRC mex_charuco_pose_est.cpp
        LINK_TO ${OpenCV_LIBS}
        )

#    TARGET_LINK_LIBRARIES( CharucoPosEst MxArray)

    set_target_properties( CharucoPosEst
        PROPERTIES
        #COMPILE_FLAGS "-fvisibility=default"
        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

#    add_custom_command(TARGET CharucoPosEst
#        POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        $<TARGET_FILE:CharucoPosEst>
#        $<TARGET_FILE_DIR:bin>/$<TARGET_FILE_NAME:library1>)

message(STATUS "${PROJECT_SOURCE_DIR}/bin")



    # copy MATLAB .m files to binary directory
    #    file(COPY .
    #            DESTINATION .
    #            FILES_MATCHING
    #            PATTERN *.m)
else ()
    message(WARNING "MATLAB root directory not found. Failed to build MATLAB bindings.")
    set(BUILD_MATLAB_BINDINGS OFF)
endif ()
