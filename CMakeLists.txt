####################################
# CMake minimum version and policies
####################################

# mimimum required version 3.15: policy CMP0092
# mimimum required version 3.12: target_compile_features C++ 20
# mimimum required version 3.8:  target_compile_features C++ 17
cmake_minimum_required(VERSION 3.15)

# policy CMP0092 MSVC COMPILE FLAGS -- no warings by default
cmake_policy(SET CMP0092 NEW)


###########################################
# Ease of setup protection on Wintendo OSes
if(MSVC AND NOT VCPKG_TOOLCHAIN)
  message(FATAL_ERROR
    "Enable VCPKG toolchain on MSVC platform\n"
    "  Star over; CMAKE_TOOLCHAIN_FILE definition missing!\n"
    "  i.e.: cmake -DCMAKE_TOOLCHAIN_FILE=<path_to>\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake <path_to>\\source\\"
    )
endif()



##############
# Project name
##############
SET(CANDIDATE_ID "cle019")
project(dte3607_${CANDIDATE_ID} VERSION 0.1 LANGUAGES CXX)



########################################################
## Global compiler spesific compile options and features
########################################################

###################
## Across compilers
SET(PROJECT_COMPILE_FEATURES cxx_std_20)
SET(PROJECT_COMPILE_FEATURES_QT cxx_std_17)

###########################
## Project compiler options
SET(CLANG_COMPILE_OPTIONS
    # REQUIRED ON
    -Weverything -Werror -Wextra -pedantic -pedantic-errors
    # REQUIRED ON

    # Turn off c++ 98 compatibility
    -Wno-c++98-compat -Wno-c++98-compat-pedantic

    # Turn off warning for undocumented code fragments
    -Wno-documentation

#    -Wno-exit-time-destructors -Wno-global-constructors
#    -Wno-extra-semi-stmt
#    -Wno-padded
#    -Wno-redundant-parens
    -Wno-zero-as-null-pointer-constant
#    -Wno-header-hygiene
  )

SET(GCC_COMPILE_OPTIONS
    # REQUIRED ON
    -Wall -Werror -Wextra -pedantic  -pedantic-errors
    # REQUIRED ON
  )

SET(MSVC_COMPILE_OPTIONS
    # REQUIRED ON
    /WX /W4 /permissive-
    /wd4715 # solver_development -- allows possible non-returning control paths
    # REQUIRED ON
  )


##################################################
## Compile options for QT MOC
## (turn off all warnings, not our responsibility)
SET(CLANG_COMPILE_OPTIONS_QT -w )
SET(GCC_COMPILE_OPTIONS_QT -w )
SET(MSVC_COMPILE_OPTIONS_QT /w )


##################################################
## Compile options for GTest targets
## (turn off all warnings, not our responsibility)
SET(CLANG_COMPILE_OPTIONS_GTEST -w )
SET(GCC_COMPILE_OPTIONS_GTEST -w )
SET(MSVC_COMPILE_OPTIONS_GTEST /w )



#########################
## 3rd party dependencies
#########################


# gmlib -- version 2.x -- fetch ref / sub-tree
add_subdirectory(libs/3rdparty/gmlib2)

# gmlib qt integration library -- vesion 2.x -- included in demo-build settings

# blaze  -- get dependency from [gmlib2]






###################################
## Colision library
## alias dte3607::coldet
###################################
add_subdirectory(libs/coldet)






################
## Testing tools
################
option(BUILD_UNITTESTS "Build unittests" OFF)
if(BUILD_UNITTESTS)
  add_subdirectory(testing/unittests)
endif(BUILD_UNITTESTS)






############################
## Application (demos)
############################
option(BUILD_DEMO "Build demo" ON)
if(BUILD_DEMO)
  add_subdirectory(libs/3rdparty/gmlibqtintegration)
  add_subdirectory(apps/demo)
endif(BUILD_DEMO)






###########################################
# Qt creator dummy target
###########################################
option(DUMMY_TARGET "Dummy target..." TRUE)
if(DUMMY_TARGET)
  add_custom_target(${PROJECT_NAME}_DUMMY)
endif(DUMMY_TARGET)


