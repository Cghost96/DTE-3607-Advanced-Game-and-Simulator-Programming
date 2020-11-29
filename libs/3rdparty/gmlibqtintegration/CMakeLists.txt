####################
# Configure Wintendo

# IF on Wintendo platform
if(MSVC AND NOT VCPKG_TOOLCHAIN)
  message(FATAL_ERROR
    "Enable VCPKG toolchain on MSVC platform\n"
    "  Star over; CMAKE_TOOLCHAIN_FILE definition missing!\n"
    "  i.e.: cmake -DCMAKE_TOOLCHAIN_FILE=<path_to>\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake <path_to>\\source\\"
    )
endif()



#########
# Project

# Project name
project(gmlibqtintegration)

# Add library target
add_library(${PROJECT_NAME} OBJECT)



###############
# Sources Files

### Translation Units
set(${PROJECT_NAME}_SOURCES

  source/customquick3dgeometry.cpp

  source/curvegeometry.cpp
  source/tensorproductsurfacegeometry.cpp
  source/polygonalsurfacegeometry.cpp

  source/vectorfieldgeometry.cpp
  )

### Qt MOC HEADERS
set( ${PROJECT_NAME}_MOC_HEADERS
  include/gmlibqtintegration/private/customquick3dgeometry.h

  include/gmlibqtintegration/geometry/curvegeometry.h
  include/gmlibqtintegration/geometry/curveintensorproductsurfacegeometry.h
  include/gmlibqtintegration/geometry/curveinpolygonalsurfacegeometry.h

  include/gmlibqtintegration/geometry/tensorproductsurfacegeometry.h
  include/gmlibqtintegration/geometry/tpsurfaceintensorproductsurfacegeometry.h

  include/gmlibqtintegration/geometry/polygonalsurfacegeometry.h

  include/gmlibqtintegration/geometry/vectorfieldgeometry.h

  include/gmlibqtintegration/geometry/curves/linegeometry.h
  include/gmlibqtintegration/geometry/curves/lineintensorproductsurfacegeometry.h
  include/gmlibqtintegration/geometry/curves/lineinpolygonalsurfacegeometry.h

  include/gmlibqtintegration/geometry/surfaces/tensorproduct/planegeometry.h
  include/gmlibqtintegration/geometry/surfaces/tensorproduct/torusgeometry.h
  include/gmlibqtintegration/geometry/surfaces/tensorproduct/spheregeometry.h
  include/gmlibqtintegration/geometry/surfaces/tensorproduct/planeintensorproductsurfacegeometry.h

  include/gmlibqtintegration/geometry/surfaces/polygonal/polygonsurfacegeometry.h
  include/gmlibqtintegration/geometry/surfaces/polygonal/gbpatchgeometry.h
  )







###################
### Compile options

# C++ standard feature set
target_compile_features(${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:Clang>:cxx_std_20>
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:cxx_std_20>
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:cxx_std_20>
  )

# Compile definitions
target_compile_definitions( ${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>: _USE_MATH_DEFINES>
  )

# Comple options
target_compile_options(${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS}>
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS}>
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS}>
  )

# Turn off platform-spesific extensions
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)




###############
# Configure Qt6
set(CMAKE_AUTORCC on)
find_package(Qt6 COMPONENTS Core Gui Quick Quick3D REQUIRED)

# Add a libaray for Qt compiled stuff -- we don't control
add_library(${PROJECT_NAME}_QT_OBJS OBJECT)

# Compilation settings for Qt object library (TURN ON FEATURES - OFF WARNINGS)
target_compile_features(${PROJECT_NAME}_QT_OBJS
  PRIVATE $<$<CXX_COMPILER_ID:Clang>:cxx_std_17>
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:cxx_std_17>
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:cxx_std_17>
  )


target_compile_options(${PROJECT_NAME}_QT_OBJS
  PRIVATE $<$<CXX_COMPILER_ID:Clang>: -w >
  PRIVATE $<$<CXX_COMPILER_ID:GNU>: -w >
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>: /w >
  )

target_link_libraries(${PROJECT_NAME}_QT_OBJS PRIVATE
  gmlib::gmlib
  Qt::Core
  Qt::Gui
  Qt::Quick
  Qt::Quick3D
  )

# Compile Qt MOC
qt_wrap_cpp( ${PROJECT_NAME}_MOC_SOURCES
  ${${PROJECT_NAME}_MOC_HEADERS}
  )
# add qt moc sources
target_sources(${PROJECT_NAME}_QT_OBJS PRIVATE
  ${${PROJECT_NAME}_MOC_SOURCES}
  )






###########################################
### Object library dependencies and linkage

# Header configuration
target_include_directories( ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# add sources
target_sources(${PROJECT_NAME} PRIVATE
  ${${PROJECT_NAME}_SOURCES}
  )

# link executable
target_link_libraries( ${PROJECT_NAME} PRIVATE
  gmlib::gmlib
  $<TARGET_OBJECTS:${PROJECT_NAME}_QT_OBJS>
  Qt::Core
  Qt::Gui
  Qt::Quick
  Qt::Quick3D
  )



##################
# Export targets and configuration
export(TARGETS ${PROJECT_NAME}
  FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}-targets.cmake"
  NAMESPACE gmlib::
  )

# Add alias gmlib2::${PROJECT_NAME} to ${PROJECT_NAME} such that we can reference this
# in the same way as for other consumer usages (external library)
add_library(gmlib::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

