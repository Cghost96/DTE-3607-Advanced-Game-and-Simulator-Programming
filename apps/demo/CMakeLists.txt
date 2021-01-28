get_filename_component(Current_Folder_Name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${Current_Folder_Name})
project(${ProjectId} VERSION 0.1 LANGUAGES CXX)
add_executable(${PROJECT_NAME})

###############
# Sources Files

### Translation Units
set(${PROJECT_NAME}_SOURCES
  guiapplication.cpp
  main.cpp
  qmlapi.cpp
  scenariomodel.cpp
  scenariolistmodel.cpp
  simulationcontroller.cpp
  simulationloopworker.cpp
  limitedplanegeometry.cpp
  )

### Qt MOC HEADERS
set( ${PROJECT_NAME}_MOC_HEADERS
  guiapplication.h
  qmlapi.h
  scenariomodel.h
  scenariolistmodel.h
  simulationcontroller.h
  simulationloopworker.h
  limitedplanegeometry.h
  )

### Qt Resources -- "RESOURCES_QML"
set( ${PROJECT_NAME}_RESOURCES_QML_PREFIX "/" )
set( ${PROJECT_NAME}_RESOURCES_QML_FILES
    "resources/qml/main.qml"
    "resources/qml/components/BlenderIconImage.qml"
    "resources/qml/gui/ApplicationWindowFooterToolBar.qml"
    "resources/qml/gui/ApplicationWindowHeaderToolBar.qml"
    "resources/qml/gui/CustomView3D.qml"
    "resources/qml/gui/ScenarioListModelViewOverlay.qml"
    "resources/qml/gui/WorkSpace01.qml"
    "resources/qml/gui/WorkSpace02.qml"
    "resources/qml/gui/WorkSpace03.qml"
    "resources/qml/scenes/DefaultLighting.qml"
    "resources/qml/scenes/DefaultCameras.qml"
    "resources/qml/scenes/ScenarioModelView01.qml"
    "resources/qml/scenes/ScenarioModelView02.qml"
    "resources/qml/scenes/Scene01.qml"
    "resources/qml/scenes/Scene02.qml"
    "resources/qml/scenes/Scene03.qml"
  )

### Qt Resources -- "RESOURCES_GFX" -- small (size)
set( ${PROJECT_NAME}_RESOURCES_GFX_PREFIX "/" )
set( ${PROJECT_NAME}_RESOURCES_GFX_FILES
    "resources/gfx/uit_logo/uit_logo_eng_2l_hvit.png"
    "resources/gfx/uit_logo/uit_logo_eng_2l_bla_rgb.png"
    "resources/gfx/uit_logo/uit_logo_eng_bla_rgb.png"
    "resources/gfx/uit_logo/uit_logo_eng_hvit.png"
    "resources/gfx/uit_logo/uit_segl_eng_bla_960px.png"
    "resources/gfx/uit_logo/uit_segl_eng_hvit_960px.png"
    "resources/gfx/uit_logo/uit_segl_eng_sort_960px.png"
    "resources/gfx/quick3d_quickball_example/ball.jpg"
    "resources/gfx/quick3d_quickball_example/ball_n.jpg"
    "resources/gfx/quick3d_quickball_example/grass.jpg"
    "resources/gfx/quick3d_quickball_example/grass_n.jpg"
  )

### Qt Resources -- "RESOURCES_SHADERS" -- small (size)
set( ${PROJECT_NAME}_RESOURCES_SHADERS_PREFIX "/" )
set( ${PROJECT_NAME}_RESOURCES_SHADERS_FILES
    "resources/shaders/quick3d_custommaterial_example/material_customlights.frag"
    "resources/shaders/quick3d_custommaterial_example/material_customspecular.frag"
    "resources/shaders/quick3d_custommaterial_example/material_distortion.vert"
    "resources/shaders/quick3d_custommaterial_example/material_transparent.frag"
  )

### Qt Resources -- "RESOURCES_BLENDER28_ICONS" -- also smaal
set( ${PROJECT_NAME}_RESOURCES_BLENDER28_ICONS_PREFIX "/" )
set( ${PROJECT_NAME}_RESOURCES_BLENDER28_ICONS_FILES
    "resources/icons/blender2.8/blender2.8_icons_2019-06-04.svg"
    "resources/icons/blender2.8/a22.png" # svg load fails on wintendo
    "resources/icons/blender2.8/a23.png" # svg load fails on wintendo
    "resources/icons/blender2.8/aa2.png" # svg load fails on wintendo
    "resources/icons/blender2.8/aa3.png" # svg load fails on wintendo
    "resources/icons/blender2.8/aa6.png" # svg load fails on wintendo
    "resources/icons/blender2.8/b4.png"  # svg load fails on wintendo
    "resources/icons/blender2.8/d17.png" # svg load fails on wintendo
    "resources/icons/blender2.8/h20.png" # svg load fails on wintendo
    "resources/icons/blender2.8/j4.png"  # svg load fails on wintendo
    "resources/icons/blender2.8/z1.png"  # svg load fails on wintendo
    "resources/icons/blender2.8/z8.png"  # svg load fails on wintendo
  )


###################
### Compile options

# C++ standard feature set
target_compile_features(${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:Clang>:${PROJECT_COMPILE_FEATURES}>
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:${PROJECT_COMPILE_FEATURES}>
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:${PROJECT_COMPILE_FEATURES}>
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
find_package(Qt6 COMPONENTS Core Gui Quick Quick3D QuickControls2 REQUIRED)

# Add a libaray for Qt compiled stuff -- we don't control
add_library(${PROJECT_NAME}_QT_OBJS OBJECT)

# Compilation settings for Qt object library (TURN ON FEATURES - OFF WARNINGS)
target_compile_features(${PROJECT_NAME}_QT_OBJS
  PRIVATE $<$<CXX_COMPILER_ID:Clang>:${PROJECT_COMPILE_FEATURES_QT}>
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:${PROJECT_COMPILE_FEATURES_QT}>
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:${PROJECT_COMPILE_FEATURES_QT}>
  )


target_compile_options(${PROJECT_NAME}_QT_OBJS
  PRIVATE $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS_QT} >
  PRIVATE $<$<CXX_COMPILER_ID:GNU>: ${GCC_COMPILE_OPTIONS_QT} >
  PRIVATE $<$<CXX_COMPILER_ID:MSVC>: ${MSVC_COMPILE_OPTIONS_QT} >
  )

set_target_properties(${PROJECT_NAME}_QT_OBJS PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(${PROJECT_NAME}_QT_OBJS PRIVATE
  dte3607::coldet
  gmlib::gmlib
  gmlib::gmlibqtintegration
  Qt::Core
  Qt::Gui
  Qt::Quick
  Qt::Quick3D
  Qt::QuickControls2
  )

# Compile Qt MOC
qt_wrap_cpp( ${PROJECT_NAME}_MOC_SOURCES
  ${${PROJECT_NAME}_MOC_HEADERS}
  )
# add qt moc sources
target_sources(${PROJECT_NAME}_QT_OBJS PRIVATE
  ${${PROJECT_NAME}_MOC_SOURCES}
  )

# Compile Qt QML Resources
qt_add_resources( ${PROJECT_NAME}_QT_OBJS "RESOURCES_QML"
  PREFIX
    ${${PROJECT_NAME}_RESOURCES_QML_PREFIX}
  FILES
    ${${PROJECT_NAME}_RESOURCES_QML_FILES}
  )

# Compile Qt Shader Resources
qt_add_resources( ${PROJECT_NAME}_QT_OBJS "RESOURCES_SHADERS"
  PREFIX
    ${${PROJECT_NAME}_RESOURCES_SHADERS_PREFIX}
  FILES
    ${${PROJECT_NAME}_RESOURCES_SHADERS_FILES}
  )

# Compile GFX Resources (SMALL IN SIZE -- WILL BE PART OF EXE)
qt_add_resources( ${PROJECT_NAME}_QT_OBJS "RESOURCES_GFX"
  PREFIX
    ${${PROJECT_NAME}_RESOURCES_GFX_PREFIX}
  FILES
    ${${PROJECT_NAME}_RESOURCES_GFX_FILES}
  )

# Compile Blender 2.8 Resources (SMALL IN SIZE -- WILL BE PART OF EXE)
qt_add_resources( ${PROJECT_NAME}_QT_OBJS "BLENDER28_ICONS"
  PREFIX
    ${${PROJECT_NAME}_RESOURCES_BLENDER28_ICONS_PREFIX}
  FILES
    ${${PROJECT_NAME}_RESOURCES_BLENDER28_ICONS_FILES}
  )


# Compile Qt QC2 (SPECIAL) Resource
qt_add_resources( ${PROJECT_NAME}_QT_OBJS "RESOURCES_QC2"
  PREFIX "/" FILES "qtquickcontrols2.conf" )


###############################
### Link our exe with libraries

# add sources
target_sources(${PROJECT_NAME} PRIVATE
  ${${PROJECT_NAME}_SOURCES}
  )

# link executable
target_link_libraries( ${PROJECT_NAME} PRIVATE
  dte3607::coldet
  gmlib::gmlib
  gmlib::gmlibqtintegration
  $<$<CXX_COMPILER_ID:Clang>:-llapack>      # blaze forgets this on some linux boxes in our current buildfile setup
  $<$<CXX_COMPILER_ID:GNU>:-llapack>        # blaze forgets this on some linux boxes in our current buildfile setup
  $<TARGET_OBJECTS:${PROJECT_NAME}_QT_OBJS>
  Qt::Core
  Qt::Gui
  Qt::Quick
  Qt::Quick3D
  )
