
# Enable testing
enable_testing()

# Configure threads
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

if(GTest_FOUND)

  # Predefined unittests
  set( PREDEFINED_UNITTESTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/predef )
  set( PREDEFINED_UNITTESTS
    predef_mech_compute_trajectory
    predef_mech_sp_vs_fpl_detection
    predef_mech_sp_vs_fpl_response
    predef_mech_sp_vs_sp_detection
    predef_mech_sp_vs_sp_response
    predef_solver_dev_step1
    predef_solver_dev_step2
    predef_solver_dev_step3a
    predef_solver_dev_step3b
    )

  # My unittests
  set( MYUNITTESTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/my )
  set( MYUNITTESTS
    unit_hello_mechanics
    unit_hello_mysolvers
    )



  # Turn off static analysis for benchmark tool (no compilation warnings)
#  target_compile_options( GTest::GTest
#    INTERFACE $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS_GTEST}>
#    INTERFACE $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS_GTEST}>
#    INTERFACE $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS_GTEST}>
#    )

#  target_compile_options( GTest::Main
#    INTERFACE $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS_GTEST}>
#    INTERFACE $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS_GTEST}>
#    INTERFACE $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS_GTEST}>
#    )

  macro(DTE3607_ADD_UNITTESTS UNITTEST_DIR)

    # Predefined Unittests : week 1
    foreach(TEST ${ARGN})

      set( TESTNAME ${TEST} )
      add_executable( ${TESTNAME} )

      target_sources( ${TESTNAME}
        PRIVATE ${UNITTEST_DIR}/${TEST}.cpp ${SRCS} )

      target_link_libraries( ${TESTNAME}
        PRIVATE
          dte3607::coldet
          gmlib::gmlib
          $<$<CXX_COMPILER_ID:Clang>:-llapack>      # blaze forgets this on some linux boxes in our current buildfile setup
          $<$<CXX_COMPILER_ID:GNU>:-llapack>        # blaze forgets this on some linux boxes in our current buildfile setup
          GTest::GTest GTest::Main
          Threads::Threads )

      # C++ standard feature set
      target_compile_features( ${TESTNAME}
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:${PROJECT_COMPILE_FEATURES}>
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:${PROJECT_COMPILE_FEATURES}>
        PRIVATE $<$<CXX_COMPILER_ID:MSVC>:${PROJECT_COMPILE_FEATURES}>
        )

      # Compile definitions
      target_compile_definitions( ${TESTNAME}
        PRIVATE $<$<CXX_COMPILER_ID:MSVC>: _USE_MATH_DEFINES>
        )

      # Comple options
      target_compile_options( ${TESTNAME}
        PRIVATE $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS}>
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS}>
        PRIVATE $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS}>
        )

      # Turn off platform-spesific extensions
      set_target_properties(${TESTNAME} PROPERTIES CXX_EXTENSIONS OFF)

      gtest_discover_tests( ${TESTNAME} "" AUTO )

    endforeach(TEST)

  endmacro(DTE3607_ADD_UNITTESTS)

  DTE3607_ADD_UNITTESTS( ${PREDEFINED_UNITTESTS_SRC_DIR} ${PREDEFINED_UNITTESTS} )
  DTE3607_ADD_UNITTESTS( ${MYUNITTESTS_SRC_DIR} ${MYUNITTESTS} )

### Blaze when it forgets lapac
#target_link_libraries(${PROJECT_NAME} PUBLIC -llapack)

endif(GTest_FOUND)

