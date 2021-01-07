cmake_minimum_required(VERSION 2.8.3)
project(bayes_tracking)
set(CPACK_PACKAGE_NAME "bayes_tracking")
set(VERSION "1.0.3")

## Find catkin macros and libraries if installed
find_package(catkin QUIET)
## Use catkin macros and include dirs
if(catkin_FOUND)
  message(STATUS "Found catkin")
  catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
else(catkin_FOUND)
    message(STATUS "NOT found catkin")
endif(catkin_FOUND)

## Headers
include_directories(
    include
)
## Source files
add_library(${PROJECT_NAME} STATIC 
    src/bayes_tracking/associationmatrix.cpp 
    src/bayes_tracking/ekfilter.cpp 
    src/bayes_tracking/ukfilter.cpp 
    src/bayes_tracking/pfilter.cpp 
##    src/bayes_tracking/trackwin.cpp 
    src/bayes_tracking/models.cpp
    src/bayes_tracking/BayesFilter/bayesFltAlg.cpp
    src/bayes_tracking/BayesFilter/bayesFlt.cpp
    src/bayes_tracking/BayesFilter/CIFlt.cpp
    src/bayes_tracking/BayesFilter/covFlt.cpp
    src/bayes_tracking/BayesFilter/infFlt.cpp
    src/bayes_tracking/BayesFilter/infRtFlt.cpp
    src/bayes_tracking/BayesFilter/itrFlt.cpp
    src/bayes_tracking/BayesFilter/matSup.cpp
    src/bayes_tracking/BayesFilter/SIRFlt.cpp
    src/bayes_tracking/BayesFilter/UDFlt.cpp
    src/bayes_tracking/BayesFilter/UdU.cpp
    src/bayes_tracking/BayesFilter/unsFlt.cpp
)

## Link catkin libraries and set install targets
if(catkin_FOUND)
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )
endif(catkin_FOUND)

## add_subdirectory(examples)

## Optional builds: documentation
OPTION (BAYESTRACKING_BUILD_DOC "Generates API documentation" OFF)
if(BAYESTRACKING_BUILD_DOC)
  ## Check if doxygen is even installed
  find_package(Doxygen)
  if (DOXYGEN_FOUND STREQUAL "NO")
      message(FATAL_ERROR "Doxygen not found. Please get a copy http://www.doxygen.org")
  endif (DOXYGEN_FOUND STREQUAL "NO")

  ## Prepare doxygen configuration file
  configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  ## Add doxygen as target
  add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  COMMENT "Generating API documentation with Doxygen" VERBATIM)

  ## Cleanup $build/doc on "make clean"
  set_property(DIRECTORY APPEND PROPERTY
        ADDITIONAL_MAKE_CLEAN_FILES doc)

  ## Install HTML API documentation and manual pages
  set(DOC_PATH "share/doc/${CPACK_PACKAGE_NAME}-${VERSION}")

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html
    DESTINATION ${DOC_PATH}
  )

endif(BAYESTRACKING_BUILD_DOC)
