file(GLOB HEADERS include/*)
find_path(PatternFollower_INCLUDE_DIRS ${HEADERS} PATH_SUFFIXES pattern_follower HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../include)
find_library(PatternFollower_LIBRARIES NAMES pattern_follower HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../lib)

MESSAGE(${PatternFollower_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pattern_follower DEFAULT_MSG PatternFollower_LIBRARIES PatternFollower_INCLUDE_DIRS)

find_package(OpenCV REQUIRED)

set(PatternFollower_LIBRARIES ${PATTERNFOLLOWER_LIBRARIES} ${OpenCV_LIBRARIES})
set(PatternFollower_INCLUDE_DIRS ${PATTERNFOLLOWER_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

mark_as_advanced(PatternFollower_LIBRARIES PatternFollower_INCLUDE_DIRS)
