include_directories(../include)

add_executable(simple_2d_tracking simple_2d_tracking.cpp)


find_package( OpenCV REQUIRED )

target_link_libraries(simple_2d_tracking bayes_tracking ${OpenCV_LIBS})
