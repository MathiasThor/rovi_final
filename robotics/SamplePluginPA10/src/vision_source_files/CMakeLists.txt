ADD_EXECUTABLE(tracker main.cpp color_detector.cpp corny_detector.cpp general_functions.cpp)
target_link_libraries(tracker ${OpenCV_LIBS} opencv_xfeatures2d)
