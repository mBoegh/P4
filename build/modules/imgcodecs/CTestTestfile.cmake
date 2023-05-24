# CMake generated Testfile for 
# Source directory: /home/jonas/test_ws/opencv-4.x/modules/imgcodecs
# Build directory: /home/jonas/test_ws/build/modules/imgcodecs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_imgcodecs "/home/jonas/test_ws/build/bin/opencv_test_imgcodecs" "--gtest_output=xml:opencv_test_imgcodecs.xml")
set_tests_properties(opencv_test_imgcodecs PROPERTIES  LABELS "Main;opencv_imgcodecs;Accuracy" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1375;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;181;ocv_add_accuracy_tests;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;0;")
add_test(opencv_perf_imgcodecs "/home/jonas/test_ws/build/bin/opencv_perf_imgcodecs" "--gtest_output=xml:opencv_perf_imgcodecs.xml")
set_tests_properties(opencv_perf_imgcodecs PROPERTIES  LABELS "Main;opencv_imgcodecs;Performance" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/performance" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1274;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;192;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;0;")
add_test(opencv_sanity_imgcodecs "/home/jonas/test_ws/build/bin/opencv_perf_imgcodecs" "--gtest_output=xml:opencv_perf_imgcodecs.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_imgcodecs PROPERTIES  LABELS "Main;opencv_imgcodecs;Sanity" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/sanity" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1275;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;192;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/imgcodecs/CMakeLists.txt;0;")
