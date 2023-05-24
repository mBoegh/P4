# CMake generated Testfile for 
# Source directory: /home/jonas/test_ws/opencv-4.x/modules/core
# Build directory: /home/jonas/test_ws/build/modules/core
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_core "/home/jonas/test_ws/build/bin/opencv_test_core" "--gtest_output=xml:opencv_test_core.xml")
set_tests_properties(opencv_test_core PROPERTIES  LABELS "Main;opencv_core;Accuracy" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1375;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;175;ocv_add_accuracy_tests;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;0;")
add_test(opencv_perf_core "/home/jonas/test_ws/build/bin/opencv_perf_core" "--gtest_output=xml:opencv_perf_core.xml")
set_tests_properties(opencv_perf_core PROPERTIES  LABELS "Main;opencv_core;Performance" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/performance" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1274;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;176;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;0;")
add_test(opencv_sanity_core "/home/jonas/test_ws/build/bin/opencv_perf_core" "--gtest_output=xml:opencv_perf_core.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_core PROPERTIES  LABELS "Main;opencv_core;Sanity" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/sanity" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1275;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;176;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/core/CMakeLists.txt;0;")
