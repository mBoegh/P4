# CMake generated Testfile for 
# Source directory: /home/jonas/test_ws/opencv-4.x/modules/photo
# Build directory: /home/jonas/test_ws/build/modules/photo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_photo "/home/jonas/test_ws/build/bin/opencv_test_photo" "--gtest_output=xml:opencv_test_photo.xml")
set_tests_properties(opencv_test_photo PROPERTIES  LABELS "Main;opencv_photo;Accuracy" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1375;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1133;ocv_add_accuracy_tests;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;7;ocv_define_module;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;0;")
add_test(opencv_perf_photo "/home/jonas/test_ws/build/bin/opencv_perf_photo" "--gtest_output=xml:opencv_perf_photo.xml")
set_tests_properties(opencv_perf_photo PROPERTIES  LABELS "Main;opencv_photo;Performance" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/performance" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1274;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1134;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;7;ocv_define_module;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;0;")
add_test(opencv_sanity_photo "/home/jonas/test_ws/build/bin/opencv_perf_photo" "--gtest_output=xml:opencv_perf_photo.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_photo PROPERTIES  LABELS "Main;opencv_photo;Sanity" WORKING_DIRECTORY "/home/jonas/test_ws/build/test-reports/sanity" _BACKTRACE_TRIPLES "/home/jonas/test_ws/opencv-4.x/cmake/OpenCVUtils.cmake;1763;add_test;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1275;ocv_add_test_from_target;/home/jonas/test_ws/opencv-4.x/cmake/OpenCVModule.cmake;1134;ocv_add_perf_tests;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;7;ocv_define_module;/home/jonas/test_ws/opencv-4.x/modules/photo/CMakeLists.txt;0;")
