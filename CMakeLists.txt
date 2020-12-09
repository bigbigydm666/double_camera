cmake_minimum_required(VERSION 3.1)
project(test)
find_package(OpenCV REQUIRED)
add_executable(run test.cpp)
include_directories(/home/ydm/linuxSDK_V2.1.0.20/lib/x86)


include_directories(/home/ydm/linuxSDK_V2.1.0.20/include)
link_directories( /home/ydm/linuxSDK_V2.1.0.20/lib/x86)
target_link_libraries(run ${OpenCV_LIBS})
target_link_libraries(run libMVSDK.so)

