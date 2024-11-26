cmake_minimum_required(VERSION 3.10)
project(AutofocusExe)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DBUILD_DLL)

# Цель для сборки .dll с использованием MSVC (или другого Windows компилятора)
if(MSVC)
    message("MSVC COMPILER BUILD")

    add_definitions(-D_WIN32_WINNT=0x0A00)
    # Boost
    set(BOOST_ROOT "C:/MinGW/include/boost")
    find_package(Boost REQUIRED)
    include_directories(${Boost_INCLUDE_DIR})
    # OpenCV 
    include_directories("C:/opencv/build-msvc/install/include")
    set(OpenCV_LIB_DIR "C:/opencv/build-msvc/install/lib")
    set(OpenCV_LIBS "${OpenCV_LIB_DIR}/opencv_world480.lib")

    add_library(autofocus SHARED src/autofocus.cpp)
    add_executable(autofocus_exe src/autofocus.cpp)
    add_executable(autofocusdlltest src/autofocusdlltest.cpp)

    target_link_libraries(autofocus ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_link_libraries(autofocus_exe ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_link_libraries(autofocusdlltest ${OpenCV_LIBS} ${Boost_LIBRARIES})

    set_target_properties(autofocus PROPERTIES OUTPUT_NAME "autofocus")
    set_target_properties(autofocus_exe PROPERTIES OUTPUT_NAME "autofocus_exe")
    set_target_properties(autofocusdlltest PROPERTIES OUTPUT_NAME "autofocusdlltest")

# Цель для сборки .so с использованием MinGW или GCC для Linux
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU")
    message("GNU COMPILER BUILD")

    # Boost
    set(BOOST_ROOT "/usr/include/boost")
    find_package(Boost REQUIRED)
    include_directories(${Boost_INCLUDE_DIR})
    # OpenCV
    set(OpenCV_DIR "/data/opencv_4.8.0/build")
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    add_library(autofocus SHARED src/autofocus.cpp)
    add_executable(autofocus_exe src/autofocus.cpp)
    add_executable(autofocusdlltest src/autofocussotest.cpp)

    target_link_libraries(autofocus ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_link_libraries(autofocus_exe ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_link_libraries(autofocusdlltest ${OpenCV_LIBS} ${Boost_LIBRARIES})

    set_target_properties(autofocus PROPERTIES OUTPUT_NAME "autofocus")
    set_target_properties(autofocus_exe PROPERTIES OUTPUT_NAME "autofocus_exe")
    set_target_properties(autofocusdlltest PROPERTIES OUTPUT_NAME "autofocusdlltest")
endif()