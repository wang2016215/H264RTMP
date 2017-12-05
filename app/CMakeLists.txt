

cmake_minimum_required(VERSION 3.4.1)

file(GLOB my_sourc  src/main/cpp/*.cpp   src/main/cpp/*.c )
set(DIR ../../../../libs)
include_directories(libs/include)

add_subdirectory(src/main/cpp/librtmp)

add_library( x264
             STATIC
             IMPORTED)

set_target_properties(x264
                        PROPERTIES IMPORTED_LOCATION
                        ${DIR}/armeabi/libx264.a)

add_library(faac
            STATIC
            IMPORTED)

set_target_properties(faac
                        PROPERTIES IMPORTED_LOCATION
                        ${DIR}/armeabi/libfaac.a)


add_library( # Sets the name of the library.
             native-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/native-lib.cpp )



find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )





target_link_libraries(
                       native-lib


                       ${log-lib}
                        faac
                        x264
                        rtmp)