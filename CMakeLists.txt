cmake_minimum_required(VERSION 3.10)

project(camera LANGUAGES C VERSION 1.0)


###############################################################################################
###############################################################################################


set(SRC_PATH ${PROJECT_SOURCE_DIR}/src)
set(INC_PATH ${PROJECT_SOURCE_DIR}/include)
set(SOURCES ${SRC_PATH}/camera.c ${SRC_PATH}/allegro_functions.c ${SRC_PATH}/thread_functions.c)
set(HEADER_FILES ${INC_PATH}/allegro_functions.h ${INC_PATH}/thread_functions.h)

execute_process (COMMAND bash -c "allegro-config --libs"
                 OUTPUT_STRIP_TRAILING_WHITESPACE
                 OUTPUT_VARIABLE ALLEG_LINK_FLAGS)

###############################################################################################
###############################################################################################


add_subdirectory(include)
add_subdirectory(src)

add_executable(camera ${SOURCES} ${HEADER_FILES})

target_include_directories(camera PUBLIC
                           ${PROJECT_SOURCE_DIR}/include
                          )

target_link_libraries(camera PUBLIC allegro_lib thread_lib)
target_link_libraries(camera PUBLIC m rt pthread)
