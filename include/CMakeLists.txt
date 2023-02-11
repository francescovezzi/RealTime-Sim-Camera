add_library(allegro_lib ${SRC_PATH}/allegro_functions.c ${INC_PATH}/allegro_functions.h)
add_library(thread_lib ${SRC_PATH}/thread_functions.c ${INC_PATH}/thread_functions.h)

target_link_libraries(allegro_lib PUBLIC ${ALLEG_LINK_FLAGS} m)
target_link_libraries(thread_lib PUBLIC pthread)

target_include_directories(allegro_lib PUBLIC ${INC_PATH})
target_include_directories(thread_lib PUBLIC ${INC_PATH})