FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library (LIMAWBC_Ctrl SHARED ${headers} ${sources} )
target_link_libraries (LIMAWBC_Ctrl biomimetics lcm)
target_link_libraries (LIMAWBC_Ctrl LIMAWBIC dynacore_param_handler)

