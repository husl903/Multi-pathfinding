add_executable(stonesngems_test test.cpp)
target_link_libraries(stonesngems_test PUBLIC stonesngems)
add_test(stonesngems_test stonesngems_test)

add_executable(sng_test_speed test_speed.cpp)
target_compile_options(sng_test_speed PUBLIC 
    -Wall -Wextra 
    $<$<CONFIG:RELEASE>:-O3> $<$<CONFIG:RELEASE>:-DNDEBUG>
    $<$<CONFIG:DEBUG>:-O1> $<$<CONFIG:DEBUG>:-g> $<$<CONFIG:DEBUG>:-DDEBUG> $<$<CONFIG:DEBUG>:-pg> 
    $<$<CONFIG:DEBUG>:-ggdb> $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer>
)
target_link_options(sng_test_speed PUBLIC 
    $<$<CONFIG:DEBUG>:-pg> $<$<CONFIG:DEBUG>:-ggdb> $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer>
)
target_link_libraries(sng_test_speed PUBLIC stonesngems)
add_test(sng_test_speed sng_test_speed)

add_executable(sng_test_throughput test_throughput.cpp)
target_compile_options(sng_test_throughput PUBLIC 
    -Wall -Wextra 
    $<$<CONFIG:RELEASE>:-O3> $<$<CONFIG:RELEASE>:-DNDEBUG>
    $<$<CONFIG:DEBUG>:-O1> $<$<CONFIG:DEBUG>:-g> $<$<CONFIG:DEBUG>:-DDEBUG> $<$<CONFIG:DEBUG>:-pg> 
    $<$<CONFIG:DEBUG>:-ggdb> $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer>
)
target_link_options(sng_test_throughput PUBLIC 
    $<$<CONFIG:DEBUG>:-pg> $<$<CONFIG:DEBUG>:-ggdb> $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer>
)
target_link_libraries(sng_test_throughput PUBLIC stonesngems)
add_test(sng_test_throughput sng_test_throughput)

add_executable(sng_test_default test_default.cpp)
target_link_libraries(sng_test_default PUBLIC stonesngems)
add_test(sng_test_default sng_test_default)

add_executable(sng_test_indices test_find_indices.cpp)
target_link_libraries(sng_test_indices PUBLIC stonesngems)
add_test(sng_test_indices sng_test_indices)