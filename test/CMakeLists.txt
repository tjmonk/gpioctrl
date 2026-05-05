cmake_minimum_required(VERSION 3.10)

project(gpioctrl_tests C)

# Unity test framework
add_library(unity STATIC
    unity/unity.c
)
target_include_directories(unity PUBLIC unity)

# Mock libraries
add_library(mock_gpiod STATIC mocks/mock_gpiod.c)
target_include_directories(mock_gpiod PUBLIC mocks)

add_library(mock_varserver STATIC mocks/mock_varserver.c)
target_include_directories(mock_varserver PUBLIC mocks)

add_library(mock_tjson STATIC mocks/mock_tjson.c)
target_include_directories(mock_tjson PUBLIC mocks)

# Test executable - parsing tests
add_executable(test_parse_input test_parse_input.c)
target_include_directories(test_parse_input PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)
target_link_libraries(test_parse_input
    unity
    mock_gpiod
    mock_varserver
    mock_tjson
    pthread
)

# Test executable - runtime logic tests
add_executable(test_runtime test_runtime.c)
target_include_directories(test_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)
target_link_libraries(test_runtime
    unity
    mock_gpiod
    mock_varserver
    mock_tjson
    pthread
)

# Register with CTest
enable_testing()
add_test(NAME test_parse_input COMMAND test_parse_input)
add_test(NAME test_runtime COMMAND test_runtime)
