set(TEST_APP_NAME ${APP_NAME}_tests)
file(GLOB TEST_SOURCES
     "*.cpp"
)

include_directories(${CPPUTEST_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../sources)
link_directories(${CPPUTEST_LIBRARIES})

# Build the unit tests objects and link then with the app library
add_executable(${TEST_APP_NAME} ${TEST_SOURCES})
target_link_libraries(${TEST_APP_NAME} ${APP_LIB_NAME} ${CPPUTEST_LDFLAGS})

# Run the test once the build is done
add_custom_command(TARGET ${TEST_APP_NAME} COMMAND ./${TEST_APP_NAME} -o junit -v POST_BUILD)