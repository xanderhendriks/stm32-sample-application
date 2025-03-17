set(TEST_APP_NAME ${APP_NAME}_mocks)
file(GLOB MOCK_SOURCES
     "*.cpp"
)

include_directories(${CPPUTEST_INCLUDE_DIRS} ../src/)

add_library(${TEST_APP_NAME} ${MOCK_SOURCES})
