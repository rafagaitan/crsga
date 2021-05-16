add_executable(onemax onemax.cpp)
target_link_libraries(onemax
                      PRIVATE crsGA)
install(TARGETS onemax
        RUNTIME DESTINATION bin/examples)