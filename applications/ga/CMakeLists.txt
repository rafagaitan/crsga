set(sources
    ga.cpp)

set(headers)

add_executable(ga ${sources} ${headers})

target_link_libraries(ga
                      PRIVATE crsGA 
                              CONAN_PKG::openscenegraph)
install(TARGETS ga
        RUNTIME DESTINATION bin/ga)

# install thirdparty dependencies
install_target_thirdparty(ga)