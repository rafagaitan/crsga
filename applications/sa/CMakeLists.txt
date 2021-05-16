set(sources
    sa.cpp)

set(headers)

add_executable(sa ${sources} ${headers})

target_link_libraries(sa
                      PRIVATE crsGA 
                              CONAN_PKG::openscenegraph)
install(TARGETS sa
        RUNTIME DESTINATION bin/sa)

# install thirdparty dependencies
install_target_thirdparty(sa)
