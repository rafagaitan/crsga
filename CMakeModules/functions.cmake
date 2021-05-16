function(setup_conan)
    include(conan)

    # third-party dependencies note: https://github.com/conan-io/cmake-conan
    conan_check(VERSION 1.33 REQUIRED)

    if(MULTI_CONFIG)
        message(STATUS "Multi config generator detected: using conanbuildinfo_multi.cmake")
        set(conan_configs_to_create ${CMAKE_CONFIGURATION_TYPES})
        set(conan_cmake_file conanbuildinfo_multi.cmake)
        set(conan_generator cmake_multi)
        set(conan_setup_additional_options)
    else()
        message(STATUS "Single config generator detected: using conanbuildinfo.cmake")
        set(conan_configs_to_create ${CMAKE_BUILD_TYPE})
        set(conan_cmake_file conanbuildinfo.cmake)
        set(conan_generator cmake)
        set(conan_setup_additional_options NO_OUTPUT_DIRS)
    endif()

    foreach(config ${conan_configs_to_create})
        set(saved_cmake_build_type ${CMAKE_BUILD_TYPE})
        set(CMAKE_BUILD_TYPE ${config})
        conan_cmake_autodetect(settings ARCH x86_64) # force 64bits intel
        conan_cmake_install(PATH_OR_REFERENCE ${CMAKE_CURRENT_SOURCE_DIR}
                            BUILD missing
                            GENERATOR ${conan_generator}
                            SETTINGS ${settings})
        set(CMAKE_BUILD_TYPE ${saved_cmake_build_type})
    endforeach()

    include(${CMAKE_BINARY_DIR}/${conan_cmake_file})
    conan_basic_setup(TARGETS KEEP_RPATHS SKIP_STD ${conan_setup_additional_options})
endfunction()

function(
    install_target_thirdparty
    _target)

    # check target type - should be exe, dll or custom
    get_target_property(
        target_type
        ${_target}
        TYPE)
    set(invalid_targets STATIC_LIBRARY OBJECT_LIBRARY)
    if(${target_type}
       IN_LIST
       invalid_targets)
        return()
    endif()

    # Get all possible conan paths for thirdparty binaries
    set(_thirdparty_paths ${CONAN_LIB_DIRS} ${CONAN_BIN_DIRS})
    set(_dest_dir ${CMAKE_INSTALL_PREFIX}/bin/${_target})
    set(_target_file $<TARGET_FILE:${_target}>)
    set_target_properties(${_target} 
                          PROPERTIES INSTALL_RPATH $ORIGIN)
    # Transfer the vars into the install script
    install(CODE "set(_thirdparty_paths \"${_thirdparty_paths}\")\n
                  set(_target_file \"${_target_file}\")\n
                  set(_dest_dir \"${_dest_dir}\")"
            COMPONENT ${_target})

    install(CODE [[
                file(GET_RUNTIME_DEPENDENCIES
                    EXECUTABLES ${_target_file}
                    RESOLVED_DEPENDENCIES_VAR _r_deps
                    UNRESOLVED_DEPENDENCIES_VAR _u_deps
                    DIRECTORIES ${_thirdparty_paths}
                )
                message(WARNING "deps: ${_r_deps}")
                foreach(_file ${_r_deps})
                    file(INSTALL
                         DESTINATION "${_dest_dir}"
                         TYPE SHARED_LIBRARY
                         FOLLOW_SYMLINK_CHAIN
                         FILES "${_file}"
                    )
                endforeach()
                list(LENGTH _u_deps _u_length)
                if("${_u_length}" GREATER 0)
                    message(WARNING "Unresolved dependencies detected!")
                endif()]]
            COMPONENT ${_target})
endfunction()
