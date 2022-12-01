function(add_ons_plugin name)
    cmake_parse_arguments(_arg
            ""
            ""
            "SOURCES;DEPENDS;PUBLIC_INCLUDES;INCLUDES;PUBLIC_DEPENDS;LIB_DEPENDS"
            ${ARGN})

    add_library(${name} SHARED ${_arg_SOURCES})

    target_include_directories(${name}
            PRIVATE
            ${_arg_INCLUDES}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            )

    get_filename_component(public_build_interface_dir "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    target_include_directories(${name} PUBLIC ${_arg_PUBLIC_INCLUDES} ${public_build_interface_dir})

    target_link_libraries(${name} PRIVATE ${_arg_DEPENDS} ${_DEP_PLUGINS} ${_arg_LIB_DEPENDS})

    target_link_libraries(${name} PUBLIC ${_arg_PUBLIC_DEPENDS})

endfunction()

function(add_ons_test name_)
    cmake_parse_arguments(_arg
            ""
            ""
            "SOURCES;DEPENDS;PUBLIC_INCLUDES;INCLUDES;PLUGIN_DEPENDS;LIB_DEPENDS"
            ${ARGN})

    set(name ${name_}_test)

    add_executable(${name} ${_arg_SOURCES})

    #    target_sources(${name} PRIVATE ${_arg_SOURCES})

    target_include_directories(${name} PRIVATE ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE benchmark::benchmark)
    target_link_libraries(${name} PRIVATE gtest gtest_main gmock)

    target_include_directories(${name}
            PRIVATE
            ${_arg_INCLUDES}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            )

    get_filename_component(public_build_interface_dir "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    target_include_directories(${name} PUBLIC ${_arg_PUBLIC_INCLUDES} ${public_build_interface_dir})

    target_link_libraries(${name} PRIVATE ${_arg_DEPENDS} ${_DEP_PLUGINS} ${_arg_LIB_DEPENDS})

endfunction()