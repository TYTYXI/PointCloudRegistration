function(add_ons_lib name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs SOURCES PUBLIC_INCLUDES INCLUDES DEPENDS LIB_DEPENDS
            COMPILE_DEFINITIONS)
    cmake_parse_arguments(_arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(${name} SHARED ${_arg_SOURCES})

    extend_ons_target(${name}
            DEPENDS ${_arg_DEPENDS}
            INCLUDES ${_arg_INCLUDES}
            PUBLIC_INCLUDES ${_arg_PUBLIC_INCLUDES}
            LIB_DEPENDS ${_arg_LIB_DEPENDS}
            COMPILE_DEFINITIONS ${_arg_COMPILE_DEFINITIONS}
    )

endfunction()

function(add_ons_test name_)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPENDS INCLUDES PUBLIC_INCLUDES LIB_DEPENDS PLUGIN_DEPENDS SOURCES
            COMPILE_DEFINITIONS)
    cmake_parse_arguments(_arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


    set(name ${name_}_test)

    add_executable(${name} ${_arg_SOURCES})

    #    target_sources(${name} PRIVATE ${_arg_SOURCES})

    target_include_directories(${name} PRIVATE ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE benchmark::benchmark)
    target_link_libraries(${name} PRIVATE gtest gtest_main gmock)

    extend_ons_target(${name}
            DEPENDS ${_arg_DEPENDS}
            INCLUDES ${_arg_INCLUDES}
            PUBLIC_INCLUDES ${_arg_PUBLIC_INCLUDES}
            LIB_DEPENDS ${_arg_LIB_DEPENDS}
            COMPILE_DEFINITIONS ${_arg_COMPILE_DEFINITIONS}
    )
endfunction()

function(extend_ons_target name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPENDS INCLUDES PUBLIC_INCLUDES LIB_DEPENDS PLUGIN_DEPENDS SOURCES
            COMPILE_DEFINITIONS)
    cmake_parse_arguments(_arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (DEFINED _arg_SOURCES)
        target_sources(${name} PRIVATE ${_arg_SOURCES})
    endif ()

    target_include_directories(${name}
            PRIVATE
            ${_arg_INCLUDES}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_include_directories(${name}
            PRIVATE
            ${_arg_PUBLIC_INCLUDES}
    )

    target_include_directories(${name} PUBLIC ${_arg_PUBLIC_INCLUDES})

    target_link_libraries(${name} PRIVATE ${_arg_DEPENDS} ${_DEP_PLUGINS} ${_arg_LIB_DEPENDS}
            ${_arg_PLUGIN_DEPENDS})

    target_compile_definitions(${name} PRIVATE ${_arg_COMPILE_DEFINITIONS})

    if (DEFINED _arg_PUBLIC_INCLUDES)
        foreach (i ${_arg_PUBLIC_INCLUDES})
            set_property(GLOBAL APPEND PROPERTY ${name}_public_dirs "${CMAKE_CURRENT_SOURCE_DIR}/${i}")
        endforeach ()

        set_property(GLOBAL APPEND PROPERTY ${name}_public_dirs ${CMAKE_CURRENT_SOURCE_DIR})
        set_property(GLOBAL APPEND PROPERTY ${name}_public_dirs ${CMAKE_CURRENT_BINARY_DIR}/${name}_autogen/include)

    else ()

        set_property(GLOBAL APPEND PROPERTY ${name}_public_dirs ${CMAKE_CURRENT_SOURCE_DIR})
        set_property(GLOBAL APPEND PROPERTY ${name}_public_dirs ${CMAKE_CURRENT_BINARY_DIR}/${name}_autogen/include)

    endif ()
    # Set dependency list
    get_property(public_dirs GLOBAL PROPERTY ${name}_public_dirs)

    set_target_properties(${name} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${public_dirs}"
    )

    set_target_properties(${name} PROPERTIES
            INTERFACE_LINK_LIBRARIES "${_arg_DEPENDS}"
    )

endfunction()