# gw_embed_web(<target>)
#
# Builds the React app under web/ and embeds the resulting dist/ tree into the
# given CMake target via CMakeRC. Defines GW_HAS_EMBEDDED_WEB=1 on the target.
#
# Requires npm on PATH and the cmrc package already made available via
# FetchContent (provides cmrc_add_resource_library).
function(gw_embed_web target)
    find_program(NPM_EXECUTABLE npm REQUIRED)

    set(WEB_DIR  "${CMAKE_SOURCE_DIR}/web")
    set(WEB_DIST "${WEB_DIR}/dist")

    message(STATUS "gw_embed_web: building React bundle in ${WEB_DIR}")

    execute_process(
        COMMAND ${NPM_EXECUTABLE} install --no-audit --no-fund
        WORKING_DIRECTORY ${WEB_DIR}
        RESULT_VARIABLE npm_install_result
    )
    if(NOT npm_install_result EQUAL 0)
        message(FATAL_ERROR "gw_embed_web: 'npm install' failed in ${WEB_DIR}")
    endif()

    execute_process(
        COMMAND ${NPM_EXECUTABLE} run build
        WORKING_DIRECTORY ${WEB_DIR}
        RESULT_VARIABLE npm_build_result
    )
    if(NOT npm_build_result EQUAL 0)
        message(FATAL_ERROR "gw_embed_web: 'npm run build' failed in ${WEB_DIR}")
    endif()

    file(GLOB_RECURSE WEB_FILES_REL RELATIVE ${WEB_DIST} ${WEB_DIST}/*)
    if(NOT WEB_FILES_REL)
        message(FATAL_ERROR "gw_embed_web: no files produced under ${WEB_DIST}")
    endif()

    set(WEB_FILES_ABS)
    foreach(f ${WEB_FILES_REL})
        list(APPEND WEB_FILES_ABS "${WEB_DIST}/${f}")
    endforeach()

    cmrc_add_resource_library(gw_web_assets
        NAMESPACE gw_web_assets
        WHENCE    ${WEB_DIST}
        ${WEB_FILES_ABS}
    )

    target_link_libraries(${target} PRIVATE gw_web_assets)
    target_compile_definitions(${target} PRIVATE GW_HAS_EMBEDDED_WEB=1)
endfunction()
