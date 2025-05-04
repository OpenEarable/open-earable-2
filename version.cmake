file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" version_lines)
foreach(line ${version_lines})
    if(line MATCHES "^VERSION_MAJOR = (.*)$")
        set(VERSION_MAJOR ${CMAKE_MATCH_1})
    elseif(line MATCHES "^VERSION_MINOR = (.*)$")
        set(VERSION_MINOR ${CMAKE_MATCH_1})
    elseif(line MATCHES "^PATCHLEVEL = (.*)$")
        set(VERSION_PATCH ${CMAKE_MATCH_1})
    endif()
endforeach()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/generated/version.h
    @ONLY
)
