get_filename_component(VERSION_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

set(VERSION_FILE "${VERSION_CMAKE_DIR}/VERSION")

file(STRINGS "${VERSION_FILE}" version_lines)
foreach(line ${version_lines})
    if(line MATCHES "^VERSION_MAJOR = (.*)$")
        set(VERSION_MAJOR "${CMAKE_MATCH_1}")
    elseif(line MATCHES "^VERSION_MINOR = (.*)$")
        set(VERSION_MINOR "${CMAKE_MATCH_1}")
    elseif(line MATCHES "^PATCHLEVEL = (.*)$")
        set(VERSION_PATCH "${CMAKE_MATCH_1}")
    elseif(line MATCHES "^VERSION_TWEAK = (.*)$")
        set(VERSION_TWEAK "${CMAKE_MATCH_1}")
    elseif(line MATCHES "^EXTRAVERSION = (.*)$")
        set(VERSION_EXTRA "${CMAKE_MATCH_1}")
    endif()
endforeach()

foreach(required_var VERSION_MAJOR VERSION_MINOR VERSION_PATCH)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "Missing ${required_var} in ${VERSION_FILE}")
    endif()
endforeach()

set(FIRMWARE_BASE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
if(DEFINED VERSION_TWEAK AND NOT VERSION_TWEAK STREQUAL "" AND NOT VERSION_TWEAK STREQUAL "0")
    string(APPEND FIRMWARE_BASE_VERSION ".${VERSION_TWEAK}")
endif()
if(DEFINED VERSION_EXTRA AND NOT VERSION_EXTRA STREQUAL "")
    string(APPEND FIRMWARE_BASE_VERSION "${VERSION_EXTRA}")
endif()

set(FIRMWARE_VERSION "${FIRMWARE_BASE_VERSION}")

function(openearable_resolve_git_dir out_var)
    set(git_path "${VERSION_CMAKE_DIR}/.git")

    if(IS_DIRECTORY "${git_path}")
        set(${out_var} "${git_path}" PARENT_SCOPE)
        return()
    endif()

    if(EXISTS "${git_path}")
        file(READ "${git_path}" git_dir_contents)
        string(REGEX MATCH "gitdir: (.+)" _ "${git_dir_contents}")
        if(CMAKE_MATCH_1)
            set(parsed_git_dir "${CMAKE_MATCH_1}")
            if(IS_ABSOLUTE "${parsed_git_dir}")
                set(resolved_git_dir "${parsed_git_dir}")
            else()
                set(resolved_git_dir "${VERSION_CMAKE_DIR}/${parsed_git_dir}")
            endif()
            file(TO_CMAKE_PATH "${resolved_git_dir}" resolved_git_dir)
            set(${out_var} "${resolved_git_dir}" PARENT_SCOPE)
        endif()
    endif()
endfunction()

function(openearable_track_git_state git_dir)
    if(NOT git_dir OR NOT EXISTS "${git_dir}/HEAD")
        return()
    endif()

    set_property(
        DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${git_dir}/HEAD"
        "${git_dir}/index"
        "${git_dir}/packed-refs"
    )

    file(READ "${git_dir}/HEAD" git_head)
    string(STRIP "${git_head}" git_head)
    if(git_head MATCHES "^ref: (.+)$")
        set_property(
            DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
            "${git_dir}/${CMAKE_MATCH_1}"
        )
    endif()
endfunction()

function(openearable_resolve_pr_suffix out_var)
    set(pr_number "")

    if(DEFINED ENV{GITHUB_EVENT_NAME} AND "$ENV{GITHUB_EVENT_NAME}" STREQUAL "pull_request")
        if(DEFINED ENV{GITHUB_REF} AND "$ENV{GITHUB_REF}" MATCHES "^refs/pull/([0-9]+)/")
            set(pr_number "${CMAKE_MATCH_1}")
        elseif(DEFINED ENV{GITHUB_HEAD_REF} AND DEFINED ENV{GITHUB_REF_NAME} AND
               "$ENV{GITHUB_REF_NAME}" MATCHES "^([0-9]+)/")
            set(pr_number "${CMAKE_MATCH_1}")
        endif()
    endif()

    if(pr_number STREQUAL "" AND DEFINED ENV{SYSTEM_PULLREQUEST_PULLREQUESTNUMBER})
        set(pr_number "$ENV{SYSTEM_PULLREQUEST_PULLREQUESTNUMBER}")
    endif()

    if(pr_number STREQUAL "" AND DEFINED ENV{CHANGE_ID})
        set(pr_number "$ENV{CHANGE_ID}")
    endif()

    if(pr_number STREQUAL "")
        set(${out_var} "" PARENT_SCOPE)
    else()
        set(${out_var} "-pr${pr_number}" PARENT_SCOPE)
    endif()
endfunction()

find_package(Git QUIET)
openearable_resolve_pr_suffix(PR_SUFFIX)
if(GIT_FOUND)
    openearable_resolve_git_dir(PROJECT_GIT_DIR)
    if(PROJECT_GIT_DIR)
        openearable_track_git_state("${PROJECT_GIT_DIR}")

        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${VERSION_CMAKE_DIR}" describe --tags --long --dirty --match "v[0-9]*"
            OUTPUT_VARIABLE git_describe
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE git_describe_result
            ERROR_QUIET
        )

        if(git_describe_result EQUAL 0 AND
           git_describe MATCHES "^v([0-9]+\\.[0-9]+\\.[0-9]+(\\.[0-9]+)?)(-[0-9A-Za-z.-]+)?-([0-9]+)-(g[0-9a-f]+)(-dirty)?$")
            set(GIT_TAG_VERSION "${CMAKE_MATCH_1}${CMAKE_MATCH_3}")
            set(GIT_COMMITS_SINCE_TAG "${CMAKE_MATCH_4}")
            set(GIT_SHORT_SHA "${CMAKE_MATCH_5}")
            set(GIT_DIRTY_SUFFIX "${CMAKE_MATCH_6}")

            if(GIT_COMMITS_SINCE_TAG EQUAL 0 AND GIT_DIRTY_SUFFIX STREQUAL "")
                set(FIRMWARE_VERSION "${GIT_TAG_VERSION}${PR_SUFFIX}")
            else()
                set(FIRMWARE_VERSION "${GIT_TAG_VERSION}-dev.${GIT_COMMITS_SINCE_TAG}${PR_SUFFIX}+${GIT_SHORT_SHA}")
                if(NOT GIT_DIRTY_SUFFIX STREQUAL "")
                    string(APPEND FIRMWARE_VERSION ".dirty")
                endif()
            endif()
        endif()
    endif()
endif()

if(PR_SUFFIX AND FIRMWARE_VERSION STREQUAL FIRMWARE_BASE_VERSION)
    set(FIRMWARE_VERSION "${FIRMWARE_VERSION}${PR_SUFFIX}")
endif()

message(STATUS "Firmware version: ${FIRMWARE_VERSION}")

configure_file(
    "${VERSION_CMAKE_DIR}/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/include/generated/version.h"
    @ONLY
)
