cmake_minimum_required(VERSION 3.20)
project(msfs_ap_bridge LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_definitions(-DUNICODE=0 -D_UNICODE=0)

if(NOT DEFINED MSFS_SDK_DIR)
    find_path(MSFS_SDK_DIR
        NAMES "SimConnect SDK/include/SimConnect.h"
        PATHS
            "C:/MSFS SDK"
            "$ENV{ProgramFiles}/MSFS SDK"
            "$ENV{ProgramFiles\(x86\)}/MSFS SDK"
            "$ENV{MSFS_SDK_DIR}"
        DOC "Path to the Microsoft Flight Simulator SDK root"
    )
endif()

if(EXISTS "${MSFS_SDK_DIR}/SimConnect SDK/include/SimConnect.h")
    include_directories("${MSFS_SDK_DIR}/SimConnect SDK/include")
    link_directories("${MSFS_SDK_DIR}/SimConnect SDK/lib")
else()
    message(FATAL_ERROR "SimConnect.h non trovato! Imposta MSFS_SDK_DIR (es. -DMSFS_SDK_DIR=\"C:/MSFS SDK\")")
endif()

set(SOURCES
    src/msfs_ap_bridge.cpp
)
set(RESOURCES
    src/app.rc
)

add_executable(msfs_ap_bridge WIN32 ${SOURCES} ${RESOURCES})
target_link_libraries(msfs_ap_bridge PRIVATE SimConnect)

set(APP_ICON "${CMAKE_SOURCE_DIR}/res/msfs_ap_bridge.ico")
set_source_files_properties(src/app.rc PROPERTIES LANGUAGE RC)

if(MSVC)
    target_compile_options(msfs_ap_bridge PRIVATE /W4 /EHsc)
    target_link_options(msfs_ap_bridge PRIVATE "/DELAYLOAD:SimConnect.dll")
    add_compile_options(/EHa)
endif()
