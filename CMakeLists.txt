cmake_minimum_required(VERSION 3.7)
project(Filters)

include(filters.cmake)

add_definitions(-fpic)

SET(LIBAMR_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/dec_amr.c
)

SET(LIBAMR_LIBS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencore-amrnb.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencore-amrwb.a
)

SET(LIBAMR_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_filter(libamr
        "${LIBAMR_SRC}"
        "${LIBAMR_LIBS}"
        []
        ""
        "${LIBAMR_INC}"
        ""
        "1")
