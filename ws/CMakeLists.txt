file(GLOB HW_SUBDIRECTORIES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/shared")
    message("Found user-created libraries")
    file(GLOB USER_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/shared/*.cpp")
    add_library(user_library STATIC ${USER_LIB_SOURCES})
    target_link_libraries(user_library PRIVATE
        yaml-cpp
        Eigen3::Eigen
        AMPCorePrivate
        AMPCorePublic
    )
    target_include_directories(user_library PUBLIC
        ${AMP_INCLUDE_DIRS}
        ${AMP_DEPENDENCY_INCLUDE_DIRS}
    )
endif()

foreach(SUBDIR ${HW_SUBDIRECTORIES})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}
        AND NOT SUBDIR STREQUAL "shared"
        AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/IGNORE"
        AND NOT SUBDIR STREQUAL "hw7/benchmark")
        file(GLOB_RECURSE USER_SOURCES "${SUBDIR}/*.cpp")
        # Remove benchmark sources from USER_SOURCES if SUBDIR is hw7
        if(SUBDIR STREQUAL "hw7")
            list(FILTER USER_SOURCES EXCLUDE REGEX ".*hw7/benchmark/.*\\.cpp$")
            list(FILTER USER_SOURCES EXCLUDE REGEX ".*hw7/hw2_.*\\.cpp$")
        endif()
        # Remove hw6_ex2 from main-hw6
        if(SUBDIR STREQUAL "hw6")
            list(FILTER USER_SOURCES EXCLUDE REGEX ".*hw6/hw6_ex2\\.cpp$")
        endif()
        set(TARGET_NAME "main-${SUBDIR}")
        add_executable(${TARGET_NAME} ${USER_SOURCES})
        target_link_libraries(${TARGET_NAME} PRIVATE
            yaml-cpp
            Eigen3::Eigen
            AMPCorePrivate
            AMPCorePublic
        )
        target_include_directories(${TARGET_NAME} PUBLIC
            ${AMP_INCLUDE_DIRS}
            ${AMP_DEPENDENCY_INCLUDE_DIRS}
        )
        if(TARGET user_library)
            target_link_libraries(${TARGET_NAME} PRIVATE user_library)
            target_include_directories(${TARGET_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/shared")
        endif()

        if(AMP_EXCLUDE_VIS)
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_VIS)
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                target_compile_options(${TARGET_NAME} PRIVATE -g)
            endif()
        endif()

        if (AMP_EXCLUDE_LOGS)
            message("Building ${TARGET_NAME} without logs")
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_LOGS)
        endif()

    endif()
endforeach()

# Add HW2 Exercise 2 executables
add_executable(hw2-exercise2 
    hw7/hw2_exercise2.cpp
    hw7/MySamplingBasedPlanners.cpp
    hw7/PathSmoothing.cpp
)
target_link_libraries(hw2-exercise2 PRIVATE
    yaml-cpp
    Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
    user_library
)
target_include_directories(hw2-exercise2 PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/shared"
    "${CMAKE_CURRENT_SOURCE_DIR}/hw7"
)

add_executable(hw2-benchmark
    hw7/hw2_benchmark.cpp
    hw7/MySamplingBasedPlanners.cpp
    hw7/PathSmoothing.cpp
)
target_link_libraries(hw2-benchmark PRIVATE
    yaml-cpp
    Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
    user_library
)
target_include_directories(hw2-benchmark PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/shared"
    "${CMAKE_CURRENT_SOURCE_DIR}/hw7"
)

add_executable(hw2-smoothed-benchmark
    hw7/hw2_smoothed_benchmark.cpp
    hw7/MySamplingBasedPlanners.cpp
    hw7/PathSmoothing.cpp
)
target_link_libraries(hw2-smoothed-benchmark PRIVATE
    yaml-cpp
    Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
    user_library
)
target_include_directories(hw2-smoothed-benchmark PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/shared"
    "${CMAKE_CURRENT_SOURCE_DIR}/hw7"
)

add_executable(hw6-ex2
    hw6/hw6_ex2.cpp
    hw6/MyCSConstructors.cpp
    hw6/MyAStar.cpp
)
target_link_libraries(hw6-ex2 PRIVATE
    yaml-cpp
    Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
    user_library
)
target_include_directories(hw6-ex2 PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/shared"
    "${CMAKE_CURRENT_SOURCE_DIR}/hw6"
)

add_subdirectory(hw7/benchmark)

