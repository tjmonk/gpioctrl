cmake_minimum_required(VERSION 3.10)

include(GNUInstallDirs)
find_package(PkgConfig REQUIRED)

project(gpioctrl
	VERSION 0.1
	DESCRIPTION "Server to interface system variables to GPIO ports"
)

pkg_check_modules(GPIOD REQUIRED libgpiod>=2.0)

find_library( LIB_VARSERVER varserver REQUIRED )
find_library( LIB_TJSON tjson REQUIRED )

add_executable( ${PROJECT_NAME}
	src/gpioctrl.c
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE inc
	${GPIOD_INCLUDE_DIRS}
)

target_link_libraries( ${PROJECT_NAME}
	${CMAKE_THREAD_LIBS_INIT}
	rt
    pthread
	${LIB_VARSERVER}
    ${LIB_TJSON}
	${GPIOD_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
