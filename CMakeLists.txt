cmake_minimum_required(VERSION 3.10)

include(GNUInstallDirs)

project(gpioctrl
	VERSION 0.1
    DESCRIPTION "Server to interface system variables to GPIO ports"
)

find_library( LIB_VARSERVER varserver REQUIRED )
find_library( LIB_TJSON tjson REQUIRED )
find_library( LIB_GPIOD gpiod REQUIRED )

add_executable( ${PROJECT_NAME}
	src/gpioctrl.c
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE inc
)

target_link_libraries( ${PROJECT_NAME}
	${CMAKE_THREAD_LIBS_INIT}
	rt
    pthread
	${LIB_VARSERVER}
    ${LIB_TJSON}
    ${LIB_GPIOD}
)

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
