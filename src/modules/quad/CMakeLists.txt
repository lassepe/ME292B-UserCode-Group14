include_directories(${CMAKE_SOURCE_DIR}/src/ME136/)


px4_add_module(
	MODULE modules__quad
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	MAIN quad
	SRCS
		main.cpp
	DEPENDS
	  ME136
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
