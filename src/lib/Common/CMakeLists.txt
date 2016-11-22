include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)

px4_add_module(
	MODULE lib__Common
	COMPILE_FLAGS
	SRCS
		Common/test.cpp
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
