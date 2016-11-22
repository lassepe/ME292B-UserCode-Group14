include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)

px4_add_module(
	MODULE lib__Components
	COMPILE_FLAGS
	SRCS
		Components/Logic/QuadcopterLogic.cpp
	DEPENDS
		lib__Common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
