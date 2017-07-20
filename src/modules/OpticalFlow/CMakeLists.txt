px4_add_module(
	MODULE modules__OpticalFlow
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	MAIN flow
	STACK_MAIN 2000
	SRCS
		flowMain.cpp
		flowdeck.cpp
		vl53l0x.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
