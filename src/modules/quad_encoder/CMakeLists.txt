px4_add_module(
	MODULE modules__quad_encoder
	MAIN quad_encoder
	SRCS
		quad_encoder_main.cpp
		quad_encoder_main.hpp
		QuadratureEncoder.cpp
		QuadratureEncoder.hpp
	DEPENDS
	)
