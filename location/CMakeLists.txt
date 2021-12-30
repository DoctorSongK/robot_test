cmake_minimum_required(VERSION 3.5)

include_directories(
	include
	${PROJECT_SOURCE_DIR}
)
link_directories(
	include
)
INCLUDE_DIRECTORIES("/usr/include/eigen3")
add_compile_options(-w)
file(GLOB Location_SRC "*.cpp")
add_compile_options(-g)
add_library (LOCATECommon ${Location_SRC})

target_link_libraries(LOCATECommon cartographer AGVCommon AMCLCommon)

