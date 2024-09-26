cmake_minimum_required(VERSION 3.16)
project(gazebo-omniverse1 NONE)

# FIXME: This always builds debug mode
add_custom_target(build ALL
	${CMAKE_CURRENT_SOURCE_DIR}/repo.sh build
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
