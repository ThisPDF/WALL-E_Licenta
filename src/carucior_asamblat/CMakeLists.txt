cmake_minimum_required(VERSION 3.5)

project(carucior_asamblat)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes textures urdf worlds
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
