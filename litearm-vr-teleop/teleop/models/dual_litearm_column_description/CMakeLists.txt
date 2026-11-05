cmake_minimum_required(VERSION 3.5)
project(dual_litearm_column_description)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes rviz urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()
