cmake_minimum_required(VERSION 3.8)
project(mg400_bringup)

find_package(ament_cmake REQUIRED)

# launch / config 파일을 share 디렉터리에 설치
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
