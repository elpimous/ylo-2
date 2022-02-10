cmake_minimum_required(VERSION 3.0.0)
project(wolf_description_utils)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdfs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
  PATTERN ".svn" EXCLUDE)

catkin_install_python(PROGRAMS scripts/go0
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
