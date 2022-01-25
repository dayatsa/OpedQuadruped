cmake_minimum_required(VERSION 3.0.2)
project(oped_teleopp)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS 
  scripts/train/floor_controller.py 
  scripts/train/quadruped_controller.py 
  scripts/train/set_action.py 
  scripts/train/train.py  
  scripts/train/testingTrain.py 
  scripts/oped/engine.py 
  scripts/oped/testingOped.py 
  scripts/oped/Imu.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)