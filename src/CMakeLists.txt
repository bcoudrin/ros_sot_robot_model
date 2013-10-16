# Python
INCLUDE(../cmake/python.cmake)

PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/dynamics" "ros_humanoid_robot.py")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/dynamics" "abstract_robot.py")

dynamic_graph_python_module("/ros/ros_sot_robot_model" ros_sot_robot_model wrap)
