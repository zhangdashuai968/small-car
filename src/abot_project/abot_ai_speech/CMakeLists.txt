cmake_minimum_required(VERSION 2.8.3)
project(abot_ai_speech)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs message_generation
  )


catkin_package()

include_directories(
	# include
	${catkin_INCLUDE_DIRS}
)


#catkin_generate_virtualenv()

#catkin_install_python(
#  PROGRAMS
#  nodes/respeaker_node.py
#  nodes/speech_to_text.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY nodes launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  USE_SOURCE_PERMISSIONS)

#install(FILES requirements.txt
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

