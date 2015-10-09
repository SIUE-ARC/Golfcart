# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sound_play: 8 messages, 0 services")

set(MSG_I_FLAGS "-Isound_play:/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg;-Isound_play:/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg;-Iactionlib_msgs:/opt/ros/hydro/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sound_play_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)
_generate_msg_cpp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestAction.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
)

### Generating Services

### Generating Module File
_generate_module_cpp(sound_play
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sound_play_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sound_play_generate_messages sound_play_generate_messages_cpp)

# target for backward compatibility
add_custom_target(sound_play_gencpp)
add_dependencies(sound_play_gencpp sound_play_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sound_play_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)
_generate_msg_lisp(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestAction.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
)

### Generating Services

### Generating Module File
_generate_module_lisp(sound_play
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sound_play_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sound_play_generate_messages sound_play_generate_messages_lisp)

# target for backward compatibility
add_custom_target(sound_play_genlisp)
add_dependencies(sound_play_genlisp sound_play_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sound_play_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)
_generate_msg_py(sound_play
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestAction.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/msg/SoundRequest.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionFeedback.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestActionResult.msg;/home/roadrunner/roadrunner_ws/devel/share/sound_play/msg/SoundRequestResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
)

### Generating Services

### Generating Module File
_generate_module_py(sound_play
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sound_play_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sound_play_generate_messages sound_play_generate_messages_py)

# target for backward compatibility
add_custom_target(sound_play_genpy)
add_dependencies(sound_play_genpy sound_play_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sound_play_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sound_play
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sound_play_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sound_play
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sound_play_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sound_play/.+/__init__.pyc?$"
  )
endif()
add_dependencies(sound_play_generate_messages_py actionlib_msgs_generate_messages_py)
