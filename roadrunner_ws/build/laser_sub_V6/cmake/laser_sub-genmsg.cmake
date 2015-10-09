# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "laser_sub: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilaser_sub:/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(laser_sub_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/lasArray.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sub
)
_generate_msg_cpp(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sub
)

### Generating Services

### Generating Module File
_generate_module_cpp(laser_sub
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sub
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(laser_sub_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(laser_sub_generate_messages laser_sub_generate_messages_cpp)

# target for backward compatibility
add_custom_target(laser_sub_gencpp)
add_dependencies(laser_sub_gencpp laser_sub_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sub_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/lasArray.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sub
)
_generate_msg_lisp(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sub
)

### Generating Services

### Generating Module File
_generate_module_lisp(laser_sub
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sub
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(laser_sub_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(laser_sub_generate_messages laser_sub_generate_messages_lisp)

# target for backward compatibility
add_custom_target(laser_sub_genlisp)
add_dependencies(laser_sub_genlisp laser_sub_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sub_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/lasArray.msg"
  "${MSG_I_FLAGS}"
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub
)
_generate_msg_py(laser_sub
  "/home/roadrunner/roadrunner_ws/src/laser_sub_V6/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub
)

### Generating Services

### Generating Module File
_generate_module_py(laser_sub
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(laser_sub_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(laser_sub_generate_messages laser_sub_generate_messages_py)

# target for backward compatibility
add_custom_target(laser_sub_genpy)
add_dependencies(laser_sub_genpy laser_sub_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sub_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sub
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(laser_sub_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sub
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(laser_sub_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sub
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(laser_sub_generate_messages_py std_msgs_generate_messages_py)
