# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "RFID_Based_Robot_Navigation: 4 messages, 0 services")

set(MSG_I_FLAGS "-IRFID_Based_Robot_Navigation:/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_custom_target(_RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RFID_Based_Robot_Navigation" "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" ""
)

get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_custom_target(_RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RFID_Based_Robot_Navigation" "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" ""
)

get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_custom_target(_RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RFID_Based_Robot_Navigation" "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" ""
)

get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_custom_target(_RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RFID_Based_Robot_Navigation" "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" "RFID_Based_Robot_Navigation/tag_msgs:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_cpp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_cpp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_cpp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg"
  "${MSG_I_FLAGS}"
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)

### Generating Services

### Generating Module File
_generate_module_cpp(RFID_Based_Robot_Navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages RFID_Based_Robot_Navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_cpp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_cpp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_cpp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_cpp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RFID_Based_Robot_Navigation_gencpp)
add_dependencies(RFID_Based_Robot_Navigation_gencpp RFID_Based_Robot_Navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RFID_Based_Robot_Navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_eus(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_eus(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_eus(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg"
  "${MSG_I_FLAGS}"
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
)

### Generating Services

### Generating Module File
_generate_module_eus(RFID_Based_Robot_Navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages RFID_Based_Robot_Navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_eus _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_eus _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_eus _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_eus _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RFID_Based_Robot_Navigation_geneus)
add_dependencies(RFID_Based_Robot_Navigation_geneus RFID_Based_Robot_Navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RFID_Based_Robot_Navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_lisp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_lisp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_lisp(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg"
  "${MSG_I_FLAGS}"
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
)

### Generating Services

### Generating Module File
_generate_module_lisp(RFID_Based_Robot_Navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages RFID_Based_Robot_Navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_lisp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_lisp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_lisp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_lisp _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RFID_Based_Robot_Navigation_genlisp)
add_dependencies(RFID_Based_Robot_Navigation_genlisp RFID_Based_Robot_Navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RFID_Based_Robot_Navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_nodejs(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_nodejs(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_nodejs(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg"
  "${MSG_I_FLAGS}"
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(RFID_Based_Robot_Navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages RFID_Based_Robot_Navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_nodejs _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_nodejs _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_nodejs _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_nodejs _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RFID_Based_Robot_Navigation_gennodejs)
add_dependencies(RFID_Based_Robot_Navigation_gennodejs RFID_Based_Robot_Navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RFID_Based_Robot_Navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_py(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_py(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
)
_generate_msg_py(RFID_Based_Robot_Navigation
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg"
  "${MSG_I_FLAGS}"
  "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
)

### Generating Services

### Generating Module File
_generate_module_py(RFID_Based_Robot_Navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(RFID_Based_Robot_Navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages RFID_Based_Robot_Navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/sendEnding.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_py _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/tag_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_py _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/RFIDdata.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_py _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/msg/rfid_msgs.msg" NAME_WE)
add_dependencies(RFID_Based_Robot_Navigation_generate_messages_py _RFID_Based_Robot_Navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RFID_Based_Robot_Navigation_genpy)
add_dependencies(RFID_Based_Robot_Navigation_genpy RFID_Based_Robot_Navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RFID_Based_Robot_Navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RFID_Based_Robot_Navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(RFID_Based_Robot_Navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RFID_Based_Robot_Navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(RFID_Based_Robot_Navigation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RFID_Based_Robot_Navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(RFID_Based_Robot_Navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RFID_Based_Robot_Navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(RFID_Based_Robot_Navigation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RFID_Based_Robot_Navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(RFID_Based_Robot_Navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
