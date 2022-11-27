# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "Uwb_Location: 1 messages, 0 services")

set(MSG_I_FLAGS "-IUwb_Location:/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(Uwb_Location_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_custom_target(_Uwb_Location_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Uwb_Location" "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(Uwb_Location
  "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Uwb_Location
)

### Generating Services

### Generating Module File
_generate_module_cpp(Uwb_Location
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Uwb_Location
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(Uwb_Location_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(Uwb_Location_generate_messages Uwb_Location_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_dependencies(Uwb_Location_generate_messages_cpp _Uwb_Location_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Uwb_Location_gencpp)
add_dependencies(Uwb_Location_gencpp Uwb_Location_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Uwb_Location_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(Uwb_Location
  "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Uwb_Location
)

### Generating Services

### Generating Module File
_generate_module_eus(Uwb_Location
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Uwb_Location
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(Uwb_Location_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(Uwb_Location_generate_messages Uwb_Location_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_dependencies(Uwb_Location_generate_messages_eus _Uwb_Location_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Uwb_Location_geneus)
add_dependencies(Uwb_Location_geneus Uwb_Location_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Uwb_Location_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(Uwb_Location
  "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Uwb_Location
)

### Generating Services

### Generating Module File
_generate_module_lisp(Uwb_Location
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Uwb_Location
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(Uwb_Location_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(Uwb_Location_generate_messages Uwb_Location_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_dependencies(Uwb_Location_generate_messages_lisp _Uwb_Location_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Uwb_Location_genlisp)
add_dependencies(Uwb_Location_genlisp Uwb_Location_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Uwb_Location_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(Uwb_Location
  "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Uwb_Location
)

### Generating Services

### Generating Module File
_generate_module_nodejs(Uwb_Location
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Uwb_Location
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(Uwb_Location_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(Uwb_Location_generate_messages Uwb_Location_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_dependencies(Uwb_Location_generate_messages_nodejs _Uwb_Location_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Uwb_Location_gennodejs)
add_dependencies(Uwb_Location_gennodejs Uwb_Location_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Uwb_Location_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(Uwb_Location
  "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Uwb_Location
)

### Generating Services

### Generating Module File
_generate_module_py(Uwb_Location
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Uwb_Location
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(Uwb_Location_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(Uwb_Location_generate_messages Uwb_Location_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xtark/UWB/catkin_ws/src/Uwb_Location/msg/uwb.msg" NAME_WE)
add_dependencies(Uwb_Location_generate_messages_py _Uwb_Location_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Uwb_Location_genpy)
add_dependencies(Uwb_Location_genpy Uwb_Location_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Uwb_Location_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Uwb_Location)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Uwb_Location
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(Uwb_Location_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Uwb_Location)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Uwb_Location
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(Uwb_Location_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Uwb_Location)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Uwb_Location
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(Uwb_Location_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Uwb_Location)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Uwb_Location
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(Uwb_Location_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Uwb_Location)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Uwb_Location\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Uwb_Location
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(Uwb_Location_generate_messages_py std_msgs_generate_messages_py)
endif()
