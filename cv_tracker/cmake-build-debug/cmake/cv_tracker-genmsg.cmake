# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cv_tracker: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icv_tracker:/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cv_tracker_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_custom_target(_cv_tracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cv_tracker" "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cv_tracker
  "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cv_tracker
)

### Generating Services

### Generating Module File
_generate_module_cpp(cv_tracker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cv_tracker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cv_tracker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cv_tracker_generate_messages cv_tracker_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_dependencies(cv_tracker_generate_messages_cpp _cv_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cv_tracker_gencpp)
add_dependencies(cv_tracker_gencpp cv_tracker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cv_tracker_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cv_tracker
  "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cv_tracker
)

### Generating Services

### Generating Module File
_generate_module_eus(cv_tracker
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cv_tracker
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cv_tracker_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cv_tracker_generate_messages cv_tracker_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_dependencies(cv_tracker_generate_messages_eus _cv_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cv_tracker_geneus)
add_dependencies(cv_tracker_geneus cv_tracker_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cv_tracker_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cv_tracker
  "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cv_tracker
)

### Generating Services

### Generating Module File
_generate_module_lisp(cv_tracker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cv_tracker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cv_tracker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cv_tracker_generate_messages cv_tracker_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_dependencies(cv_tracker_generate_messages_lisp _cv_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cv_tracker_genlisp)
add_dependencies(cv_tracker_genlisp cv_tracker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cv_tracker_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cv_tracker
  "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cv_tracker
)

### Generating Services

### Generating Module File
_generate_module_nodejs(cv_tracker
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cv_tracker
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cv_tracker_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cv_tracker_generate_messages cv_tracker_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_dependencies(cv_tracker_generate_messages_nodejs _cv_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cv_tracker_gennodejs)
add_dependencies(cv_tracker_gennodejs cv_tracker_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cv_tracker_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cv_tracker
  "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cv_tracker
)

### Generating Services

### Generating Module File
_generate_module_py(cv_tracker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cv_tracker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cv_tracker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cv_tracker_generate_messages cv_tracker_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/hcg-ubuntu/DATA/Linux/MSBRC_final_project/src/cv_tracker/msg/destination_msg.msg" NAME_WE)
add_dependencies(cv_tracker_generate_messages_py _cv_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cv_tracker_genpy)
add_dependencies(cv_tracker_genpy cv_tracker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cv_tracker_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cv_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cv_tracker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cv_tracker_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cv_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cv_tracker
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cv_tracker_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cv_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cv_tracker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cv_tracker_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cv_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cv_tracker
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cv_tracker_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cv_tracker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cv_tracker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cv_tracker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cv_tracker_generate_messages_py std_msgs_generate_messages_py)
endif()
