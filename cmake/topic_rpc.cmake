
function(add_topic_rpc_messages)
  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "add_topic_rpc_messages() should be called before generate_messages")
  endif()

  set(GEN_OUTPUT_DIR ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/${PROJECT_NAME})

  foreach(srv ${ARGV})
    # TODO:
    #  * destination filename
    #  * destination file content (from ...src import ...)
    set(GEN_OUTPUT_FILE "${GEN_OUTPUT_DIR}/msg/_${srv}.py")
    message("Generating topic_rpc shim for ${srv} in ${GEN_OUTPUT_FILE}")

    add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
      COMMAND echo "from ${PROJECT_NAME}.src import ${srv}Request,${srv}Response" > ${GEN_OUTPUT_FILE}
      COMMENT "Generating topic_rpc message for ${srv}")

    list(APPEND ALL_GEN_OUTPUT_FILES_py ${GEN_OUTPUT_FILE})
    set(ALL_GEN_OUTPUT_FILES_py ${ALL_GEN_OUTPUT_FILES_py} PARENT_SCOPE)
  endforeach()
endfunction()
