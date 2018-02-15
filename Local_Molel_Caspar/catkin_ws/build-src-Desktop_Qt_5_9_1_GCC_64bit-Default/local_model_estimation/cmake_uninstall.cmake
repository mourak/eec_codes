IF(NOT EXISTS "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/build-src-Desktop_Qt_5_9_1_GCC_64bit-Default/local_model_estimation/install_manifest.txt")
  MESSAGE(FATAL_ERROR "Cannot find install manifest: \"/home/eec/Documents/Mouloud/Code_V2/catkin_ws/build-src-Desktop_Qt_5_9_1_GCC_64bit-Default/local_model_estimation/install_manifest.txt\"")
ENDIF(NOT EXISTS "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/build-src-Desktop_Qt_5_9_1_GCC_64bit-Default/local_model_estimation/install_manifest.txt")

FILE(READ "/home/eec/Documents/Mouloud/Code_V2/catkin_ws/build-src-Desktop_Qt_5_9_1_GCC_64bit-Default/local_model_estimation/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
FOREACH(file ${files})
  MESSAGE(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  IF(EXISTS "$ENV{DESTDIR}${file}")
    EXEC_PROGRAM(
      "/usr/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
      OUTPUT_VARIABLE rm_out
      RETURN_VALUE rm_retval
      )
    IF("${rm_retval}" STREQUAL 0)
    ELSE("${rm_retval}" STREQUAL 0)
      MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    ENDIF("${rm_retval}" STREQUAL 0)
  ELSE(EXISTS "$ENV{DESTDIR}${file}")
    MESSAGE(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  ENDIF(EXISTS "$ENV{DESTDIR}${file}")
ENDFOREACH(file)
