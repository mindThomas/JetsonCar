# check the existance of MRPT, if not/outdated, install/reinstall
find_package(MRPT) # REQUIRED
if (${MRPT_FOUND})
	message(STATUS "MRPT found")
else()
	message(STATUS "MRPT not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "mrpt.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(MRPT REQUIRED) # try and find again
endif()
include_directories(${MRPT_INCLUDE_DIR})
