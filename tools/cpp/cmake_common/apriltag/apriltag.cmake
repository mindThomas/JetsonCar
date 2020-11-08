# check the existance of apriltags, if not/outdated, install/reinstall
find_package(apriltag)
if (${apriltag_FOUND})
	message(STATUS "Apriltag found")
else()
	message(STATUS "Apriltag not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "apriltag.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(apriltag REQUIRED) # try and find again
endif()
include_directories(${apriltag_INCLUDE_DIRS})
