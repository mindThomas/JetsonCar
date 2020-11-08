# Check if ACADO is already compiled and installed
list(APPEND CMAKE_MODULE_PATH "$ENV{ACADO_ENV_CMAKE_DIR}") # Ensure to source the 'acado_env.sh' script from the ACADO build folder if it is already installed elsewhere
message(STATUS "ACADO search path: ${CMAKE_MODULE_PATH}")
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}

find_package(ACADO)
if (${ACADO_FOUND})
	message(STATUS "ACADO found")
else()
	#message(STATUS "ACADO not found! Ensure to source the 'acado_env.sh' script from the ACADO build folder. Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "acado.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	list(APPEND CMAKE_MODULE_PATH "$ENV{ACADO_ENV_CMAKE_DIR}")
	find_package(acado) # try and find again
	if (${ACADO_FOUND})
		message(STATUS "Successfully installed ACADO found")
	else()
		message(STATUS "Could not install nor find ACADO")
	endif()
	
endif()
include_directories(${ACADO_INCLUDE_DIRS})
