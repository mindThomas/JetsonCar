# Check if acados is already compiled and installed
set(ACADOS_SEARCH_PATH ~/repos/acados/cmake)
list(APPEND CMAKE_PREFIX_PATH "${ACADOS_SEARCH_PATH}")
message(STATUS "acados search path: ${ACADOS_SEARCH_PATH}")

find_package(acados)
if (${acados_FOUND})
	message(STATUS "acados found")
else()
	#message(STATUS "ACADO not found! Ensure to source the 'acado_env.sh' script from the ACADO build folder. Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "acados.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	
	find_package(acados) # try and find again
	if (${acados_FOUND})
		message(STATUS "Successfully installed acados found")
	else()
		message(FATAL_ERROR "Could not install nor find acados")        
	endif()
	
endif()
include_directories(${ACADOS_INCLUDE_DIRS})
