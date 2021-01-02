if($ENV{JETBRAINS_IDE})

    set(IDF_PATH $ENV{IDF_PATH})
    set(ENV{PYTHONPATH} "")

    MESSAGE("You may need to run 'pip install -r \"$ENV{IDF_PATH}\\requirements.txt\"' after the first run.")

    file(APPEND "${IDF_PATH}/../CMakeLists.txt" "")
    file(APPEND "${DOT_EXPRESSIF}/../CMakeLists.txt" "")

    add_subdirectory("${DOT_EXPRESSIF}/../" "${CMAKE_CURRENT_BINARY_DIR}/dot-expressifDummy")
    add_subdirectory("${IDF_PATH}/../" "${CMAKE_CURRENT_BINARY_DIR}/esp-idfDummy")

    #set(CMAKE_VERBOSE_MAKEFILE ON)

    if (NOT DEFINED globbedtools)
        file(GLOB_RECURSE globbedtools
                LIST_DIRECTORIES true
                ${DOT_EXPRESSIF}/tools)
        set(globbedtools ${globbedtools} CACHE INTERNAL "Speedup")
    endif ()


    #mess with the path
    #would be nice if idf_tools.py supported exporting all the necessary paths to a \n seperated list so that I don't have to use globbing
    set(CMAKE_PROGRAM_PATH
            ${GIT_PATH}
            ${PYTHON_PATH}
            ${IDF_PATH}/tools
            ${globbedtools}
            )

    include_directories($ENV{IDF_PATH}/components/soc/esp32/include/soc)

    if (WIN32)
        add_custom_target(aopenbuildshell COMMAND start cmd /k "${DOT_EXPRESSIF}/idf_cmd_init.bat ${PYTHON_PATH} ${GIT_PATH} && cd ${CMAKE_SOURCE_DIR}"
                WORKING_DIRECTORY ${IDF_PATH})
        add_custom_target(amaenuconfig COMMAND start cmd /c "${DOT_EXPRESSIF}/idf_cmd_init.bat ${PYTHON_PATH} ${GIT_PATH} && cd ${CMAKE_SOURCE_DIR} && idf.py menuconfig"
                WORKING_DIRECTORY ${IDF_PATH})
        add_custom_target(amonitor COMMAND start cmd /c "${DOT_EXPRESSIF}/idf_cmd_init.bat ${PYTHON_PATH} ${GIT_PATH} && cd ${CMAKE_SOURCE_DIR} && idf.py monitor & pause"
                WORKING_DIRECTORY ${IDF_PATH})
        add_custom_target(ainit COMMAND start cmd /c "${DOT_EXPRESSIF}/idf_cmd_init.bat ${PYTHON_PATH} ${GIT_PATH} && cd ${CMAKE_SOURCE_DIR} && idf.py build || pause"
                WORKING_DIRECTORY ${IDF_PATH})
    endif()

endif()
