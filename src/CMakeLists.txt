#
# NB! if changing this file, use "idf.py clean"! (1 hour lost)
#
# Also, for -fcolor-diagnostics (just a tiny bit of colors for errors in gcc), you have to
# 'set GCC_COLORS="error=01;31:warning=01;33:note=01;36:caret=01;32:locus=01:quote=01"' beforehand
# in windows.
#

# The following lines of boilerplate have to be in your project's CMakeLists in this exact order (more or less)
# for cmake to work correctly.
cmake_minimum_required(VERSION 3.5)

set(SUPPORTED_TARGETS esp32)


# JetBrains clion needs a little help for code inspection/completion to work. We are using this helper:
#   https://gitlab.feather.ink/raldone01/esp32-clion-project-template/tree/master

if (WIN32 AND "$ENV{JETBRAINS_IDE}")
#if (WIN32)

	# Change this to what you chose during install
	set(ENV{IDF_PATH} "C:\\programming\\esp32\\esp-v4.1\\esp-idf")    # The ESP-IDF location
	set(PYTHON_PATH "C:\\Python\\Python38\\")                         # The systems python installation
	set(GIT_PATH "C:\\Program Files\\Git\\cmd")                       # The systems git location
	set(DOT_EXPRESSIF "C:\\programming\\esp32\\esp-v4.1\\.espressif") # The expressif tools folder

	include(espmake/esp_include.cmake)

endif()


include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#add_compile_options(-fcolor-diagnostics -Werror)
add_compile_options(-fdiagnostics-color=always -Wno-unused-variable -Wno-unused-function)

project(tiltwheeler)



