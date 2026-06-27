# Copyright 2020 Robotic Systems Lab - ETH Zurich:
# Lennart Nachtigall, Jonas Junger
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.22)
project(ethercat_sdk_master)


set(CMAKE_CXX_STANDARD 17)
add_definitions(-DMELO_USE_COUT)


find_package(ament_cmake REQUIRED)
find_package(soem_interface_rsl REQUIRED)
find_package(message_logger REQUIRED)
find_package(Threads REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/EthercatMaster.cpp
  src/${PROJECT_NAME}/EthercatDevice.cpp
  src/${PROJECT_NAME}/EthercatMasterSingleton.cpp
)
add_library(${PROJECT_NAME}:${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic  -Wpointer-arith -Wcast-qual -Wcast-align -Werror=return-type)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC -fPIC)


# Support ros distros older than lyrical
if(COMMAND ament_target_dependencies)
    ament_target_dependencies(${PROJECT_NAME} message_logger soem_interface_rsl)
else()
    target_link_libraries(${PROJECT_NAME}
    message_logger::message_logger
    soem_interface_rsl::soem_interface_rsl)
endif()


ament_export_dependencies(message_logger soem_interface_rsl)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
