# Copyright 2018 gRPC authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# cmake build file for C++ ecloud example.
# Assumes protobuf and gRPC have been installed using cmake.
# See cmake_externalproject/CMakeLists.txt for all-in-one cmake build
# that automatically builds all the dependencies before building ecloud.

cmake_minimum_required(VERSION 3.8)

project(EcloudServer C CXX)
include(../cmake/common.cmake)

# Proto file
get_filename_component(ecloud_proto "../protos/ecloud.proto" ABSOLUTE)
get_filename_component(ecloud_proto_path "${ecloud_proto}" PATH)

# Generated sources
set(ecloud_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/ecloud.pb.cc")
set(ecloud_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/ecloud.pb.h")
set(ecloud_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/ecloud.grpc.pb.cc")
set(ecloud_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/ecloud.grpc.pb.h")
add_custom_command(
      OUTPUT "${ecloud_proto_srcs}" "${ecloud_proto_hdrs}" "${ecloud_grpc_srcs}" "${ecloud_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${ecloud_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${ecloud_proto}"
      DEPENDS "${ecloud_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# ecloud_grpc_proto
add_library(ecloud_grpc_proto
  ${ecloud_grpc_srcs}
  ${ecloud_grpc_hdrs}
  ${ecloud_proto_srcs}
  ${ecloud_proto_hdrs})
target_link_libraries(ecloud_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

# Targets greeter_[async_](client|server)
foreach(_target
  ecloud_server)
  add_executable(${_target} "${_target}.cc")
  target_link_libraries(${_target}
    ecloud_grpc_proto
    absl::flags
    absl::flags_parse
    absl::log
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
    absl::flags
    absl::flags_parse
    absl::flags_usage
  )
endforeach()
