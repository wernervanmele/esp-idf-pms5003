# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/wvanmele/esp/esp-idf/components/bootloader/subproject"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/tmp"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/src"
  "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/wvanmele/ESP_Workspaces/examples/pms5003/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
