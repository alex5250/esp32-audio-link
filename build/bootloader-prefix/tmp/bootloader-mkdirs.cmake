# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/alex/esp/v5.2.2/esp-idf/components/bootloader/subproject"
  "/home/alex/Documents/esp32-audio-link/build/bootloader"
  "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix"
  "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/tmp"
  "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/src/bootloader-stamp"
  "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/src"
  "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/alex/Documents/esp32-audio-link/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
