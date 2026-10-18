# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/vboxuser/esp/esp-idf/components/bootloader/subproject"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/tmp"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/src/bootloader-stamp"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/src"
  "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/vboxuser/INSPACE/INSPACE-MECHAHOLICS/Rocketry/FSW/Rocketry-firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
