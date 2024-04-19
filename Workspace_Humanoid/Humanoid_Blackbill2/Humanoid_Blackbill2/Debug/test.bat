@echo off
 
REM STM32CubeProgrammer software must be installed before using this script
 
REM Path to STM32CubeProgrammer CLI executable
set PROGRAMMER="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set APP_BINARY=Humanoid_Blackbill2.bin 

 
REM Path to ST-Link interface
set STLINK_INTERFACE="SWD"

REM Connect to ST-Link using SWO protocol
%PROGRAMMER% -c port=%STLINK_INTERFACE% freq=3900 ap=0

REM Erase flash
%PROGRAMMER% -c port=%STLINK_INTERFACE% -e all

REM Download APP binary at 0x08000000 
%PROGRAMMER% -c port=%STLINK_INTERFACE% mode=UR -d %APP_BINARY% 0x08000000 -v

REM RESET MCU after flashing 
%PROGRAMMER% -c port=%STLINK_INTERFACE% -rst

set /p DUMMY=Hit ENTER to continue...