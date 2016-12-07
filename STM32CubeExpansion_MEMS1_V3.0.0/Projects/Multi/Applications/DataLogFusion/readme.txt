/**
  @page Sensor Demo Fusion Application based on Sensor expansion board and STM32 Nucleo Boards
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    readme.txt  
  * @version V1.6.0
  * @date    8-Nov-2016
  * @brief   This application contains an example which shows how to use X_NUCLEO_IKS01A1 or X_NUCLEO_IKS01A2
  *          expansion board to send sensor fusion data from a Nucleo board using UART to a connected PC
  *          or Desktop and display it on Unicleo, which is developed by STMicroelectronics and provided
  *          in binary with a separated package, or on Sensors_DataLog specific application, which is developed
  *          by STMicroelectronics and provided in binary with X-CUBE-MEMS1 package.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_IKS01A2 expansion board to send sensor fusion data from a Nucleo board 
using UART to a connected PC or Desktop and display it on Unicleo, which is developed by STMicroelectronics and provided in binary with a separated package,
or on Sensors_DataLog specific application, which is developed by STMicroelectronics and provided in binary with X-CUBE-MEMS1 package.
After connection has been established with Unicleo or Sensors_DataLog application:
- the user can view the data from various on-board environment sensors like Temperature, Humidity and Pressure
- the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope and Magnetometer
- the user can also visualize these data as graphs
- the user can also visualize a 3D animation that shows sensor fusion outputs
- the user can also visualize MEMS data (Accelerometer, Gyroscope and Magnetometer) and sensor fusion data (quaternions and Euler angles) when sensor fusion is activated
- the user can calibrate the Magnetometer when sensor fusion is activated pressing on the user button; the calibration data are stored in memory; this procedure
  is needed only when fusion 9X is used
- the user can dynamically switch from fusion 9X to fusion 6X and vice-versa
- the user can visualize the Magnetometer scatter plots in order to check the goodness of the calibration


@par Hardware and Software environment

  - This example runs on sensor expansion board (X-NUCLEO-IKS01A1 or X-NUCLEO-IKS01A2) attached to STM32F401RE or STM32F411RE or STM32L476RG devices.
  - If you power the Nucleo board via USB 3.0 port, please check that you have flashed the last version of
    the firmware of ST-Link v2 inside the Nucleo board. In order to flash the last available firmware of the 
	ST-Link v2, you can use the STM32 ST Link Utility.
  - Only for X-NUCLEO-IKS01A1, if you have the DIL24 expansion component with the LSM6DS3 sensor and you want to enable the LSM6DS3 sensor,
    plug the component into the DIL24 interface; otherwise the LSM6DS0 sensor is enabled by default.
  - Only for X-NUCLEO-IKS01A1, if you have the DIL24 expansion component with the LPS25HB sensor and you want to enable the LPS25HB sensor,
    plug the component into the DIL24 interface; otherwise the LPS25H/B onboard sensor is enabled by default.
  - Only for X-NUCLEO-IKS01A1, if you have the DIL24 expansion component with the LPS22HB sensor and you want to enable the LPS22HB sensor,
    plug the component into the DIL24 interface; otherwise the LPS25H/B onboard sensor is enabled by default.	
  - In order to calibrate the Magnetometer the user has to press the user button; after this operation the LED2 will be switched off;
    in order to calibrate the device the user has to perform the 8 movement calibration; when the calibration will be finished the LED2
    will be switched on; the calibration can be done only after the Sensor Fusion has been activated, i.e. after the "Start Sensor Fusion"
    button on Sensors_DataLog application has been pressed. The calibration data are stored in memory; by default these data are stored
	in RAM memory; instead if you define in your project "OSXMOTIONFX_STORE_CALIB_FLASH" the calibration data are stored
	in the last block of the flash memory. The calibration procedure is needed only when fusion 9X is used.
  - This example has been tested with STMicroelectronics NUCLEO-F401RE RevC, NUCLEO-F411RE RevC and NUCLEO-L476RG RevC and 
    can be easily tailored to any other supported device and development board.
    

@par How to use it ? 

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. In order to make the 
program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - WARNING: Sensor Fusion library requires a valid license in order to be used;
   without this license, you cannot compile the sample application; please read
   OSXMotionFX Release Notes in order to know how to get a valid license.
 - WARNING: before opening the project with System Workbench be sure your folder
   installation path does not include spaces, otherwise the project does not compile correctly.
 - WARNING: this sample application is only compatible with MEMS1 version 3.0.0;
   it does not work with previous versions of MEMS1 package.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.70.1).
 - Open the IAR project file EWARM\STM32F401RE-Nucleo\Project.eww or EWARM\STM32L476RG-Nucleo\Project.eww according the target board used.
 - For Nucleo-F401RE or Nucleo-F411RE choose STM32F4xx-Nucleo-IKS01A1 target if you want to use X-NUCLEO-IKS01A1 or
   STM32F4xx-Nucleo-IKS01A2 target if you want to use X-NUCLEO-IKS01A2; for Nucleo-L476RG choose STM32L4xx-Nucleo-IKS01A1 target
   if you want to use X-NUCLEO-IKS01A1 or STM32L4xx-Nucleo-IKS01A2 target if you want to use X-NUCLEO-IKS01A2.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For µVision:
 - Open µVision 5 toolchain (this firmware has been 
   successfully tested with MDK-ARM Professional Version: 5.18a).
 - Open the µVision project file MDK-ARM\STM32F401RE-Nucleo\Project.uvprojx or MDK-ARM\STM32L476RG-Nucleo\Project.uvprojx according the target board used.
 - For Nucleo-F401RE or Nucleo-F411RE choose STM32F4xx-Nucleo-IKS01A1 target if you want to use X-NUCLEO-IKS01A1 or
   STM32F4xx-Nucleo-IKS01A2 target if you want to use X-NUCLEO-IKS01A2; for Nucleo-L476RG choose STM32L476RG_NUCLEO_IKS01A1 target
   if you want to use X-NUCLEO-IKS01A1 or STM32L476RG_NUCLEO_IKS01A2 target if you want to use X-NUCLEO-IKS01A2.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For System Workbench:
 - Open System Workbench for STM32 (this firmware has been 
   successfully tested with System Workbench for STM32 Version 1.10.0.20160725).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be SW4STM32\STM32F401RE-Nucleo\STM32F4xx-Nucleo-DataLogFusion
   or SW4STM32\STM32L476RG-Nucleo\STM32L4xx-Nucleo-DataLogFusion according the target board used).
 - For Nucleo-F401RE or Nucleo-F411RE choose Debug_IKS01A1 target if you want to use X-NUCLEO-IKS01A1 or
   Debug_IKS01A2 target if you want to use X-NUCLEO-IKS01A2; for Nucleo-L476RG choose Debug_IKS01A1 target
   if you want to use X-NUCLEO-IKS01A1 or Debug_IKS01A2 target if you want to use X-NUCLEO-IKS01A2.
 - Rebuild all files and load your image into target memory.
 - Run the example.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
