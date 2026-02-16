# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## GPIO Blinking LED Demo for FRDM-Auto-S32K312
This demo presents the S32 Design Studio toolchain, flashing flow and the basic GPIO/clock initialization on the FRDM S32K312 by toggling the onboard LED. 

#### Boards: FRDM S32K312
#### Categories: GPIO, Clock
#### Peripherals: SIUL2 (Port/DIO), Clock
#### Toolchains: S32 Design Studio IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [Support](#step6)
6. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
- [S32 Design Studio IDE v3.6.5 or later](https://www.nxp.com/design/design-center/software/automotive-software-and-tools/s32-design-studio-ide/s32-design-studio-for-s32-platform:S32DS-S32PLATFORM)
- [S32K3_S32M27x Real-Time Drivers ASR R23-11 Version 7.0.0](https://www.nxp.com/design/design-center/software/automotive-software-and-tools/real-time-drivers-rtd:AUTOMOTIVE-RTD)

## 2. Hardware<a name="step2"></a>
- Type-C USB cable
- Personal Computer
- 1 Power Adapter 12V
- [FRDM S32K312](https://www.nxp.com/design/design-center/development-boards-and-designs/S32K312MINI-EVB)[<p align="center"><img src="images/S32K312MINI-EVB.png" width="400"/></p>](images/S32K312MINI-EVB.png)

## 3. Setup<a name="step3"></a>
### 3.1 Import the project to S32 Design Studio IDE

1. Open S32 Design Studio IDE, in the Dashboard Panel, choose **Import project from Application Code Hub**.
   ​	[<p align="center"><img src="images/import_project_1.png" width="400"/></p>](./images/import_project_1.png)

2. Found demo you need by searching for the name directly. Open the project, click the **GitHub link**, S32 Design Studio IDE will automatically retrieve project attributes then click **Next>**.
    [<p align="center"><img src="images/import_project_2.png" width="600"/></p>](./images/import_project_2.png) 
    [<p align="center"><img src="images/import_project_3.png" width="600"/></p>](./images/import_project_3.png) 

3. Select **main** branch and then click **Next>**.
   ​	[<p align="center"><img src="images/import_project_4.png" width="600"/></p>](./images/import_project_4.png)

4. Select your local path for the repo in **Destination->Directory** window. The S32 Design Studio IDE will clone the repo into this path, click **Next>**.
      [<p align="center"><img src="images/import_project_5.png" width="600"/></p>](./images/import_project_5.png)

5. Select **Import existing Eclipse projects** then click **Next>**.
      [<p align="center"><img src="images/import_project_6.png" width="600"/></p>](./images/import_project_6.png)

6. Select the project in this repo (only one project in this repo) then click **Finish**.
      [<p align="center"><img src="images/import_project_7.png" width="600"/></p>](./images/import_project_7.png)

### 3.2 Generating, building and running the example application
1. In Project Explorer, right-click the project and select **Update Code and Build Project**. This will generate the configuration (Pins, Clocks, Peripherals), update the source code and build the project using the active configuration (e.g. Debug_FLASH).
Make sure the build completes successfully and the *.elf file is generated without errors.
[<p align="center"><img src="images/update_and_build.png" width="200"/></p>](./images/update_and_build.png)
Press **Yes** in the **SDK Component Management** pop-up window to continue.

2. Go to **Debug** and select **Debug Configurations**. There will be a debug configuration for this project:
[<p align="center"><img src="images/Debug_config.png" width="200"/></p>](./images/Debug_config.png)

        Configuration Name                  Description
        -------------------------------     -----------------------
        $(example)_debug_flash_pemicro      Debug the FLASH configuration using PEmicro probe

    Select the desired debug configuration and click on **Debug**. Now the perspective will change to the **Debug Perspective**.
    Use the controls to control the program flow.


## 4. Results<a name="step4"></a>
 The onboard LED should toggle with a 500ms cadence (500ms on, 500ms off).

## 5. Support<a name="step6"></a>
*Provide URLs for help here.*

#### Project Metadata

<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-S32K312MINI&ndash;EVB-blue)]()

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://mcuxpresso.nxp.com/appcodehub?peripheral=gpio)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-S32%20Design%20Studio%20IDE-orange)](https://mcuxpresso.nxp.com/appcodehub?toolchain=s32_design_studio_ide)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected functionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/X-Follow%20us%20on%20X-black.svg)](https://x.com/NXP)

## 6. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        |February 17<sup>th</sup> 2026|

