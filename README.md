# Project Elijah
## Cedarville University NASA Student Code

Code for all three microcontroller systems on the Cedarville University NASA Student Launch rocket for 2024-2025.

## Installation

### 1. IDE Installation

Get a JetBrains student license from [their website](https://www.jetbrains.com/community/education/#students). This README contains setup for JetBrains, but any IDE can technically be used. Download and install CLion and PyCharm. I also recommend the "Serial Port Monitor" plugin from JetBrains (install from Settings  > Plugins).

### 2. C/C++ Compiler Installation

Download and install [Visual Studio](https://visualstudio.microsoft.com/) and in the installer, install Visual Studio Build tools. Restart your computer. When using the command line, search in Windows for "Developer Command Prompt for VS 2022" (or 2019).

Install [arm-none-eabi](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) for Windows. Download and run the installer (the `.exe`, not the `.zip`), and make sure you select "add to path".

### 3. Pico SDK and Picotool Installation

For Windows, download the latest Windows binary for [libusb](https://libusb.info/). Extract the `.7z` to somewhere (I put everything in `C:\pico-dev`) and set up your `LIBUSB_ROOT` environment variable to point to it. 

Clone the [pico-sdk repository](https://github.com/raspberrypi/pico-sdk) and run `git submodule update --init` within it. Then add `mbedtls` by using `git submodule update --init lib/mbedtls`. Set your `PICO_SDK_PATH` environment variable to the root of the cloned repo (ex: `C:\pico-dev\pico-sdk`). You'll also need to build `pioasm`, by going into `%PICO_SDK_PATH%\tools\pioasm` and then running:

```cmd
mkdir build
cd build
cmake -G "NMake Makefiles" ..
nmake
cmake --install .
```

Also clone the [Picotool repository](https://github.com/raspberrypi/picotool) and build it using:

```cmd
mkdir build
cd build
cmake -G "NMake Makefiles" ..
nmake
cmake --install .
```

Add the `bin` directory from both outputs to your path (should be `C:\Program Files (x86)\picotool\bin` and `C:\Program Files (x86)\pioasm\bin`). Open up another command prompt and make both sure `picotool` and `pioasm` are discoverable.

### 4. Zadig Installation

Install (Zadig)[https://zadig.akeo.ie] by just downloading and installing it. When you run it, press install driver. You can close it after installation and forget it exists.

### 5. CLion Setup
Clone this repository, then update the submodules using:

```cmd
git submodule update --init
```

When you open CLion, open the cloned project; you should be prompted to set up a toolchain. If you're not, go to Settings > Build, Execution, Deployment > Toolchain, and then add a new one. I called mine "Pico". You'll need to set up your C, C++, and GDB executables. If you left the arm-none-eabi setup as default, these should be in `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\`. Use arm-none-eabi-gcc for C, gpp for C++, and gdb for GDB.

It should also prompt you to set up CMake profiles, you'll need to set up two. For both profiles, make sure you're using the Pico toolchain you set up, and using "Ninja" as the generator. If it doesn't open Settings > Build, Execution, Deployment > CMake, and then add another profile; call it "Debug". Set the CMake build type to Debug Use `build-debug` for the build directory use these options:

```
-DENTER_USB_BOOT_ON_EXIT=ON -DSTATUS_LED_ENABLE=ON -DFLASH_SIM_ENABLE=ON
```

Add another profile called "Release", with the build directory `build-release` and these options:

```
-DSTATUS_LED_ENABLE=ON -DFLASH_SIM_ENABLE=ON
```

For the most part, we'll be using Release, Debug is used with GDB if you ever hook up another Pico too it. It's 9/10 times not worth using it though.

You'll also need to set up a run configuration for each target. Go to Run Configurations (top right) > Edit Configurations. Add a new one, for the target, pick `elijah-*` (where `*` is payload, override, etc.), and for executable, press the dropdown, then Custom Executable and put `install.bat` (or `install.sh` on MacOS/Linux). For Program arguments, put `$CMakeCurrentProductFile$`, and for Current Working directory put `$CMakeCurrentBuildDir$`. Check "Emulate terminal in the output console". You can clone this configuration for each `elijah-*` target.

### 6. PyCharm Setup

Open up PyCharm, and open the `state-framework-com` directory within `project-elijah`. At the top, press "Configure Python Interpreter" then, add a new local interpreter, generated Virtualenv, based on Python 3.12 (download and install if needed). Once your venv is set up, you'll need to install packages by running this in the PyCharm terminal (PyCharm's terminal will activate the venv for you):

```cmd
pip3 install -r requirements.txt
```

Run "Main" to start reading data from any Pico using the state framework.

## Common Issues

### COM/Serial Port Not Showing Up

If the Pico suddenly stops showing its Serial port on your computer, download [flash_nuke.uf2](https://github.com/dwelch67/raspberrypi-pico/blob/main/flash_nuke.uf2) from GitHub. Put the Pico into BOOTSEL, and then drag and drop this `.uf2` onto it. This will wipe the entire flash.