# WindRider Firmware
## Dependencies
1. [arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) - ARM toolchain.
2. [CMake](https://cmake.org/download/) - Build system.
3. SWD probe - Used for flashing and debugging. Can be either st-link or j-link.

## Build
Specify ARM toolchain location. Set [`TOOLCHIN_PATH`](CMakeLists.txt#5) in CMakeLists.txt to point to the toolchain's bin directory.
```CMake
SET(TOOLCHIN_PATH "/your_path/gcc-arm-none-eabi-9-2019-q4-major/bin")
```
Navigate to the project root directory. Create and enter a build directory.
```sh
mkdir build
cd build
```
Run cmake.
```sh
cmake ..
```
On success CMake should exit with:
```
...
-- Configuring done
-- Generating done
-- Build files have been written to: /…/windrider/build
```
Compile the code.
```sh
make -j4
```
Build folder will contain three firmware files *windrider.bin*, *windrider.hex*, *windrider.elf* — these are compiled binary blobs ready to be flashed to the microcontroller.

## Flash
Connect windrider pcb to the swd probe. Follow instructions for J-Link or ST-Link based on your probe.

### J-Link
There are two ways to upload firmware using J-Link - Command Line and GUI.

#### JFlashLite GUI
Run JFlashLite with the following settings.

In the next prompt select *windrider.hex* as data file. Click **Erase**, then **Program**.
#### JLinkExe Command Line
Navigate to build folder. Connect to target.
```shell
JLinkExe -device "STM32F103C6 (allow opt. bytes)" -if SWD -speed 4000 -autoconnect
```
Reset, erase, load, verify.
```
r
erase
loadbin "windrider.bin" 0x08000000 
verifybin "windrider.bin" 0x08000000 
```
### ST-Link
Navigate to build folder. Run `st-flash`.
```
st-flash write windrider.bin 0x08000000
```

