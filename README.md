# WindRider Firmware
## Dependencies
1. [arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) - ARM toolchain.
2. [CMake](https://cmake.org/download/) - Build system.
3. SWD probe - Used for flashing and debugging. Can be either st-link or j-link.

## Build
Specify ARM toolchain location. Set [`TOOLCHIN_PATH`](CMakeLists.txt#5) in CMakeLists.txt to point to your toolchain's bin directory.
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

<img src=".readme_img/JFlashLite.png" width="600">

In the next prompt select *windrider.hex* as data file. Click **Erase**, then **Program**.
#### JLinkExe Command Line
Navigate to build folder. Connect to target.
```shell
JLinkExe -device "STM32F103C8" -if SWD -speed 4000 -autoconnect
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

## Run
Connect host computer to WindRider board via USB. Use a modem control software (e.g. [minicom](https://linux.die.net/man/1/minicom)) or serial port API (e.g. [pyserial](https://pyserial.readthedocs.io/en/latest)).

To print a full list of commands and arguments type `?` and hit return.

Some examples of valid commands:  
`solenoid 1 5 20` -- Enable impactor at channel 1, on_time = 5ms, off_time = 20ms.  
`servo 0 100`     -- Set servo at channel 0 to 100 deg. 

### Command Table
| Token | Argument 1 | Argument 2 | Argument 3 | Description |
|:-----:|:----------:|:----------:|:----------:|:------------|
|current| None       | None       | None       | Print suction current |
|led    |#channel [0-3]| current [0-1500]mA| None | Set led current |
|left   |[–30000…30000]| None   | None       | Set left motor speed |
|right  |[–30000…30000]| None   | None       | Set right motor speed |
|servo  |#channel [0-1]|Angle [0-180]deg| None | Set servo angle |
|solenoid|#channel [0-1]|[on] or [off]| None   | Enable/Disable impactor |
|solenoid|#channel [0-1]|[0-5000]ms on_time|[0-5000]ms off_time| Set impactor on/off time intervals|
|straight|[–30000…30000]| None  | None       | Speed loaded into both motor drivers synchronously|
|suction|[on] or [off]| None      | None       | Enable/Disable suction power|
|uart   | [forward] |["cmd_to_send"]| None     | Forward "cmd_to_send" to rs232|
|uart   | [reply]   |[on] or [off]| None       | Print incoming feedback from rs232|

A detailed command description can be found in the project wiki.

## Contribute
CCNY Robotics Lab welcomes you, human! You are interested in working on our embedded projects and..

### You don't know where to start
A good chunk of this project is built on top of the STM Pheripheral Library; following steps will familiarize you with with its ~~bugs~~ capabilities.

  1. *Wake up, Neo!* Get a blue pill, or any other stm32 dev board. You will need a debugger probe (e.g. st-link or j-link). Most of the official stm 32 dev boards come with st-link on board.  
  2. *Goodbye Arduino.* Write a blinky. Start a timer to trigger an interrupt. Send the core to sleep. Interrupt to change the led state and back to sleep.  
  3. *Feel the pulse.* Set up a timer driven pwm to change the led brightness. Make a potentiometer to controll the brightness. Main loop must be empty.   
  4. *Plug and play!* Set up a USB Communication Device Class. Make your led dimmable form a serial terminal.  
  5. Report to base.  
  
  Make sure you can build and flash your code without using an IDE.
  
  
