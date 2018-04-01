## navXTimeSync

A Linux/MacOS C++ library for Kauai Labs navX-MXP sensor

## Usage

* Installs the library into the system and compiles test executable

* Library installed under the name `libahrs.so` 

* Library exposed with header file `"AHRS.h"`

* Includes test code under `TimeStamp.h/cpp` compiled to executable `NavXTimeStamp`

* Example compilation: `g++ -std=c++11 -o NavXTimeStamp TimeStamp.cpp -lahrs`

## Installation

```bash
# cd working directory
git clone https://github.com/FRC900/navXTimeSync.git 
cd navXTimeSync
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
```
