Proton kernel for Lenovo A820
===============================

Compile with arm-eabi 4.7 or 4.8 toolchain

### Build the kernel
$ cd kernel
$ ./build.sh

### Build the kernel with modules
$ ./build.sh -m
or
$ ./build.sh -modules

You can find the flashable zip in out directory

### Cleanup
$ ./clean.sh
