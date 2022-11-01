# Railbot
> Railbot is a open-source software for biped robot control, especially, for NAO robot. If you are interested in robot motion control, use this code as you wish. Any problem, please contact with us. email: ziihan_xu@163.com

## Flash
You must flash the NAO robot to the robocup version first and unlock the root permission. More details see [bhuman wiki](https://wiki.b-human.de/coderelease2022/getting-started/).

## Requirments
+ CMake >=  3.6
+ libeigen3-dev
+ libboost1.71-dev
+ libboost-system1.71-dev
+ [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog)

## Linked Libraries
Copy the library files form your pc to the same path on NAO robot. You can use `ldd` command to check the detailed path information of the excutable files.
we need:
+ -leigen-quadprog
+ -lboost_system

## Build
Build this program on your PC and no need to complie on the robot. We build our code on Ubuntu20.04, PC. Use the `Make.sh` script to build the project. This scrip will automaticly make directories `Build` and `Bin`. After the build process, you can find the excutable files, `lola_conn` and `controller`, in `Bin`. 

## Run
Copy the `lola_conn` and `controller` to NAO home path, usually `/home/nao`, `lola_conn` and `controller` need to be run simultaneously. We recommend that run `lola_conn` first and then run `controller`.  

If the program does not work property, use `ldd` to check the linked libraries, you will see the following, the outputs maybe different depend on your setup. Find the missing library files and copy them from your PC. That's why we use Ubuntu20.04 to build our program. If you use other version of Linux system, the library files maybe different and may not work as well.
```bash
ldd controller 
	linux-vdso.so.1 (0x00007ffc5f781000)
	libeigen-quadprog.so.1 => /usr/local/lib/libeigen-quadprog.so.1 (0x00007fc03b5fc000)
	librt.so.1 => /lib/x86_64-linux-gnu/librt.so.1 (0x00007fc03b5ae000)
	libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fc03b3cc000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fc03b27d000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fc03b262000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fc03b23f000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fc03b04b000)
	/lib64/ld-linux-x86-64.so.2 (0x00007fc03b639000)
```

```bash
ldd lola_conn 
	linux-vdso.so.1 (0x00007ffe82fc0000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fab7a095000)
	librt.so.1 => /lib/x86_64-linux-gnu/librt.so.1 (0x00007fab7a08b000)
	libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fab79ea9000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fab79e8e000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fab79c9c000)
	/lib64/ld-linux-x86-64.so.2 (0x00007fab7a0fe000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fab79b4d000)

```