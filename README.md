# jetson_nano_sdk
its a jetson nano project both for image  (kernel space) and some apps(User space)


date: 2021-07-28

UPDATE: rebuilt SDK project with local.Mk like "Andiod.Mk" style

USER SPACE BUILD: APPS
sample:
	1. jetson_nano_sdk/app/hello_world
	2. jetson_nano_sdk/app/power_switch
compile:
	make hello_world
	make power_switch
will generate target bin files in jetson_nano_sdk/out/app_out/app_targets/apps


date: 2021-07-29
shared libs and static libs compiling

<1 shared libs:
	reference libtest_shared local.Mk 
	using "make libtest_shared" to compile a shared lib

   apps using shared libs:
	reference test_shared local.Mk	
	1. using "make libtest_shared" to compile a shared lib
	2. using "make test_shared" to compile a test app using share lib

<2 static libs:
	reference libtest_static local.Mk
	using "make libtest_static" to compile a static lib

   apps using static libs:
        reference test_static local.Mk
        1. using "make libtest_static" to compile a static lib
        2. using "make test_static" to compile a test app using static lib


=============================================================================================	


KERNEL SPACE BUILD: kernel bootloader and rootfs

build apdds:
samples:
	1. kernel
	2. uboot
	3. rootfs

<1 build kernel:
	1. cp linux kernel source files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/kernel"
	2. modify "jetson_nano_sdk/apdd/apdd.Mk" about kernel config 
	3. using "make kernel" to build kernel

<2 build bootloader:
	1. cp uboot source files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/bootloader"
	2. modify "jetson_nano_sdk/apdd/apdd.Mk" about bootloader config
	3. uding "make bootloader" to build uboot

<3 build rootfs:
        1. cp busybox files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/rootfs"
        2. modify "jetson_nano_sdk/apdd/apdd.Mk" about rootfs config
        3. uding "make rootfs" to build uboot


