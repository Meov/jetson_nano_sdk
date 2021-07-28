# jetson_nano_sdk
its a jetson nano project both for image  (kernel space) and some apps(User space)


2021-07-28

UPDATE: rebuilt SDK project with local.Mk like "Andiod.Mk" style

USER SPACE BUILD: APPS
sample:
	1. jetson_nano_sdk/app/hello_world
	2. jetson_nano_sdk/app/power_switch
compile:
	make hello_world
	make power_switch
will generate target bin files in jetson_nano_sdk/out/app_out/app_targets

=============================================================================================	


KERNEL SPACE BUILD: kernel bootloader and rootfs

build apdds:
samples:
	1. kernel
	2. uboot
	3. rootfs

build kernel:
	<1. cp linux kernel source files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/kernel"
	<2. modify "jetson_nano_sdk/apdd/apdd.Mk" about kernel config 
	<3. using "make kernel" to build kernel

build bootloader:
	<1. cp uboot source files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/bootloader"
	<2. modify "jetson_nano_sdk/apdd/apdd.Mk" about bootloader config
	<3. uding "make bootloader" to build uboot

build rootfs:
        <1. cp busybox files to the "jetson_nano/jetson_nano_sdk/apdd/bsp/rootfs"
        <2. modify "jetson_nano_sdk/apdd/apdd.Mk" about rootfs config
        <3. uding "make rootfs" to build uboot


