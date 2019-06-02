TP-LINK GPL code readme

1. This package contains all GPL code used by TP-LINK Routers/APs with Linux OS.
2. All components have been built successfully on Redhat Enterprise Linux 4.0 Workstation.
3. Compiling components in this package on platforms other than Redhat Enterprise Linux 4.0 Workstation may have unexpected results.
4. Recommended using "root" or "sudo" command to build the code.
5. After building code, you can get the uboot and linux kernel image, then you can make rootfs with mksquashfs tool, but you can't upgrade these images to your router through web management page.

BOARD_TYPE definitions
1. ap123: TL-WR841N/ND 8.0

Build Instructions
1. All build targets are in ~/ap123_gpl/build/Makefile, you should enter this directory to build components.
2. Pre-built toolchain is avaliable in this package.
3. Toolchain (include uclibc) source code is avaliable in toolchain_src.
4. Prepare Pre-built toolchain:  
	make BOARD_TYPE=ap123 toolchain_prep	(You can decide the BOARD_TYPE by the BOARD_TYPE definitions.)
5. Prepare basic root filesystem:
	make BOARD_TYPE=ap123 fs_prep
6. Build fake root tool:
	make BOARD_TYPE=ap123 fakeroot_build
7. Build components:
	1)Build u-boot bootloader
		make BOARD_TYPE=ap123 uboot
	
	2)Build linux kernel image
		make BOARD_TYPE=ap123 kernel_build

	3)Build some kernel modules, such as netfilter, netsched.
		make BOARD_TYPE=ap123 kernel_modules_prep
		make BOARD_TYPE=ap123 netfilter netsched ts_kmp pppol2tp pptp_module
	
	4)Build some wireless support software, such as hostapd, wpa supplicant.
		make BOARD_TYPE=ap123 wireless_prep
		make BOARD_TYPE=ap123 wireless_tools
		make BOARD_TYPE=ap123 wpa2
		
	5)Build some application, such as busybox, iptables and so on.
		make BOARD_TYPE=ap123 busybox pppoe l2tp bpa iptables tc_build lltd arp