#################################################################
#                      NAS build                                #
#                                                               #
#  add this makefile segment to your toolchain build makefile   #
#               added by gengxianhu for NAS                     #
#################################################################

# path setting
# modify this path info for your environment
NAS_PATH=$(TOPDIR)/apps/nas
NASPACKAGE_PATH=$(NAS_PATH)/naspackage
ROOTFS_PATH=$(INSTALL_ROOT)

# NAS build rule, don't copy NAS file to filesystem
usbp_build : 
	@echo making usbp ...
	cd $(NAS_PATH)/usbp && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld 

fuse_build:
	@echo making fuse kernel module ...
	cd $(NAS_PATH)/fuse-2.7.4 && \
	./configure --enable-kernel-module --with-kernel=$(KERNELPATH) --build=i686-linux --host=mips-linux CC=$(TOOLPREFIX)gcc AR=$(TOOLPREFIX)ar LD=$(TOOLPREFIX)ld && \
	cd $(NAS_PATH)/fuse-2.7.4/kernel && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld

ntfs_build:
	@echo making ntfs ...
	cd $(NAS_PATH)/ntfs-3g-1.2918 && \
	./configure --build=i686-linux --host=mips-linux CC=$(TOOLPREFIX)gcc CXX=$(TOOLPREFIX)g++ AR=$(TOOLPREFIX)ar ac_cv_func_mbsinit=no ac_cv_header_syslog_h=no && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld 

samba_build:
	@echo build samba-3.0.14a ...
	cd $(NAS_PATH)/samba-3.0.14a/source && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld min

nas_build: usbp_build fuse_build ntfs_build samba_build
	@echo build nas over ...


# clean target for NAS
nas_clean: NAS_CLEAN = $(MAKE) clean 
nas_clean: NAS_DISTCLEAN = $(MAKE) distclean

# just clean file which is produced by nas_build
nas_clean: 
	@echo clean nas ...
	cd $(NAS_PATH)/usbp && $(NAS_CLEAN) 
	-cd $(NAS_PATH)/fuse-2.7.4 && $(NAS_DISTCLEAN)
	-cd $(NAS_PATH)/ntfs-3g-1.2918 && $(NAS_DISTCLEAN)
	cd $(NAS_PATH)/samba-3.0.14a/source && $(NAS_CLEAN)


# copy NAS file to naspackage
nas_copy: 
	@echo make sure target dir exists in naspackage
	if test ! -d $(NASPACKAGE_PATH)/usbp ; then mkdir $(NASPACKAGE_PATH)/usbp; fi
	if test ! -d $(NASPACKAGE_PATH)/ntfs-3g; then mkdir $(NASPACKAGE_PATH)/ntfs-3g; fi
	if test ! -d $(NASPACKAGE_PATH)/module; then mkdir $(NASPACKAGE_PATH)/module; fi
	@echo copy nas binary file to package directory ...
	@echo copy usbp
	cp -f $(NAS_PATH)/usbp/usbp $(NASPACKAGE_PATH)/usbp/usbp
	@echo copy fuse.ko
	cp -f $(NAS_PATH)/fuse-2.7.4/kernel/fuse.ko $(NASPACKAGE_PATH)/module/fuse.ko
	@echo copy usb kernel module
	cp -f $(KERNELPATH)/drivers/usb/core/usbcore.ko $(NASPACKAGE_PATH)/module/usbcore.ko
	cp -f $(KERNELPATH)/drivers/usb/host/ehci-hcd.ko $(NASPACKAGE_PATH)/module/ehci-hcd.ko
	cp -f $(KERNELPATH)/drivers/usb/storage/usb-storage.ko $(NASPACKAGE_PATH)/module/usb-storage.ko
	@echo copy fs kernel module
	cp -f $(KERNELPATH)/fs/fat/fat.ko $(NASPACKAGE_PATH)/module/fat.ko
	cp -f $(KERNELPATH)/fs/vfat/vfat.ko $(NASPACKAGE_PATH)/module/vfat.ko
	cp -f $(KERNELPATH)/fs/nls/nls_utf8.ko $(NASPACKAGE_PATH)/module/nls_utf8.ko
	cp -f $(KERNELPATH)/fs/nls/nls_cp850.ko $(NASPACKAGE_PATH)/module/nls_cp850.ko
	@echo copy ntfs-3g
	cp -f $(NAS_PATH)/ntfs-3g-1.2918/src/.libs/ntfs-3g $(NASPACKAGE_PATH)/ntfs-3g/ntfs-3g
	@echo copy libntfs-3g.so.38.0.0
	cp -f $(NAS_PATH)/ntfs-3g-1.2918/libntfs-3g/.libs/libntfs-3g.so.38.0.0 $(NASPACKAGE_PATH)/ntfs-3g/libntfs-3g.so.38.0.0
	@echo copy samba
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/libbigballofmud.so $(NASPACKAGE_PATH)/samba/libbigballofmud.so
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/smbd $(NASPACKAGE_PATH)/samba/smbd
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/smbpasswd $(NASPACKAGE_PATH)/samba/smbpasswd

# run nas package shell to install NAS file to filesystem
nas_package:
	@echo package processing ...
	cd $(NASPACKAGE_PATH) && ./shell.sh $(ROOTFS_PATH)
	
#  build NAS all, you can use prerequestes seperately by yourself 
nas: nas_clean nas_build nas_copy nas_package
	@echo nas module process over ...
