#################################################################
#                      NAS build                                #
#                                                               #
#  add this makefile segment to your toolchain build makefile   #
#               added by gengxianhu for NAS                     #
#################################################################

# path setting
# modify this path info for your environment
NAS_PATH=$(TOPDIR)/apps/nas
NAS_KERNEL_MODULE_PATH=${INSTALL_ROOT}/lib/modules/2.6.15/nas
NASPREREQUISITES=$(NAS_PATH)/prerequisites

hotplug_build:
	cp -f $(NASPREREQUISITES)/hotplug $(INSTALL_ROOT)/sbin/hotplug
	chmod +x $(INSTALL_ROOT)/sbin/hotplug
	
usbp_build : 
	@echo making usbp ...
	cd $(NAS_PATH)/usbp && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld 
	cp -f $(NAS_PATH)/usbp/usbp $(INSTALL_ROOT)/sbin 
	chmod +x $(INSTALL_ROOT)/sbin/usbp
	cp -f $(NAS_PATH)/usbp/usbp $(TFTPPATH)

nas_module_pre:
	if test -d $(NAS_KERNEL_MODULE_PATH) ; then echo $(NAS_KERNEL_MODULE_PATH) exists ; else mkdir $(NAS_KERNEL_MODULE_PATH); fi

fuse_build: nas_module_pre
	@echo making fuse kernel module ...
	cd $(NAS_PATH)/fuse-2.7.4 && \
	./configure --enable-kernel-module --with-kernel=$(KERNELPATH) --build=i686-linux --host=mips-linux CC=$(TOOLPREFIX)gcc AR=$(TOOLPREFIX)ar LD=$(TOOLPREFIX)ld && \
	cd $(NAS_PATH)/fuse-2.7.4/kernel && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld 
	@echo copy fuse.ko kernel module
	cp -f $(NAS_PATH)/fuse-2.7.4/kernel/fuse.ko $(NAS_KERNEL_MODULE_PATH)/fuse.ko

nas_kernel_module: nas_module_pre fuse_build  
	@echo copy usb kernel module
	cp -f $(KERNELPATH)/drivers/usb/core/usbcore.ko $(NAS_KERNEL_MODULE_PATH)/usbcore.ko
	cp -f $(KERNELPATH)/drivers/usb/host/ehci-hcd.ko $(NAS_KERNEL_MODULE_PATH)/ehci-hcd.ko
	cp -f $(KERNELPATH)/drivers/usb/storage/usb-storage.ko $(NAS_KERNEL_MODULE_PATH)/usb-storage.ko
	@echo copy fs kernel module
	cp -f $(KERNELPATH)/fs/fat/fat.ko $(NAS_KERNEL_MODULE_PATH)/fat.ko
	cp -f $(KERNELPATH)/fs/vfat/vfat.ko $(NAS_KERNEL_MODULE_PATH)/vfat.ko
	cp -f $(KERNELPATH)/fs/nls/nls_utf8.ko $(NAS_KERNEL_MODULE_PATH)/nls_utf8.ko
	cp -f $(KERNELPATH)/fs/nls/nls_cp850.ko $(NAS_KERNEL_MODULE_PATH)/nls_cp850.ko
		

ntfs_build:
	@echo making ntfs ...
	cd $(NAS_PATH)/ntfs-3g-1.2918 && \
	./configure --build=i686-linux --host=mips-linux CC=$(TOOLPREFIX)gcc CXX=$(TOOLPREFIX)g++ AR=$(TOOLPREFIX)ar ac_cv_func_mbsinit=no ac_cv_header_syslog_h=no && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld 
	@echo copy ntfs-3g
	cp -f $(NAS_PATH)/ntfs-3g-1.2918/src/.libs/ntfs-3g $(INSTALL_ROOT)/bin/ntfs-3g
	@echo copy libntfs-3g.so.38.0.0
	cp -f $(NAS_PATH)/ntfs-3g-1.2918/libntfs-3g/.libs/libntfs-3g.so.38.0.0 $(INSTALL_ROOT)/lib/libntfs-3g.so.38.0.0
	cd $(INSTALL_ROOT)/lib && \
	ln -fs libntfs-3g.so.38.0.0 libntfs-3g.so.38 &&\
	ln -fs libntfs-3g.so.38.0.0 libntfs-3g.so

samba_pre:
	if test ! -d $(INSTALL_ROOT)/etc/samba/lib; then mkdir -p $(INSTALL_ROOT)/etc/samba/lib ; fi
	cp -f $(NASPREREQUISITES)/samba/lib/* $(INSTALL_ROOT)/etc/samba/lib

samba_build: samba_pre
	@echo build samba-3.0.14a ...
	cd $(NAS_PATH)/samba-3.0.14a/source && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld min
	@echo copy samba
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/libbigballofmud.so $(INSTALL_ROOT)/lib/libbigballofmud.so
	cd $(INSTALL_ROOT)/lib && ln -fs libbigballofmud.so libbigballofmud.so.0
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/smbd $(INSTALL_ROOT)/usr/sbin/smbd
	cp -f $(NAS_PATH)/samba-3.0.14a/source/bin/smbpasswd $(INSTALL_ROOT)/usr/bin/smbpasswd

nas_build: hotplug_build usbp_build nas_kernel_module ntfs_build samba_build squashfs_build
	@echo build nas over ...

usbp_clean:
	@echo usbp clean ...
	cd $(NAS_PATH)/usbp && make clean
	
fuse_clean:
	@echo fuse clean ...
	-cd $(NAS_PATH)/fuse-2.7.4 && make distclean

ntfs_clean:
	@echo ntfs-3g clean ...
	-cd $(NAS_PATH)/ntfs-3g-1.2918 && make distclean

samba_clean:
	@echo samba clean ...
	cd $(NAS_PATH)/samba-3.0.14a/source && make clean

nas_clean: usbp_clean fuse_clean ntfs_clean samba_clean
	@echo clean nas over ...
