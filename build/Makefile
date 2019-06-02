ifndef BOARD_TYPE
$(error "You have to define Board Type to use this Makefile")
#export BOARD_TYPE=ap81fus
#export BOARD_TYPE=ap93fus
endif

#
# board related targets
#
include Makefile.$(BOARD_TYPE)

include snmp.mk

export WEBDIR=$(TOPDIR)/webs/web_pages/

######## build fakeroot host tool ########
fakeroot_build:
	cd $(TOPDIR)/util/fakeroot/fakeroot-1.18.2 && \
	if test -e Makefile; then make distclean; fi ;
	cd $(TOPDIR)/util/fakeroot/fakeroot-1.18.2 && ./configure --prefix=$(TOPDIR)/util/fakeroot/install
	cd $(TOPDIR)/util/fakeroot/fakeroot-1.18.2 && make
	cd $(TOPDIR)/util/fakeroot/fakeroot-1.18.2 && make install
	
######## prepare toolchain ########
toolchain_prep:
	@echo Uncompressing toolchain, please wait for a few minutes ...
	cd $(TOPDIR)/toolchain && \
	tar -jxf gcc-4.3.3.tar.bz2 && \
	mv gcc-4.3.3 $(TOPDIR)/build

#
#  CONFIG setting
#

# modules used by all boards
export BOARD_KERNEL_MODULES = stats netfilter wlan_warn netsched enet_build br_filter ts_kmp pppol2tp pptp_module tp_domain

# modules specified by special board type
ifneq ($(filter $(BOARD_TYPE), ap123 ap123-803), )
BOARD_KERNEL_MODULES += usb_serial usb_storage
endif

ifeq ($(CONFIG_USB_SERIAL_MODULE),y)
BOARD_KERNEL_MODULES += usb_serial
endif

# applications used by all board types 
BOARD_APPLICATIONS = busybox pppoe l2tp bpa iptables tc_build lltd net_ioctl arp snmp

# applications specified by special board
ifneq ($(filter $(BOARD_TYPE), pb92 ap123-8M), )
BOARD_APPLICATIONS += nas ftpserver ushare
endif

ifeq ($(CONFIG_3G_TOOLS),y)
CONFIG_3G_TOOLS=y
BOARD_APPLICATIONS += 3g_tools
endif

#ifeq ($(BOARD_TYPE),ap99)
#CONFIG_3G_TOOLS=y
#BOARD_APPLICATIONS += 3g_tools
#endif
#ifeq ($(BOARD_TYPE),ap99tpl)
#CONFIG_3G_TOOLS=y
#BOARD_APPLICATIONS += 3g_tools
#endif

ifneq ($(findstring ap9, $(BOARD_TYPE)), )
BOARD_APPLICATIONS += wlanreg
endif

#
# common build
#

fs_prep:install_root_clean util_chmod check_tftp libs_copy etc_prep empty_dir_prep

install_root_clean:
	-rm -rf $(INSTALL_ROOT)/*
	
util_chmod:
	-cd $(TOPDIR)/util && chmod 777 cplibs md5_des mkdev mksquashfs pad
	-chmod 777 $(TOPDIR)/webs/web_pages/utils/getini/gather
	-chmod 777 $(TOPDIR)/webs/web_pages/utils/mkrc/mkrc
	-chmod 777 $(TOPDIR)/webs/web_pages/utils/mkrc/rmtag
	-chmod 777 $(TOPDIR)/webs/web_pages/utils/web_collector/webc
	
check_tftp:
	mkdir -p $(IMAGEPATH)
	if test -d $(TFTPPATH); then echo $(TFTPPATH) exists; else mkdir -p $(TFTPPATH); fi;

libs_copy:
	$(TOPDIR)/util/cplibs
	
dev_create:
	rm -rf $(INSTALL_ROOT)/dev/*
	$(TOPDIR)/util/mkdev

etc_prep:
	rm -rf $(INSTALL_ROOT)/etc
	cp -r $(TOPDIR)/filesystem/ap93fus/etc $(INSTALL_ROOT)/

empty_dir_prep:
	if test -d $(INSTALL_ROOT)/mnt; then echo $(INSTALL_ROOT)/mnt exists; else mkdir $(INSTALL_ROOT)/mnt; fi;
	if test -d $(INSTALL_ROOT)/proc; then echo $(INSTALL_ROOT)/proc exists; else mkdir $(INSTALL_ROOT)/proc; fi;
	if test -d $(INSTALL_ROOT)/root; then echo $(INSTALL_ROOT)/root exists; else mkdir $(INSTALL_ROOT)/root; fi;
	if test -d $(INSTALL_ROOT)/tmp; then echo $(INSTALL_ROOT)/mnt exists; else mkdir $(INSTALL_ROOT)/tmp; fi;
	if test -d $(INSTALL_ROOT)/var; then echo $(INSTALL_ROOT)/var exists; else mkdir -p $(INSTALL_ROOT)/var/run; fi;
# added by tf, /sys for mounting sysfs
	if test -d $(INSTALL_ROOT)/sys; then echo $(INSTALL_ROOT)/sys exists; else mkdir $(INSTALL_ROOT)/sys; fi;
	
#
# kernel modules
#

KERNEL_MODULE_PATH=$(TOPDIR)/images/$(BOARD_TYPE)/kernel_modules/

kernel_modules: kernel_modules_prep $(BOARD_KERNEL_MODULES) 

kernel_modules_prep:
	test -d $(KERNEL_MODULE_PATH) || mkdir -p $(KERNEL_MODULE_PATH)

kernel_modules_install:
	mkdir -p $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/kernel/
	cp -Rf $(KERNEL_MODULE_PATH)/*.ko $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/kernel/
	
stats:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/stats modules
	cp $(TOPDIR)/kernel_modules/stats/statistics.ko $(KERNEL_MODULE_PATH)

netfilter:
	if [ "$(KERNELVER)" = "2.6.15" ]; then \
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/ipv4/netfilter modules ; \
	else \
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/netfilter modules ; \
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/ipv4/netfilter modules ; \
	fi;
	-cp $(KERNELPATH)/net/ipv4/netfilter/*.ko $(KERNEL_MODULE_PATH)
	-cp $(KERNELPATH)/net/netfilter/*.ko $(KERNEL_MODULE_PATH)	

wlan_warn:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/wlan_warn modules
	cp $(TOPDIR)/kernel_modules/wlan_warn/wlan_warn.ko $(KERNEL_MODULE_PATH)
	
tp_domain:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/tp_domain modules
	cp $(TOPDIR)/kernel_modules/tp_domain/tp_domain.ko $(KERNEL_MODULE_PATH)

netsched:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/sched modules	
	cp $(KERNELPATH)/net/sched/*.ko $(KERNEL_MODULE_PATH)

ts_kmp:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=lib modules	
	cp $(KERNELPATH)/lib/ts_kmp.ko $(KERNEL_MODULE_PATH)
	
br_filter:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/br_hook modules
	cp $(TOPDIR)/kernel_modules/br_hook/br_filter.ko $(KERNEL_MODULE_PATH)
	
flashid:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/flash_id modules
	cp $(TOPDIR)/kernel_modules/flash_id/flashid.ko $(KERNEL_MODULE_PATH)
	
antind:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/anti_nat_detect modules
	#cp $(TOPDIR)/kernel_modules/anti_nat_detect/ipt_ANTIND.ko $(KERNEL_MODULE_PATH)
	cp $(TOPDIR)/kernel_modules/anti_nat_detect/ipt_ANTIND.ko /tftpboot/tftp/
	
gpio:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/gpio modules
	cp $(TOPDIR)/kernel_modules/gpio/gpio.ko $(KERNEL_MODULE_PATH)
	
cone_udp:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/cone_udp modules
	cp $(TOPDIR)/kernel_modules/cone_udp/cone_udp.ko $(KERNEL_MODULE_PATH) 
	cp $(TOPDIR)/kernel_modules/cone_udp/cone_udp.ko /tftpboot/tftp/
	
starcraft_helper:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/starcraft_helper modules
	cp $(TOPDIR)/kernel_modules/starcraft_helper/ip_conntrack_helper_starcraft.ko /tftpboot/tftp/

harmony:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/harmony modules
	
pppol2tp:	
	if [ "$(KERNELVER)" = "2.6.15" ]; then \
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/pppol2tp-kmod-0.17 modules && \
	cp $(TOPDIR)/kernel_modules/pppol2tp-kmod-0.17/*.ko $(KERNEL_MODULE_PATH) ; \
	else \
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=drivers/net modules && \
	cp $(KERNELPATH)/drivers/net/pppol2tp.ko $(KERNEL_MODULE_PATH) ; \
	fi;
	
#~generic_ppp:	
#~ifeq ($(KERNELVER), "2.6.15")
#~	echo "Nothing to do..."
#~else
#~	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=drivers/net modules    
#~	cp $(KERNELPATH)/drivers/net/ppp_generic.ko $(KERNEL_MODULE_PATH) && \
#~	cp $(KERNELPATH)/drivers/net/ppp_generic.ko $(TOPDIR)/util/mkfirmware/bin
#~	#HouXB
#~endif

pptp_module:
	 $(MAKEARCH) -C $(KERNELPATH) SUBDIRS=$(TOPDIR)/kernel_modules/pptp_driver-0.8.3 modules
	cp $(TOPDIR)/kernel_modules/pptp_driver-0.8.3/*.ko $(KERNEL_MODULE_PATH)
#
# applications
#

apps_build: $(BOARD_APPLICATIONS)


libffmpeg: # easy build for libffmpeg 
	cd $(TOPDIR)/apps/nas/media_server/ffmpeg-0.6 && \
	if test -e config.mak; then make distclean; fi ;
	cd $(TOPDIR)/apps/nas/media_server/ffmpeg-0.6 && \
	./configure --prefix=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE) \
				--libdir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib \
				--enable-static --disable-ffmpeg --disable-ffserver \
				--disable-ffplay --disable-ffprobe \
				--disable-zlib --disable-bzlib \
				--enable-cross-compile --target-os=linux --arch=mips;
	cd $(TOPDIR)/apps/nas/media_server/ffmpeg-0.6 &&\
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib &&\
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib install

libdlna: libffmpeg # easy build for libdlna
	cd $(TOPDIR)/apps/nas/media_server/libdlna-0.2.3 && \
	if test -e config.mak; then make distclean; fi ;
	cp -rf $(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/include/* $(TOPDIR)/build/$(TOOLCHAIN)/build_mips/staging_dir/usr/include/;	
	cd $(TOPDIR)/apps/nas/media_server/libdlna-0.2.3 &&\
	export CC=$(TOOLPREFIX)gcc &&\
	export LD=$(TOOLPREFIX)ld &&\
	export AR=$(TOOLPREFIX)ar &&\
	export RANLIB=$(TOOLPREFIX)ranlib && \
	./configure --libdir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib \
				--prefix=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE) \
				--includedir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/include \
				--disable-shared --cross-compile && \
	make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib install
	
libupnp: libdlna # easy build for libupnp
	cd $(TOPDIR)/apps/nas/media_server/libupnp-1.6.6 &&\
	if test -e Makefile; then make distclean; fi &&\
	./configure --prefix=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE) \
				--libexecdir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE) \
				--libdir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib \
				--build=i386 --host=mips-linux && \
	make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib install

ushare: libupnp # easy build for ushare 
	cp -rf $(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/include/* $(TOPDIR)/build/$(TOOLCHAIN)/build_mips/staging_dir/usr/include/;	
	cd $(TOPDIR)/apps/nas/media_server/ushare-1.1a &&\
	if test -e config.mak; then make distclean; fi ;
	cd $(TOPDIR)/apps/nas/media_server/ushare-1.1a &&\
	export CC=$(TOOLPREFIX)gcc &&\
	export LD=$(TOOLPREFIX)ld &&\
	export AR=$(TOOLPREFIX)ar &&\
	export RANLIB=$(TOOLPREFIX)ranlib && \
	export PKG_CONFIG_PATH=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib/pkgconfig &&\
	./configure --prefix=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE) \
				--with-libupnp-dir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib \
				--with-libdlna-dir=$(TOPDIR)/apps/nas/media_server/$(BOARD_TYPE)/lib \
				--cross-compile --enable-dlna --disable-nls && \
	make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib install;

busybox: # easy build for busybox, no clean
	cd $(TOPDIR)/apps/$(BUSYBOX); $(MAKE) PREFIX=$(INSTALL_ROOT) uninstall; \
	find . -name "*.o" | xargs rm -rf; \
	$(MAKE); \
	$(MAKE) PREFIX=$(INSTALL_ROOT) install
	
pppoe:
	cd $(TOPDIR)/apps/ppp-2.4.3/pppd && make clean && make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp pppd $(INSTALL_ROOT)/usr/sbin

pptp:
	cd $(TOPDIR)/apps/pptp-1.7.1 && make clean && make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp pptp $(INSTALL_ROOT)/usr/sbin

l2tp:
	cd $(TOPDIR)/apps/xl2tpd-1.1.12 && make clean &&  make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp xl2tpd $(INSTALL_ROOT)/usr/sbin

bpa:
	cd $(TOPDIR)/apps/bpalogin-2.0.2 && make clean &&  make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp bpalogin $(INSTALL_ROOT)/usr/sbin
	
iptables:
ifeq ($(KERNELVER), 2.6.15)
	cd $(TOPDIR)/apps/iptables-1.3.7/ && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld DO_IPV6=0 KERNEL_DIR=$(KERNELPATH) LIBDIR=/lib && \
	cp iptables $(INSTALL_ROOT)/sbin/ && \
	cd extensions && \
	test -d $(INSTALL_ROOT)/lib/iptables/ || mkdir -p $(INSTALL_ROOT)/lib/iptables/ && \
	mv -f libipt_DNAT.so libipt_MASQUERADE.so libipt_SNAT.so libipt_TCPMSS.so libipt_TRIGGER.so \
	libipt_comment.so libipt_icmp.so libipt_iprange.so libipt_mac.so \
	libipt_multiport.so libipt_standard.so libipt_state.so libipt_string.so \
	libipt_tcp.so libipt_time.so libipt_udp.so libipt_MARK.so libipt_multiurl.so libipt_wlanwarn.so $(INSTALL_ROOT)/lib/iptables/
else ifeq ($(KERNELVER), 2.6.31)
	cd $(TOPDIR)/apps/iptables-1.4.5 && \
	if test -e Makefile ; then make distclean ; fi && \
	./configure --prefix=$(INSTALL_ROOT) \
	--build=i386 --host=mips-linux  \
	--with-ksource=$(KERNELPATH) \
	--disable-devel --disable-libipq \
	--disable-ipv6 && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar RANLIB=$(TOOLPREFIX)ranlib install && \
	./rm_ext.sh
endif
	rm -rf $(INSTALL_ROOT)/lib/libexec
	-mv $(INSTALL_ROOT)/libexec $(INSTALL_ROOT)/lib
	rm -rf $(INSTALL_ROOT)/share
	rm -rf $(INSTALL_ROOT)/man
	

tc_build:
	cd $(TOPDIR)/apps/iproute2-$(KERNELVER) && make CC=$(TOOLPREFIX)gcc && \
	cp $(TOPDIR)/apps/iproute2-$(KERNELVER)/tc/tc $(INSTALL_ROOT)/sbin
	
lltd:
	@echo ********** making WinRally ************
	cd $(TOPDIR)/apps/lltd/wr841n-linux && \
	make clean && $(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)gcc && \
	cp lld2d $(INSTALL_ROOT)/usr/bin/

net_ioctl:
	cd $(TOPDIR)/debug_tools/net_ioctl && \
	$(TOOLPREFIX)gcc  -o net_ioctl net_ioctl.c && cp net_ioctl $(INSTALL_ROOT)/usr/
	
wlanreg:
	cd $(MADWIFIPATH)/tools/hal_diag && make && cp -f reg $(INSTALL_ROOT)/usr/bin/wlanreg
	
arp:
	cd $(TOPDIR)/apps/arp && \
	$(TOOLPREFIX)gcc -o arp arp.c && cp arp $(INSTALL_ROOT)/usr/
	
ftpserver:
	cd $(TOPDIR)/apps/vsftpd-2.3.2 && \
	$(MAKE) clean && $(MAKE) vsftpd CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp vsftpd $(IMAGEPATH)

web_server:
	echo -n "BUILD_TIME = " > $(TOPDIR)/web_server/sys/buildtime.h;
	expr `date +%H` \* 3600 + `date +%M` \* 60 + `date +%S` >> $(TOPDIR)/web_server/sys/buildtime.h;
	echo -n "BUILD_DATE = " >> $(TOPDIR)/web_server/sys/buildtime.h;
	@date +%D >> $(TOPDIR)/web_server/sys/buildtime.h;
	
	if test $(clean) -eq 1; then \
	cd $(TOPDIR)/web_server && 	make clean; \
	fi;
	
	cd $(TOPDIR)/web_server && $(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld && \
	ls -l httpd && \
	cp httpd $(IMAGEPATH) && cp httpd $(TFTPPATH)
	
httpd_build:
#if can find $(BOARD_TYPE) from "ap81fus ap83fus ap91fus ap93fus"
ifneq ($(findstring $(BOARD_TYPE), ap81fus ap83fus ap91fus ap93fus), )
	make web_server clean=1 WR841NV5=1 && \
	cd $(TOPDIR)/web_server && \
	cp httpd $(TOPDIR)/images/ap93fus/ && cp httpd $(TOPDIR)/images/ap91fus/ && \
	cp httpd $(TOPDIR)/images/ap99 && cp httpd $(TOPDIR)/images/ap99tpl/ && \
	cp httpd $(TOPDIR)/images/ap83fus/ && cp httpd $(TOPDIR)/images/ap81fus/
endif

ifneq ($(findstring $(BOARD_TYPE), ap99 ap99tpl), )
	make web_server clean=1 WR841NV5=1 INCLUDE_3G=1 &&\
	cd $(TOPDIR)/web_server && cp httpd $(TOPDIR)/images/ap99/ && cp httpd $(TOPDIR)/images/ap99tpl/
endif

ifneq ($(findstring $(BOARD_TYPE), pb92 ap123 ap123-8M ap123-803), )
	make web_server clean=1 INCLUDE_3G=1 &&\
	cd $(TOPDIR)/web_server && cp httpd $(TOPDIR)/images/ap123/ && cp httpd $(TOPDIR)/images/pb92/
endif

#httpd:
#	cd $(TOPDIR)/WR841N && $(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld
	
#
# utils
#

make_flash:
	cd $(TOPDIR)/util/make_flash_src/ && make && \
	cp make_flash $(TOPDIR)/util/

md5_des:
	cd $(TOPDIR)/util/md5_des_src/ && gcc main.c md5_des.c -o md5_des && \
	cp md5_des $(TOPDIR)/util/
		
#
# fs and image
#
strip_bin:
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/bin/*
ifneq ($(findstring $(BOARD_TYPE), pb92), )
	mv $(INSTALL_ROOT_FOR_BOARD)/sbin/wpa_supplicant $(INSTALL_ROOT_FOR_BOARD)
	-$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/sbin/*
	mv $(INSTALL_ROOT_FOR_BOARD)/wpa_supplicant $(INSTALL_ROOT_FOR_BOARD)/sbin/wpa_supplicant
else
	-$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/sbin/*
endif
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/usr/bin/*
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/usr/sbin/*
ifneq ($(filter $(BOARD_TYPE), ap123 ap123-8M ap123-803), )
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/lib/*0.9.30.so
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/lib/libgcc_s.so.1
endif

strip_kmod:
	find $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER) -name "*.ko" -type f | xargs \
	$(OLDSTRIP) \
			--strip-unneeded \
			--remove-section=.mdebug.abi32 \
			--remove-section=.comment \
			--remove-section=__ksymtab_unused \
			--remove-section=__kcrctab_unused \
			--remove-section=__ksymtab_unused_gpl \
			--remove-section=__kcrctab_unused_gpl
	
#~strip_all:
#~	cd $(INSTALL_ROOT_FOR_BOARD)/bin/ && \
#~	ls -l busybox && $(TOOLPREFIX)strip busybox && ls -l busybox 
#~	
#~	cd $(INSTALL_ROOT_FOR_BOARD)/sbin/ && \
#~	ls -l hostapd && $(TOOLPREFIX)strip hostapd && ls -l hostapd && \
#~	ls -l tc && $(TOOLPREFIX)strip tc && ls -l tc && \
#~	ls -l iptables && $(TOOLPREFIX)strip iptables && ls -l iptables && \
#~	ls -l wlanconfig && $(TOOLPREFIX)strip wlanconfig && ls -l wlanconfig && \
#~	ls -l iwlist && $(TOOLPREFIX)strip iwlist && ls -l iwlist && \
#~	ls -l wpatalk && $(TOOLPREFIX)strip wpatalk && ls -l wpatalk && \
#~	ls -l iwconfig && $(TOOLPREFIX)strip iwconfig && ls -l iwconfig && \
#~	ls -l apstart && $(TOOLPREFIX)strip apstart && ls -l apstart && \
#~	ls -l iwevent && $(TOOLPREFIX)strip iwevent && ls -l iwevent && \
#~	ls -l iwpriv && $(TOOLPREFIX)strip iwpriv && ls -l iwpriv && \
#~	ls -l iwgetid && $(TOOLPREFIX)strip iwgetid && ls -l iwgetid && \
#~	ls -l iwspy && $(TOOLPREFIX)strip iwspy && ls -l iwspy 
#~	
#~	cd $(INSTALL_ROOT)/usr/bin/ && \
#~	ls -l httpd && $(TOOLPREFIX)strip httpd && ls -l httpd && \
#~	ls -l lld2d && $(TOOLPREFIX)strip lld2d && ls -l lld2d
	
#~	cd $(INSTALL_ROOT)/usr/sbin/ && \
#~	ls -l bpalogin && $(TOOLPREFIX)strip bpalogin && ls -l bpalogin && \
#~	ls -l pppd && $(TOOLPREFIX)strip pppd && ls -l pppd && \
#~	ls -l pptp && $(TOOLPREFIX)strip pptp && ls -l pptp && \
#~	ls -l xl2tpd && $(TOOLPREFIX)strip xl2tpd && ls -l xl2tpd
	
#~ifndef lzma
#~encode = -nolzma
#~endif

apxxfus_fs_prep:
	rm -Rf $(INSTALL_ROOT_FOR_BOARD)
	cp -Rf $(INSTALL_ROOT) $(INSTALL_ROOT_FOR_BOARD)
	@echo CONFIG_3G_TOOLS=$(CONFIG_3G_TOOLS)
	make kernel_modules_install	
	make wireless_install
ifneq ($(filter $(BOARD_TYPE), pb92 ap123-8M), )
	make nas_package
	cp $(IMAGEPATH)/vsftpd $(INSTALL_ROOT_FOR_BOARD)/usr/sbin/
	$(TOPDIR)/apps/nas/media_server/media_server_install.sh $(BOARD_TYPE) $(INSTALL_ROOT_FOR_BOARD)
endif
ifeq ($(CONFIG_3G_TOOLS),y)
	make 3g_tools_install
endif
	cp $(IMAGEPATH)/httpd $(INSTALL_ROOT_FOR_BOARD)/usr/bin/
ifeq ($(CONFIG_SNMP),y)
	cp $(IMAGEPATH)/snmpd $(INSTALL_ROOT_FOR_BOARD)/usr/sbin/
endif
	make strip_bin
	
	@mv $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/kernel/$(enet)_mod.ko $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/net/
	@-mv $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/kernel/$(enet)_mod_r.ko $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/net/ >/dev/null 2>&1
	@-mv $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/kernel/$(enet)_readid.ko $(INSTALL_ROOT_FOR_BOARD)/lib/modules/$(KERNELVER)/net/ >/dev/null 2>&1
	@cd $(WEBDIR)/utils/web_collector/ && ./webc -s $(WEBDIR)/$(lang)/common:$(WEBDIR)/$(lang)/oem/$(oem_dir) -d $(INSTALL_ROOT_FOR_BOARD)/web >/dev/null 2>&1
	@test -d $(INSTALL_ROOT_FOR_BOARD)/web/oem/ || mkdir -p $(INSTALL_ROOT_FOR_BOARD)/web/oem/
	@cp $(TOPDIR)/webs/models/$(conf_file) $(TOPDIR)/webs/models/model_tmp.conf

fill_model_conf:
	@echo "" >> $(TOPDIR)/webs/models/model_tmp.conf
	@cat $(TOPDIR)/webs/models/version.conf >> $(TOPDIR)/webs/models/model_tmp.conf
	@echo "" >> $(TOPDIR)/webs/models/model_tmp.conf
	@cat $(TOPDIR)/web_server/sys/buildtime.h >> $(TOPDIR)/webs/models/model_tmp.conf
	@$(TOPDIR)/util/md5_des $(TOPDIR)/webs/models/model_tmp.conf $(INSTALL_ROOT_FOR_BOARD)/web/oem/model.conf enc
	
mk_squashfs_build:
	make fill_model_conf
	@find $(INSTALL_ROOT_FOR_BOARD) -name ".svn" | xargs rm -rf
	@$(TOPDIR)/util/fakeroot/install/bin/fakeroot $(TOPDIR)/util/buildFS_LZ
	@ls -l $(IMAGEPATH)/$(BOARD_TYPE)-squashfs
	@$(TOPDIR)/util/pad -i $(IMAGEPATH)/$(BOARD_TYPE)-squashfs -p $(fs_size)
	@cp $(IMAGEPATH)/$(BOARD_TYPE)-squashfs $(TFTPPATH)
	
#~squashfs_build:
#~	$(TOPDIR)/util/mksquashfs $(INSTALL_ROOT) $(IMAGEPATH)/$(BOARD_TYPE)-squashfs -noappend -be -always-use-fragments $(encode)
#~	ls -l $(IMAGEPATH)/$(BOARD_TYPE)-squashfs
#~	$(TOPDIR)/util/pad -i $(IMAGEPATH)/$(BOARD_TYPE)-squashfs -p 0x2C0000
#~	cp $(IMAGEPATH)/$(BOARD_TYPE)-squashfs $(TFTPPATH)
#~	cp $(IMAGEPATH)/$(BOARD_TYPE)-squashfs $(TOPDIR)/make_flash
#~	ls -l $(TFTPPATH)/$(BOARD_TYPE)-squashfs
	
image_build:
	cd $(IMAGEPATH)/ && \
	$(TOPDIR)/util/make_flash -b u-boot.bin -k vmlinux.bin.gz -f $(BOARD_TYPE)-squashfs \
	-o $(prefix) -c $(TOPDIR)/webs/models/model_tmp.conf -t $(TOPDIR)/web_server/sys/buildtime.h && \
	chmod 777 *

linux_build_all: uboot kernel_build kernel_modules wireless apps_build
