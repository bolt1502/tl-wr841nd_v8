#!/bin/sh

if [ ! -n "$1" ]; then
	echo "insufficient arguments!"
	echo "Usage: $0 <rootfs.build dir>"
	echo "Example: $0 /home/root/WR1043/rootfs.build"
	exit 0
fi

TOP_DIR=`pwd`
ROOTFS_DIR="$1"

#link /etc/passwd
echo " link /etc/passwd "
rm -f ${ROOTFS_DIR}/etc/passwd
cd ${ROOTFS_DIR}/etc
ln -s /tmp/passwd passwd

#put fuse
echo " put kernel modules "
mkdir ${ROOTFS_DIR}/lib/modules/${KERNELVER}/nas
rm -rf ${ROOTFS_DIR}/lib/modules/${KERNELVER}/nas/*
cp -f ${TOP_DIR}/module/* ${ROOTFS_DIR}/lib/modules/${KERNELVER}/nas/

#put usbp
echo " put usbp"
rm -f ${ROOTFS_DIR}/sbin/usbp
cp -f ${TOP_DIR}/usbp/usbp ${ROOTFS_DIR}/sbin/usbp
chmod 755 ${ROOTFS_DIR}/sbin/usbp

#put ntfs-3g
echo " put ntfs-3g"
rm -f ${ROOTFS_DIR}/bin/ntfs-3g
cp -f ${TOP_DIR}/ntfs-3g/ntfs-3g ${ROOTFS_DIR}/bin/ntfs-3g
chmod 755 ${ROOTFS_DIR}/bin/ntfs-3g

rm -f ${ROOTFS_DIR}/lib/libntfs-3g.so
rm -f ${ROOTFS_DIR}/lib/libntfs-3g.so.38
rm -f ${ROOTFS_DIR}/lib/libntfs-3g.so.38.0.0
cd ${ROOTFS_DIR}/lib/
ln -s libntfs-3g.so.38.0.0 libntfs-3g.so.38
ln -s libntfs-3g.so.38.0.0 libntfs-3g.so
cp -f ${TOP_DIR}/ntfs-3g/libntfs-3g.so.38.0.0 ${ROOTFS_DIR}/lib/libntfs-3g.so.38.0.0

cd ${TOP_DIR}

#put samba
echo "put samba"

rm -f ${ROOTFS_DIR}/lib/libbigballofmud.so
rm -f ${ROOTFS_DIR}/lib/libbigballofmud.so.0
cp -f ${TOP_DIR}/samba/libbigballofmud.so ${ROOTFS_DIR}/lib/libbigballofmud.so
cd ${ROOTFS_DIR}/lib/
ln -s libbigballofmud.so libbigballofmud.so.0

mkdir -p ${ROOTFS_DIR}/etc/samba
rm -rf ${ROOTFS_DIR}/etc/samba/*
cp -rf ${TOP_DIR}/samba/lib ${ROOTFS_DIR}/etc/samba/lib
#cp -rf ${TOP_DIR}/samba/shscript ${ROOTFS_DIR}/etc/samba/shscript
#chmod 755 ${ROOTFS_DIR}/etc/samba/shscript/*
cp -rf ${TOP_DIR}/samba/smbpasswd ${ROOTFS_DIR}/usr/bin/smbpasswd
chmod 755 ${ROOTFS_DIR}/usr/bin/smbpasswd
cp -rf ${TOP_DIR}/samba/smbd ${ROOTFS_DIR}/usr/sbin/smbd
chmod 755 ${ROOTFS_DIR}/usr/sbin/smbd

#put hotplug shell script
cp -f ${TOP_DIR}/hotplug ${ROOTFS_DIR}/sbin/hotplug
chmod 755 ${ROOTFS_DIR}/sbin/hotplug

#delete nas shell cmd in rcS
#don't execute  /etc/samba/shscript/sysuser.sh now
PASSWDSTR="/etc/samba/shscript/sysuser.sh"
grep -q ${PASSWDSTR} ${ROOTFS_DIR}/etc/rc.d/rcS
if [ $? -eq 0 ] ; then
grep -v ${PASSWDSTR} ${ROOTFS_DIR}/etc/rc.d/rcS > ${ROOTFS_DIR}/etc/rc.d/rcS.tmp
mv -f ${ROOTFS_DIR}/etc/rc.d/rcS.tmp ${ROOTFS_DIR}/etc/rc.d/rcS
chmod +x ${ROOTFS_DIR}/etc/rc.d/rcS
fi
