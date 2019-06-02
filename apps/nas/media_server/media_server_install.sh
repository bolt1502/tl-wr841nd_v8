#!/bin/bash

SRC_PATH=${TOPDIR}/apps/nas/media_server/$1

cp $SRC_PATH/bin/ushare $2/usr/bin
cp $SRC_PATH/lib/libixml.so.2.0.4 $2/lib
cp $SRC_PATH/lib/libupnp.so.3.0.5 $2/lib
cp $SRC_PATH/lib/libthreadutil.so.2.2.3 $2/lib
cp $SRC_PATH/etc/ushare.conf $2/etc

cd $2/lib

ln -s libixml.so.2.0.4 libixml.so
ln -s libixml.so.2.0.4 libixml.so.2

ln -s libupnp.so.3.0.5 libupnp.so
ln -s libupnp.so.3.0.5 libupnp.so.3

ln -s libthreadutil.so.2.2.3 libthreadutil.so
ln -s libthreadutil.so.2.2.3 libthreadutil.so.2
