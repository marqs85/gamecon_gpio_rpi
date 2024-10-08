#!/bin/bash

source dkms.conf

DEB_NAME=$(echo $PACKAGE_NAME | sed 's,_,-,g')

MKDEB_DIR=${PACKAGE_NAME}-dkms-mkdeb
BUILD_DIR=${DEB_NAME}_${PACKAGE_VERSION}

mkdir -p $BUILD_DIR/DEBIAN
sed -e "s/DEBIAN_PACKAGE/${DEB_NAME}/g" -e '/^$/d' $MKDEB_DIR/debian/control > $BUILD_DIR/DEBIAN/control
echo "Version: ${PACKAGE_VERSION}" >> $BUILD_DIR/DEBIAN/control
sed -e "s/NAME=MODULE_NAME/NAME=${DEB_NAME}/g" $MKDEB_DIR/debian/postinst > $BUILD_DIR/DEBIAN/postinst
sed -e "s/NAME=MODULE_NAME/NAME=${DEB_NAME}/g" -e "s/VERSION=MODULE_VERSION/VERSION=${PACKAGE_VERSION}/g" $MKDEB_DIR/debian/prerm > $BUILD_DIR/DEBIAN/prerm
chmod 775 $BUILD_DIR/DEBIAN/postinst $BUILD_DIR/DEBIAN/prerm

mkdir -p $BUILD_DIR/usr/share/doc
cp README $BUILD_DIR/usr/share/doc

mkdir -p $BUILD_DIR/usr/src/${DEB_NAME}-${PACKAGE_VERSION}
cp -t $BUILD_DIR/usr/src/${DEB_NAME}-${PACKAGE_VERSION} dkms.conf ${PACKAGE_NAME}.c Makefile

dpkg-deb --build $BUILD_DIR