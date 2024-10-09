#!/bin/bash

source dkms.conf

DEB_NAME=$(echo $PACKAGE_NAME | sed 's,_,-,g')

MKDEB_DIR=${PACKAGE_NAME}-dkms-mkdeb
BUILD_DIR=${DEB_NAME}-dkms_${PACKAGE_VERSION}_all

mkdir -p $BUILD_DIR/DEBIAN
sed -e "s/DEBIAN_PACKAGE/${DEB_NAME}/g" -e '/^$/d' $MKDEB_DIR/debian/control > $BUILD_DIR/DEBIAN/control
echo "Version: ${PACKAGE_VERSION}" >> $BUILD_DIR/DEBIAN/control
sed -e "s/DEBIAN_PACKAGE/${PACKAGE_NAME}/g" -e "s/MODULE_VERSION/${PACKAGE_VERSION}/g" -e "s/DATE_STAMP/$(date)/g" $MKDEB_DIR/debian/changelog > $BUILD_DIR/DEBIAN/changelog
sed -e "s/NAME=MODULE_NAME/NAME=${PACKAGE_NAME}/g" $MKDEB_DIR/debian/postinst > $BUILD_DIR/DEBIAN/postinst
sed -e "s/NAME=MODULE_NAME/NAME=${PACKAGE_NAME}/g" -e "s/VERSION=MODULE_VERSION/VERSION=${PACKAGE_VERSION}/g" $MKDEB_DIR/debian/prerm > $BUILD_DIR/DEBIAN/prerm
chmod 775 $BUILD_DIR/DEBIAN/postinst $BUILD_DIR/DEBIAN/prerm

mkdir -p $BUILD_DIR/usr/share/doc/$PACKAGE_NAME
cp README $BUILD_DIR/usr/share/doc/$PACKAGE_NAME
gzip $BUILD_DIR/usr/share/doc/$PACKAGE_NAME/README

mkdir -p $BUILD_DIR/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}
cp -t $BUILD_DIR/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION} dkms.conf ${PACKAGE_NAME}.c Makefile

dpkg-deb --build $BUILD_DIR