DESCRIPTION = "User Lib for manipulating GPIO pins"
HOMEPAGE = "http://www.davehylands.com"
SECTION = "base"
LICENSE = "GPL"
DEPENDS = "gpio-module"

PV="svn${SRCDATE}"
PR = "r0"

SRC_URI = "svn://ktgt-uav-overo.googlecode.com/svn/trunk;module=gpio-lib;proto=http"

S = "${WORKDIR}/gpio-lib"
SRCREV = "HEAD"

TARGET_CC_ARCH += "${LDFLAGS}"

do_stage () {
    oe_libinstall -a libgpio ${STAGING_LIBDIR}
    install -m 0644 ${S}/user-gpio.h ${STAGING_INCDIR}/
}

do_install() {  
    install -d ${D}${libdir}/
    install -m 0755 ${S}/libgpio.a ${D}${libdir}/
}

PACKAGES = "${PN}"

FILES_${PN} = "${libdir}/libgpio.a"

