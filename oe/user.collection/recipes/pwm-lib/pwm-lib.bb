DESCRIPTION = "User Lib for manipulating PWM pins"
HOMEPAGE = "http://www.davehylands.com"
SECTION = "base"
LICENSE = "GPL"
DEPENDS = "pwm-module"

PV="svn${SRCDATE}"
PR = "r0"

SRC_URI = "svn://ktgt-uav-overo.googlecode.com/svn/trunk;module=pwm-lib;proto=http"

S = "${WORKDIR}/pwm-lib"
SRCREV = "HEAD"

TARGET_CC_ARCH += "${LDFLAGS}"

do_stage () {
    oe_libinstall -a libpwm ${STAGING_LIBDIR}
    install -m 0644 ${S}/pwm_lib.h ${STAGING_INCDIR}/
}

do_install() {  
    install -d ${D}${libdir}/
    install -m 0755 ${S}/libpwm.a ${D}${libdir}/
}

PACKAGES = "${PN}"

FILES_${PN} = "${libdir}/libpwm.a"

