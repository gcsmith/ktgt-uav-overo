DESCRIPTION = "Linux Driver for manipulating GPIO pins"
HOMEPAGE = "http://www.davehylands.com"
SECTION = "base"
LICENSE = "GPL"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PV="svn${SRCDATE}"
PR = "r1"

INITSCRIPT_NAME = "gpio"
INITSCRIPT_PARAMS = "defaults 40"

SRC_URI = " \
    svn://ktgt-uav-overo.googlecode.com/svn/trunk;module=gpio-module;proto=http \
"

S = "${WORKDIR}/gpio-module"
SRCREV = "HEAD"

inherit module update-rc.d

do_stage () {
    install -m 0644 ${S}/user-gpio-drv.h ${STAGING_INCDIR}/
}

do_install() {  
    install -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/
    install -m 0644 ${S}/user-gpio-drv.ko ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/

    install -d ${D}${sysconfdir}/init.d/
    install -m 0755 ${S}/gpio.init ${D}${sysconfdir}/init.d/gpio
}

PACKAGES = "${PN}"

FILES_${PN} = "${base_libdir}/modules/"
FILES_${PN} += "${sysconfdir}/"

