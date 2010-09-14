DESCRIPTION = "Linux Driver for Detecting GPIO events"
SECTION = "base"
PRIORITY = "optional"
LICENSE = "GPL"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PV="svn${SRCDATE}"
PR = "r0"

INITSCRIPT_NAME = "gpio-event"
INITSCRIPT_PARAMS = "defaults 40"

SRC_URI = " \
    svn://ktgt-uav-overo.googlecode.com/svn/trunk;module=gpio-event-module;proto=http \
"

S = "${WORKDIR}/module"
SRCREV = "HEAD"

inherit module update-rc.d

do_stage () {
    install -m 0644 ${S}/gpio-event-drv.h ${STAGING_INCDIR}/
}

do_install() {  
    install -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/
        install -m 0644 ${S}/gpio-event-drv.ko ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/

        install -d ${D}${sysconfdir}/init.d/
        install -m 0755 ${S}/gpio-event.init ${D}${sysconfdir}/init.d/gpio-event
}

PACKAGES = "${PN}"

FILES_${PN} = "${base_libdir}/modules/"
FILES_${PN} += "${sysconfdir}/"


