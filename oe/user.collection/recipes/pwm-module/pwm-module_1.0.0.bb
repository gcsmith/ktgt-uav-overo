DESCRIPTION = "PWM kernel module for UAV senior design project"
HOMEPAGE = "http://www.ce.rit.edu"
SECTION = "base"
PRIORITY = "optional"
LICENSE = "GPL"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PR = "r0"

INITSCRIPT_NAME = "pwm"
INITSCRIPT_PARAMS = "defaults 40"

SRC_URI = " \
    file://files/pwm.init \
    file://files/pwm_module.c \
    file://files/pwm_module.h \
    file://files/pwm_interface.h \
    file://files/Makefile \
"

S = "${WORKDIR}/files"

inherit module update-rc.d

do_compile () {
    unset CFLAGS CPPFLAGS CXXFLAGS LDFLAGS CC LD CPP
    oe_runmake 'MODPATH="${D}${base_libdir}/modules/${KERNEL_VERSION}/kernel/drivers/ecu" ' \
        'KERNEL_SOURCE="${STAGING_KERNEL_DIR}" ' \
        'KDIR="${STAGING_KERNEL_DIR}"' \
        'KERNEL_VERSION="${KERNEL_VERSION}" ' \
        'CC="${KERNEL_CC}" ' \
        'LD="${KERNEL_LD}" '
}

do_stage () {
    install -m 0644 ${S}/pwm_interface.h ${STAGING_INCDIR}/
}

do_install () {
    install -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/pwm_module*${KERNEL_OBJECT_SUFFIX} ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra

    install -d ${D}${sysconfdir}/init.d/
    install -m 0755 ${S}/pwm.init ${D}${sysconfdir}/init.d/pwm
}

PACKAGES = "${PN}"

FILES_${PN} = "${base_libdir}/modules/"
FILES_${PN} += "${sysconfdir}/"

