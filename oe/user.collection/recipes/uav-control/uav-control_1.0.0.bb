DESCRIPTION = "Control software for UAV senior design project"
HOMEPAGE = "http://www.ce.rit.edu"
LICENSE = "GPL"
DEPENDS = ""

PR = "r0"

INITSCRIPT_NAME = "uav"
INITSCRIPT_PARAMS = "defaults 40"

SRC_URI = " \
    file://uav.init \
    file://uav_control.c \
    file://Makefile \
"

S = "${WORKDIR}"

inherit update-rc.d

do_install () {
    install -d ${D}${sbindir}/
    install -m 0755 ${S}/uav_control ${D}${sbindir}/

    install -d ${D}${sysconfdir}/init.d/
    install -m 0755 ${S}/uav.init ${D}${sysconfdir}/init.d/uav
}

FILES_${PN} = "${sbindir}/"
FILES_${PN} += "${sysconfdir}/"

