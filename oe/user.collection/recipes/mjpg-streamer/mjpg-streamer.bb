DESCRIPTION = "Streaming HTTP server for UVC webcam feeds"
HOMEPAGE = "http://mjpg-streamer.sourceforge.net/"
DEPENDS = "jpeg"

PV="svn${SRCDATE}"
PR = "r0"

SRC_URI = " \
    svn://mjpg-streamer.svn.sourceforge.net/svnroot/mjpg-streamer;module=mjpg-streamer;proto=https \
    file://makefile.patch;apply=yes \
"

S = "${WORKDIR}/mjpg-streamer"
SRCREV = "HEAD"

do_install () {
    echo "WORKDIR = ${WORKDIR}"
    echo "SOURCEDIR = ${S}"
    install -d ${D}${bindir}/
    install -m 0755 ${S}/mjpg_streamer ${D}${bindir}/
    install -m 0755 ${S}/input_uvc.so ${D}${bindir}/
    install -m 0755 ${S}/input_testpicture.so ${D}${bindir}/
    install -m 0755 ${S}/output_http.so ${D}${bindir}/
}

FILES_${PN} = " \
    ${bindir}/mjpg_streamer \
    ${bindir}/input_uvc.so \
    ${bindir}/input_testpicture.so \
    ${bindir}/output_http.so \
    "

