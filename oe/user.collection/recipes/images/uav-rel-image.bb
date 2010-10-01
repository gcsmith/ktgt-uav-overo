inherit image

DEPENDS = "task-uav"

export IMAGE_BASENAME = "uav-rel-image"

BASE_INSTALL += " \
    task-uav-extended \
    uav-control \
    pwm-module \
    gpio-module \
    gpio-event-module \
    "

FIRMWARE_INSTALL = " \
    libertas-sd-firmware \
    rt73-firmware \
    zd1211-firmware \
    "

VIDEO_INSTALL += " \
    libv4l \
    mjpg-streamer \
    jpeg \
    "

DSP_INSTALL += " \
    ti-cmem-module \
    ti-dsplink-module \
    ti-lpm-module \
    "

TOOLS_INSTALL = " \
    ckermit \
    devmem2 \
    dhcp-client \
    dosfstools \
    grep \
    fbgrab \
    fbset \
    fbset-modes \
    i2c-tools \
    ksymoops \
    mkfs-jffs2 \
    mtd-utils \
    ntp ntpdate \
    openssh-ssh \
    omap3-writeprom \
    procps \
    socat \
    syslog-ng \
    task-proper-tools \
    vim \
    "

IMAGE_INSTALL += " \
    ${BASE_INSTALL} \
    ${FIRMWARE_INSTALL} \
    ${VIDEO_INSTALL} \
    ${DSP_INSTALL} \
    ${TOOLS_INSTALL} \
    "

