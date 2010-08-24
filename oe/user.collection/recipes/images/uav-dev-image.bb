require recipes/images/uav-rel-image.bb

export IMAGE_BASENAME = "uav-dev-image"

DEV_INSTALL += " \
    automake \
    binutils \
    binutils-symlinks \
    cpp \
    cpp-symlinks \
    diffstat \
    diffutils \
    gcc \
    gcc-symlinks \
    git \
    libtool \
    libtool-dev \
    make \
    patch \
    patchutils \
    subversion \
	virtual-libc-dev \
#   task-native-sdk \
#   artoolkit-dev \
#   libdc1394 \
#   pkgconfig \
#   gsl-dev \
#   ncurses-dev \
#   boost-dev \
    "

DSP_EXTRA_INSTALL += " \
    ti-dsplink-examples \
    ti-codec-engine \
    "

GLES_INSTALL += " \
    libgles-omap3 \
    libgles-omap3-dev \
    "

TOOLS_EXTRA_INSTALL += " \
    bash \
    bzip2 \
    openssh-misc \
    openssh-scp \
    screen \
    sed \
    strace \
    vim-syntax \
    "

IMAGE_INSTALL += " \
    ${DEV_INSTALL} \
    ${DSP_EXTRA_INSTALL} \
    ${GLES_INSTALL} \
    ${TOOLS_EXTRA_INSTALL} \
    "

