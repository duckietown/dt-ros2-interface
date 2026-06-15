# parameters
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG PROJECT_ICON="cube"
ARG PROJECT_FORMAT_VERSION

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION=duckietown
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/${BASE_ORGANIZATION}/${BASE_REPOSITORY}:${BASE_TAG} AS base

# recall all arguments
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
ARG PROJECT_ICON
ARG PROJECT_FORMAT_VERSION
ARG BASE_TAG
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT
# - ROS2
ARG ROS2_DISTRO

# ROS2 info
ENV ROS2_DISTRO="${ROS2_DISTRO}" \
    COLCON_WS="/code"

# check build arguments
RUN dt-args-check \
    "PROJECT_NAME" "${PROJECT_NAME}" \
    "PROJECT_DESCRIPTION" "${PROJECT_DESCRIPTION}" \
    "PROJECT_MAINTAINER" "${PROJECT_MAINTAINER}" \
    "PROJECT_ICON" "${PROJECT_ICON}" \
    "PROJECT_FORMAT_VERSION" "${PROJECT_FORMAT_VERSION}" \
    "ARCH" "${ARCH}" \
    "DISTRO" "${DISTRO}" \
    "DOCKER_REGISTRY" "${DOCKER_REGISTRY}" \
    "BASE_REPOSITORY" "${BASE_REPOSITORY}" \
    && dt-check-project-format "${PROJECT_FORMAT_VERSION}"

# define/create repository path
ARG PROJECT_PATH="${SOURCE_DIR}/${PROJECT_NAME}"
ARG PROJECT_LAUNCHERS_PATH="${LAUNCHERS_DIR}/${PROJECT_NAME}"
RUN mkdir -p "${PROJECT_PATH}" "${PROJECT_LAUNCHERS_PATH}"
WORKDIR "${PROJECT_PATH}"

# keep some arguments as environment variables
ENV DT_PROJECT_NAME="${PROJECT_NAME}" \
    DT_PROJECT_DESCRIPTION="${PROJECT_DESCRIPTION}" \
    DT_PROJECT_MAINTAINER="${PROJECT_MAINTAINER}" \
    DT_PROJECT_ICON="${PROJECT_ICON}" \
    DT_PROJECT_PATH="${PROJECT_PATH}" \
    DT_PROJECT_LAUNCHERS_PATH="${PROJECT_LAUNCHERS_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# copy binaries
#COPY ./assets/bin/. /usr/local/bin/

# setup ROS2 sources
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# remove catkin from base image (installed by ament)
# TODO: this might not be needed anymore
RUN apt-get remove -y \
    python3-catkin-pkg

# install apt dependencies
COPY ./dependencies-apt.txt "${PROJECT_PATH}/"
RUN dt-apt-install ${PROJECT_PATH}/dependencies-apt.txt

# Upgrade all ros-jazzy-* packages to a consistent snapshot. The dt-ros2-commons
# base image was built months ago; when we install additional ros-jazzy-*
# packages above, apt may pull in fresh dependencies (e.g. std_msgs 5.3.7 from
# 2026-04-12) that are ABI-incompatible with the older fastcdr/fastrtps
# typesupport libs already installed in the base. Symptom: publishing any
# std_msgs topic hits "symbol lookup error ... libfastcdr ... serializeEPc".
# Upgrading everything from the ros repo in one apt transaction resolves it.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

# Build libcamera from the Raspberry Pi fork so we have the OV5647 fixes that
# landed after 0.2. Ubuntu Noble ships libcamera 0.2.0, which trips a
# `prepareIsp()` IPA-buffer assertion on the Duckiedrone's OV5647 sensor.
ARG LIBCAMERA_REF=v0.4.0
RUN set -eux; \
    git clone --depth 1 --branch "${LIBCAMERA_REF}" \
        https://github.com/raspberrypi/libcamera.git /tmp/libcamera; \
    cd /tmp/libcamera; \
    meson setup build \
        --prefix=/usr \
        --buildtype=release \
        -Dipas=rpi/vc4 \
        -Dpipelines=rpi/vc4 \
        -Dpycamera=enabled \
        -Ddocumentation=disabled \
        -Dgstreamer=disabled \
        -Dv4l2=true \
        -Dtest=false \
        -Dcam=disabled \
        -Dqcam=disabled \
        -Dlc-compliance=disabled ; \
    ninja -C build install; \
    ldconfig; \
    cd /; rm -rf /tmp/libcamera

# libcamera's meson install drops the Python bindings under
# /usr/lib/python<ver>/site-packages/, which Ubuntu Noble's Python doesn't
# auto-include. Expose it via a .pth file so `import libcamera` works.
RUN set -e; \
    pyver="$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; \
    lc_parent="$(python3 -c "import glob,os;hits=glob.glob('/usr/lib/python*/site-packages/libcamera')+glob.glob('/usr/lib/python*/dist-packages/libcamera')+glob.glob('/usr/lib/*/python*/site-packages/libcamera');print(os.path.dirname(hits[0]) if hits else '')")"; \
    if [ -z "${lc_parent}" ]; then \
        echo "ERROR: libcamera Python bindings not found after install" >&2; exit 1; \
    fi; \
    mkdir -p "/usr/local/lib/python${pyver}/dist-packages"; \
    echo "${lc_parent}" > "/usr/local/lib/python${pyver}/dist-packages/libcamera.pth"; \
    echo "libcamera Python bindings: ${lc_parent}"

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${PROJECT_PATH}/"
RUN dt-pip3-install "${PROJECT_PATH}/dependencies-py3.*"

# picamera2.previews/__init__.py eagerly imports drm_preview, which needs
# pykms (python3-kms++). That package is not available on Noble and we run
# headless, so soften the import to tolerate ImportError.
RUN set -e; \
    pyver="$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; \
    previews_init="/usr/local/lib/python${pyver}/dist-packages/picamera2/previews/__init__.py"; \
    if [ -f "${previews_init}" ] && ! grep -q "except ImportError" "${previews_init}"; then \
        sed -i 's|^from .drm_preview import DrmPreview$|try:\n    from .drm_preview import DrmPreview\nexcept ImportError:\n    DrmPreview = None|' "${previews_init}"; \
    fi

# copy the source code
COPY ./packages "${PROJECT_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
  dt-colcon-build ${WORKSPACE_DIR}

# install launcher scripts
COPY ./launchers/. "${PROJECT_LAUNCHERS_PATH}/"
RUN dt-install-launchers "${PROJECT_LAUNCHERS_PATH}"

# install scripts
COPY ./assets/entrypoint.d "${PROJECT_PATH}/assets/entrypoint.d"
COPY ./assets/environment.d "${PROJECT_PATH}/assets/environment.d"

# mavros plugin configuration (loaded by launchers/mavros.sh)
COPY ./assets/mavros "${PROJECT_PATH}/assets/mavros"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL \
    # module info
    org.duckietown.label.project.name="${PROJECT_NAME}" \
    org.duckietown.label.project.description="${PROJECT_DESCRIPTION}" \
    org.duckietown.label.project.maintainer="${PROJECT_MAINTAINER}" \
    org.duckietown.label.project.icon="${PROJECT_ICON}" \
    org.duckietown.label.project.path="${PROJECT_PATH}" \
    org.duckietown.label.project.launchers.path="${PROJECT_LAUNCHERS_PATH}" \
    # format
    org.duckietown.label.format.version="${PROJECT_FORMAT_VERSION}" \
    # platform info
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    # code info
    org.duckietown.label.code.distro="${DISTRO}" \
    org.duckietown.label.code.launcher="${LAUNCHER}" \
    org.duckietown.label.code.python.registry="${PIP_INDEX_URL}" \
    # base info
    org.duckietown.label.base.organization="${BASE_ORGANIZATION}" \
    org.duckietown.label.base.repository="${BASE_REPOSITORY}" \
    org.duckietown.label.base.tag="${BASE_TAG}"
# <== Do not change the code above this line
# <================================================== \

# install mavros geographiclib datasets
RUN wget -O - https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh | bash