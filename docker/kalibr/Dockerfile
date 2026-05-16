# Adapted from Kalibr's upstream Dockerfile_ros1_20_04
# (ethz-asl/kalibr @ commit 1f60227442d25e36365ef5f72cd80b9666d73467,
#  https://github.com/ethz-asl/kalibr/blob/master/Dockerfile_ros1_20_04).
#
# `git clone` instead of upstream's `ADD .` keeps the build context to this
# directory rather than requiring the full Kalibr source tree alongside.
#
# Force linux/amd64: Kalibr depends on amd64-only debs from the ROS Noetic
# archive. On Apple Silicon, Colima emulates amd64 via qemu; the build is
# slow once, then cached. Pass `colima start --arch x86_64 …` for a native
# x86 VM if you need faster builds.

FROM --platform=linux/amd64 osrf/ros:noetic-desktop-full

ARG KALIBR_GIT_REPO=https://github.com/ethz-asl/kalibr.git
ARG KALIBR_GIT_REF=1f60227442d25e36365ef5f72cd80b9666d73467
# `catkin build -j$(nproc)` runs many cc1plus instances in parallel; numpy_eigen
# in particular generates lots of heavy Eigen template instantiations and
# OOM-segfaults under amd64 emulation when memory is tight. Capping the job
# count keeps peak memory low at the cost of a slightly longer build. Override
# with --build-arg KALIBR_BUILD_JOBS=<N> if your VM has plenty of RAM.
ARG KALIBR_BUILD_JOBS=2

# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon


# Create the workspace and build kalibr in it
ENV WORKSPACE=/catkin_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN cd $WORKSPACE/src && \
	git clone "$KALIBR_GIT_REPO" kalibr && \
	cd kalibr && \
	git checkout "$KALIBR_GIT_REF"

RUN	cd $WORKSPACE &&\
	catkin build -j${KALIBR_BUILD_JOBS}


# When a user runs a command we run this entrypoint script first, which
# sources the catkin workspace's setup.bash so `rosrun kalibr ...` resolves.
# The script lives in the build context (next to this Dockerfile).
# Adapted from the spirit of https://github.com/ethz-asl/kalibr/pull/346
COPY entrypoint.sh /usr/local/bin/gw-kalibr-entrypoint
RUN chmod +x /usr/local/bin/gw-kalibr-entrypoint
ENTRYPOINT ["/usr/local/bin/gw-kalibr-entrypoint"]
CMD ["/bin/bash"]
