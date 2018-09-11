FROM afdaniele/libbot2:latest

# arguments
ARG INSTALL_DIR=/usr/local

# environment
ENV HOKUYO_DRIVER_INSTALL_DIR $INSTALL_DIR

# copy source
COPY ./ /root/hokuyo/

# build
RUN cd /root/hokuyo/ && make

# build hokuyo driver
RUN cd /root/hokuyo/ && make BUILD_PREFIX=$HOKUYO_DRIVER_INSTALL_DIR

# publish hokuyo
#   TODO: I'd like to install this globally but it does not work for some reason,
#   the CMAKE_INSTALL_PREFIX variable always reverts to /root/hokuyo/build/.
ENV PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$HOKUYO_DRIVER_INSTALL_DIR/lib/pkgconfig/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOKUYO_DRIVER_INSTALL_DIR/lib/
