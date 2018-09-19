FROM afdaniele/libbot2:latest

# arguments
ARG INSTALL_DIR=/usr/local

# environment
ENV HOKUYO_DRIVER_INSTALL_DIR $INSTALL_DIR

# copy source
COPY ./ /root/hokuyo/

# build hokuyo driver
WORKDIR /root
RUN cd hokuyo && BUILD_PREFIX=$HOKUYO_DRIVER_INSTALL_DIR make

# set entrypoint
ENTRYPOINT ["hr-hokuyo"]
