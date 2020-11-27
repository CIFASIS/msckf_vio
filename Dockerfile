FROM ros:kinetic-perception

RUN apt-get update && apt-get install -y \
      libsuitesparse-dev \
      ros-kinetic-tf-conversions \
      ros-kinetic-random-numbers && \
    rm -rf /var/lib/apt/lists/*

# if you want to reference a previously defined env variable in another definition, use multiple ENV
ENV CATKIN_WS=/root/catkin_ws MSCKF_ROOT=/root/catkin_ws/src/msckf_vio/

COPY ./ $MSCKF_ROOT

WORKDIR $CATKIN_WS

COPY ./scripts/ $CATKIN_WS
RUN ["/bin/bash", "-c", "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./build.sh && ./modify_entrypoint.sh"]
