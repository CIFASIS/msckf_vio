FROM ros:kinetic-perception

ENV CATKIN_WS=/root/catkin_ws

RUN apt-get update && apt-get install -y \
      libsuitesparse-dev \
      ros-kinetic-tf-conversions \
      ros-kinetic-random-numbers && \
    rm -rf /var/lib/apt/lists/*


COPY ./ $CATKIN_WS/src/msckf_vio/

WORKDIR $CATKIN_WS

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make' && \
    sed -i '/exec "$@"/i \
           source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
