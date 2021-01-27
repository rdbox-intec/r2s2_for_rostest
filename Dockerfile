FROM ros:noetic-ros-core-focal

LABEL maintainer="INTEC Inc<info-rdbox@intec.co.jp>"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

COPY ./ros_entrypoint.sh /ros_entrypoint.sh

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
                                    git \
                                    build-essential \
                                    iputils-ping \
                                    net-tools \
                                    python3-pip

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                mkdir -p /catkin_ws/src && \
                cd /catkin_ws/src && \
                catkin_init_workspace && \
                cd /catkin_ws/ && \
                catkin_make && \
                source /catkin_ws/devel/setup.bash && \
                chmod +x /ros_entrypoint.sh && \
                apt autoclean -y && \
                apt autoremove -y && \
                rm -rf /var/cache/apt/archives/* /var/lib/apt/lists/*"

COPY . /catkin_ws/src/r2s2_for_rostest
RUN pip3 install -r /catkin_ws/src/r2s2_for_rostest/requirements.txt

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                cd /catkin_ws && \
                catkin_make"

COPY ./ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roscore"]