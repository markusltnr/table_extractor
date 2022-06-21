FROM ros:melodic-perception-bionic
# uJCpNeVz1Q3SaowTUyxF
SHELL ["/bin/bash", "-c"]
ENV HOSTNAME=localhost

RUN apt-get update && apt-get -qq -y --allow-unauthenticated install git vim python-catkin-tools python-pip curl \
	&& rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apt-get -qq -y --allow-unauthenticated install \ 
	ros-melodic-mongodb-store ros-melodic-ros-numpy python-backports.functools-lru-cache tmux \
	ros-melodic-move-base-msgs \
	&& rm -rf /var/lib/apt/lists/*
WORKDIR /home/v4r/catkin_ws/src

RUN git clone https://github.com/markusltnr/table_extractor.git
RUN git clone https://github.com/markusltnr/edith_msgs.git
#RUN git clone https://gitlab+deploy-token-43:zkdqJHaArD_VVZVYXJUy@rgit.acin.tuwien.ac.at/leitnermrks/stare_at_tables.git
RUN git clone https://github.com/IFL-CAMP/tf_bag.git
RUN pip install --upgrade setuptools
RUN pip install pyrsistent==0.16.1 transforms3d==0.3.1 open3d_ros_helper==0.2.0.3 jsonschema==2.4 
RUN pip install open3d==0.9.0.0
RUN pip install scikit-image==0.14.5
WORKDIR /home/v4r/catkin_ws/
RUN . /opt/ros/melodic/setup.sh && catkin build
RUN sysctl -w net.ipv6.conf.all.forwarding=1
