#For deployment we decided to get all dependencies inside this Dockerfile, it's also possible to run stage 5 only and use the
#base image FROM diy-full-description/ros-render:"$ROS_DISTRO" as diy-robotarm-driver instead.

#But then, its necessary to build a image from the diy_full_cell_description package first. (This will be cached and used as base image by stage 5)

#With this implementation, it's not necessary to build the image from the diy_full_cell_description package first, you can just run this container.

#sourcing of all dependencies in the "middle" stages is not necessary, but implemented because there's no disadvantage from.

##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user with root privilege
ARG USER=hephaestus
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID --shell $(which bash) $USER 

#install xacro and joint state publisher gui package
USER root
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-xacro
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui
USER ${USER}

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws


##############################################################################
##             2. stage: robotarm-description repo from github              ##
##############################################################################
FROM base as diy_robotarm

# Install git to clone diy-soft-robotarm-description packages
USER root
RUN apt-get update && apt-get install --no-install-recommends -y git
USER $USER

# Clone the diy-soft-robotarm-description package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_robotarm_wer24_description_ws/src
RUN cd /home/$USER/dependencies/diy_robotarm_wer24_description_ws/src && \
    git clone https://github.com/RobinWolf/diy_robotarm_wer24_description.git

# Build the diy-robotarm package
RUN cd /home/$USER/dependencies/diy_robotarm_wer24_description_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Add built diy-robotarm package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_robotarm_wer24_description_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

##############################################################################
##             3. stage: gripper-description repo from github               ##
##############################################################################
FROM diy_robotarm as diy_gripper

# Clone the diy-soft-gripper-description package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_soft_gripper_description_ws/src
RUN cd /home/$USER/dependencies/diy_soft_gripper_description_ws/src && \
    git clone https://github.com/RobinWolf/diy_soft_gripper_description.git
    
# Build the diy-gripper package
RUN cd /home/$USER/dependencies/diy_soft_gripper_description_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Add built diy-gripper package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_soft_gripper_description_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

##############################################################################
##               4. stage: cell-description repo from github                ##
##############################################################################
FROM diy_gripper as diy_cell

# Clone the diy-full-cell-description package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_robot_full_cell_description_ws/src
RUN cd /home/$USER/dependencies/diy_robot_full_cell_description_ws/src && \
    git clone https://github.com/RobinWolf/diy_robot_full_cell_description.git
    
# Build the diy-full cell description package
RUN cd /home/$USER/dependencies/diy_robot_full_cell_description_ws && \
   . /opt/ros/$ROS_DISTRO/setup.sh && \
   . /home/$USER/dependencies/diy_robotarm_wer24_description_ws/install/setup.sh && \
   . /home/$USER/dependencies/diy_soft_gripper_description_ws/install/setup.sh && \
   colcon build

# Add built diy-full cell description package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_robot_full_cell_description_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

##############################################################################
##                  5. stage: arm-driver repo from github                   ##     
##############################################################################
FROM diy_cell as diy_robotarm_driver       

#install necessary packages
USER root
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-controller-interface 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-controller-manager 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-hardware-interface 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-pluginlib 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rclcpp
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rclcpp-lifecycle
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros2-control
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros2-controllers
USER $USER

# Clone the diy-robotarm-wer24-driver package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_robotarm_wer24_driver_ws/src
RUN cd /home/$USER/dependencies/diy_robotarm_wer24_driver_ws/src && \
    git clone https://github.com/RobinWolf/diy_robotarm_wer24_driver.git
    
# Build and source the diy-robotarm-wer24-driver description package and source all dependeicies inside this stage
RUN cd /home/$USER/dependencies/diy_robotarm_wer24_driver_ws && \
   . /opt/ros/$ROS_DISTRO/setup.sh && \
   . /home/$USER/dependencies/diy_robot_full_cell_description_ws/install/setup.sh && \
   . /home/$USER/dependencies/diy_robotarm_wer24_description_ws/install/setup.sh && \
   . /home/$USER/dependencies/diy_soft_gripper_description_ws/install/setup.sh && \
   colcon build

# Add built diy-robotarm-wer24-driver package to ros entrypoint
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_robotarm_wer24_driver_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER


###################################################################################
##     6. sage: Gripper Driver (includes used Service to open/ close)            ##
###################################################################################
FROM diy_robotarm_driver as diy_gripper_driver


# Clone the diy-soft-gripper-driver package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_soft_gripper_driver_ws/src
RUN cd /home/$USER/dependencies/diy_soft_gripper_driver_ws/src && \
    git clone https://github.com/RobinWolf/diy_soft_gripper_driver.git


#install additional necessarity packages
USER root
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rclcpp
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rosidl-default-generators
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-std-srvs
USER ${USER}


# Build the diy-gripper-driver package and source all dependeicies inside this stage
RUN cd /home/$USER/dependencies/diy_soft_gripper_driver_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /home/$USER/dependencies/diy_robotarm_wer24_driver_ws/install/setup.sh && \ 
    . /home/$USER/dependencies/diy_robot_full_cell_description_ws/install/setup.sh && \
    . /home/$USER/dependencies/diy_robotarm_wer24_description_ws/install/setup.sh && \
    . /home/$USER/dependencies/diy_soft_gripper_description_ws/install/setup.sh && \
    colcon build


# Add built diy-gripper-driver package to entrypoint
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_soft_gripper_driver_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER


###################################################################################
##    7. stage: Moveit Image from driver Image (description already included)    ##
###################################################################################
FROM  diy_gripper_driver as diy_robotarm_moveit

# Clone the diy-soft-gripper-driver package into its own workspace
RUN mkdir -p /home/$USER/dependencies/diy_robot_moveit_ws/src
RUN cd /home/$USER/dependencies/diy_robot_moveit_ws/src && \
    git clone https://github.com/RobinWolf/diy_robot_wer24_moveit

# install dependencie packages
USER root
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && apt install -y  \
    ros-$ROS_DISTRO-moveit  \
    ros-$ROS_DISTRO-moveit-common  \
    ros-$ROS_DISTRO-moveit-servo  \
    ros-$ROS_DISTRO-xacro  \
    ros-$ROS_DISTRO-joint-trajectory-controller  \
    ros-$ROS_DISTRO-joint-state-broadcaster  \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-sensor-msgs-py  \
    ros-$ROS_DISTRO-joy*  \
    ros-$ROS_DISTRO-rqt-controller-manager
USER $USER

#install dependencies for python interface
USER root
RUN apt-get update && apt-get install -y pip
USER $USER

RUN pip install scipy

# copy dependencies folder from local machine --> maybe move to application package or clone from github?
#RUN mkdir -p /home/$USER/py_dependencies
#COPY ./dependencies /home/$USER/py_dependencies
RUN mv /home/$USER/dependencies/diy_robot_moveit_ws/src/diy_robot_wer24_moveit/py_dependencies /home/$USER

USER root
RUN chown -R "$USER":"$USER" /home/"$USER"/py_dependencies
USER $USER

RUN cd /home/"$USER"/py_dependencies/manipulation_tasks && pip install .


# Build and source the diy-robotarm-wer24-moveit description packages (3 in total) and source all dependeicies inside this stage
 RUN cd /home/$USER/dependencies/diy_robot_moveit_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /home/$USER/dependencies/diy_soft_gripper_driver_ws/install/setup.sh && \ 
    . /home/$USER/dependencies/diy_robotarm_wer24_driver_ws/install/setup.sh && \ 
    . /home/$USER/dependencies/diy_robot_full_cell_description_ws/install/setup.sh && \
    . /home/$USER/dependencies/diy_robotarm_wer24_description_ws/install/setup.sh && \
    . /home/$USER/dependencies/diy_soft_gripper_description_ws/install/setup.sh && \
    colcon build


# Add built diy-moveit package to entrypoint to make scripts acessable
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_robot_moveit_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

###################################################################################
##                8. stage: start Moveit with default launh arguments            ##
###################################################################################

CMD ["ros2", "launch", "diy_robot_wer24_moveit", "complete.launch.py"]