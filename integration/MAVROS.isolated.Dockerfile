# Existing local ARM64 Gazebo/ROS image, used only as a separate test base.
FROM gnss-gazebo-ardupilot:dev
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-jazzy-mavros=2.16.0-1noble.20260927.172210 ros-jazzy-mavros-extras=2.16.0-1noble.20260927.181038 geographiclib-tools && /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
ENV ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1
ENTRYPOINT ["sleep", "infinity"]
