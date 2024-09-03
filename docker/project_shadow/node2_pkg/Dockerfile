# Use a base ROS image
FROM osrf/ros:foxy-desktop

# Set up the workspace
WORKDIR /ros2_ws/src
COPY shared_pkgs/node2_pkg /ros2_ws/src/node2_pkg

# Build the workspace
WORKDIR /ros2_ws
# Entrypoint
# COPY docker/pkg_a/entrypoint.sh /ros2_ws/
# ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
