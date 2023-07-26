# publish all data needed in stanley method at once

std_msgs/Header header

# subscribed data by /ego_vehicle/odometry
nav_msgs/Odometry car_odometry

# publish front wheel's center pose
geometry_msgs/Pose front_wheel_pose

# control msgs included
CarlaEgoVehicleStatus vehicle_status
