uint8 UNAVAILABLE = 0
uint8 SIGN_UNKNOWN = 1
uint8 AVAILABLE = 2

# Pose covariance is always provided by tracking.
geometry_msgs/PoseWithCovariance pose_with_covariance

uint8 orientation_availability

geometry_msgs/TwistWithCovariance twist_with_covariance

geometry_msgs/AccelWithCovariance acceleration_with_covariance

#value (default=False)
bool is_stationary