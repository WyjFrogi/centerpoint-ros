uint8 UNAVAILABLE = 0
uint8 SIGN_UNKNOWN = 1
uint8 AVAILABLE = 2

geometry_msgs/PoseWithCovariance pose_with_covariance

bool has_position_covariance
uint8 orientation_availability

geometry_msgs/TwistWithCovariance twist_with_covariance

bool has_twist
bool has_twist_covariance