# This message type is to get message of VIO State

Header header
float32 vision_failure		# 0-Vision Functioning, 1-IMU only functioning
float32 inertial_failure	# 0-IMU functioning, 1-IMU bias estimation is bad
float32 failure_drift		# Additional drift due to vision failure
int8 vio_failure		# -1-Unknown, 0-Normal & 1-VIO failure
int32 reset_counter 
