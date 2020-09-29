#Position of interesting joints
std_msgs/Float64 emg_rate

# Kinematics
std_msgs/Float64MultiArray shoulder_abd
std_msgs/Float64MultiArray shoulder_rot
std_msgs/Float64MultiArray shoulder_flex
std_msgs/Float64MultiArray elbow_flex
std_msgs/Float64 vel_shoulder_abd
std_msgs/Float64 vel_shoulder_rot
std_msgs/Float64 vel_shoulder_flex
std_msgs/Float64 vel_elbow_flex

# EMG signals
std_msgs/Float64MultiArray biceps
std_msgs/Float64MultiArray triceps
