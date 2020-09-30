#Header header # I CANT GET THIS TO COMPILE!

int32 body_id
int32 tracking_status
int32 gesture

geometry_msgs/Point32 position2D # x,y in camera frame, z distance from camera

geometry_msgs/Point32 centerOfMass

#Position of interesting joints
geometry_msgs/Point32 head
geometry_msgs/Point32 neck
geometry_msgs/Point32 shoulder
geometry_msgs/Point32 spine_top
geometry_msgs/Point32 spine_mid
geometry_msgs/Point32 spine_bottom

geometry_msgs/Point32 l_shoulder
geometry_msgs/Point32 l_elbow
geometry_msgs/Point32 l_wrist
geometry_msgs/Point32 l_hand
geometry_msgs/Point32 l_hip
geometry_msgs/Point32 l_knee
geometry_msgs/Point32 l_foot

geometry_msgs/Point32 r_shoulder
geometry_msgs/Point32 r_elbow
geometry_msgs/Point32 r_wrist
geometry_msgs/Point32 r_hand
geometry_msgs/Point32 r_hip
geometry_msgs/Point32 r_knee
geometry_msgs/Point32 r_foot

# Robot is usually too close to see legs, and not very interesting


