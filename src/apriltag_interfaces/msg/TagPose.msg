# One detected AprilTag, pose in the camera optical frame of the enclosing TagPoseArray
string family    # pupil_apriltags family name, e.g. "tag36h11"
string id        # decoded tag id as a decimal string, e.g. "0"
geometry_msgs/Pose pose
# corner pixels as pupil_apriltags gives them, x0 y0 x1 y1 x2 y2 x3 y3
float64[8] corners
float32 decision_margin
int32 hamming
