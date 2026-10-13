# The one tag apriltag_detection tracks in one camera image.
# header.stamp is the image capture time, header.frame_id the camera frame
# (same as the TagPoseArray of the same image).
std_msgs/Header header
# false: no tag of the tracked family / id in this image; tag is then empty
bool valid
TagPose tag
# size of the image the corners refer to (px)
uint32 image_width
uint32 image_height
