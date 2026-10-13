# All AprilTags detected in one camera image (empty when none is visible).
# header.stamp is the image capture time, header.frame_id the camera frame.
std_msgs/Header header
TagPose[] tags
# size of the image the corners refer to (px)
uint32 image_width
uint32 image_height
