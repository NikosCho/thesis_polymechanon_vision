std_msgs/Header header
# Header contains:
# 	uint32 seq	
# 	time stamp
# 	string frame_id
string text
# Message to pass (either Qrcode-text or Hazardous label-text)
geometry_msgs/Point point
# Point of detected object according to frame