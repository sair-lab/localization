Header header					# ROS header
uint8 source_id					# Identity number of the data sending P4xx
uint8 source_idx				# Index number of the data sending node passed by rosparam
int8 antenna					# Antenna where the measurement was carried out
uint32 uwb_time 				# ms since radio boot at the time of the range conversation
uint16 data_size				# number of data bytes received
uint8[] data					# data array, data message will come seperately from the range
