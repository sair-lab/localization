Header header					            # ROS header
uint8 requester_id
uint8 requester_idx
uint8 responder_id				            # Identity number of the responding P4xx
uint8 responder_idx				            # Index number of the node in the responding array passed by rosparam
float32 distance				            # Distance measurement
float32 distance_err                                        # Error estimated by P4xx
float32 distance_dot                                        # Range velocity estimated by Pxx
float32 distance_dot_err                                    # Range velocity error estimated by Pxx
uint8 antenna					            # Antenna where the measurement was carried out
uint16 stopwatch_time                                       # How long the range conversation took, in ms
uint32 uwb_time					            # ms since radio boot at the time of the range conversation nb
geometry_msgs/Point responder_location                      # Location of the responding node if known (explicitly declared as anchor by rosparam), otherwise 9999 indicates unknown.
