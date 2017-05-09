# All possible fields in the message are listed, but not currently needed.

#Standard metadata for higher-level stamped data types.
#This is generally used to communicate timestamped data 
#in a particular coordinate frame. 
#sequence ID: consecutively increasing ID 
#uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
#time-handling sugar is provided by the client library
#time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
#string frame_id
Header header

# frame number
uint64 frame

# additional message options provided by the gazetool. Currently not set or needed
#string id
#string label

# used for eye closeness detection TODO: figure out the range of number and what they mean
#float32 lid

# horizontal gaze angle TODO: figure out the range of number and what they mean
float32 horGaze

# vertical gaze angle TODO: figure out the range of number and what they mean
float32 verGaze

# binary classification result of mutual gaze. 
#  0: no mutual gaze
#  1: mutual gaze detected
bool mutGaze
