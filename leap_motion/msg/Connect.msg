#controlled by the left hand
bool is_translation   ##using is_circle

bool is_grab      ## if 1 : grabbed
#controlled by the right hand
bool is_twohands
##eg: is_forward=0 : no motion ; is_forward=1 : go forward ;is_forward=-1 : go backward ;translation
#translation
int8 is_forward
int8 is_right
int8 is_up
#rotation
int8 is_roll
int8 is_pitch
int8 is_yaw
