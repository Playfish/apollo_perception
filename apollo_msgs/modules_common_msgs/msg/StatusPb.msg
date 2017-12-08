#No error, reutrns on success.
uint32 OK = 0

#Control module error codes start from here.
uint32 CONTROL_ERROR = 1000
uint32 CONTROL_INIT_ERROR = 1001
uint32 CONTROL_COMPUTE_ERROR = 1002

#Canbus module error codes start from here.
uint32 CANBUS_ERROR = 2000
uint32 CAN_CLIENT_ERROR_BASE = 2100
uint32 CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101
uint32 CAN_CLIENT_ERROR_FRAME_NUM = 2102
uint32 CAN_CLIENT_ERROR_SEND_FAILED = 2103
uint32 CAN_CLIENT_ERROR_RECV_FAILED = 2104

#Localization module error codes start from here.
uint32 LOCALIZATION_ERROR = 3000

#Perception module error codes start from here.
uint32 PERCEPTION_ERROR = 4000
uint32 PERCEPTION_ERROR_TF = 4001
uint32 PERCEPTION_ERROR_PROCESS = 4002

#Prediction module error codes start from here.
uint32 PREDICTION_ERROR = 5000

#Planning module error codes start from here
uint32 PLANNING_ERROR = 6000

#HDMap module error codes start from here
uint32 HDMAP_DATA_ERROR = 7000

#Routing module error codes
uint32 ROUTING_ERROR = 8000
uint32 ROUTING_ERROR_REQUEST = 8001
uint32 ROUTING_ERROR_RESPONSE = 8002
uint32 ROUTING_ERROR_NOT_READY = 8003

#Indicates an input has been exhausted.
uint32 END_OF_INPUT = 9000

uint32 error_code
string msg
