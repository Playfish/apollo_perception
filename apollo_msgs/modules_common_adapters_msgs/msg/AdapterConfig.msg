
# Property of a certain Input/Output that will be used by a module.

#Message Type
AdapterConfigMessageType  type

#Mode
AdapterConfigMode mode

#The max number of received messages to keep in the adapter, this field
#is not useful for PUBLISH_ONLY mode messages.
int32 message_history_limit
