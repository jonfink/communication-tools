Header header
# Source (src) and Destination (dest) are designed to be abstract
# addresses (size=4 for ip, size=1 for zigbee)
int8[] src
int8[] dest
int8 type
float32 rssi
float32 tx_power
int8[] payload

# Packet types (for filtering)
int8 LINKTYPE=1
int8 ROUTINGTYPE=2
int8 APPTYPE=3