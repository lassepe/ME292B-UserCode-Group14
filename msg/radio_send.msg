# Send up to four packets at a time
uint8 numPackets
uint8[30] data1
uint8[30] data2
uint8[30] data3
uint8[30] data4

# Packets are grouped together to ensure that the
# uwb radio ATTEMPTS to send the whole group one after the other.
# Note: this does not ensure receival of the entire group, as
# individual packets can still get dropped.