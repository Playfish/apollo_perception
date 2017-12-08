string device = 1  # Something like "/dev/ttyXXX".  In pyhton should be string
int32 baud_rate = 2 #[default = 9600]
    # In general, we assumes no parity, 8 data bits, 1 stop bit, no
    # handshaking, break detection enabled. If not, add more fields here.
