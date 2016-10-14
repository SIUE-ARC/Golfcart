import usb.core
import usb.util
import time
# find our device
dev = usb.core.find(idVendor=0x4B4, idProduct=0x8051)

# was it found?
if dev is None:
    raise ValueError('Device not found')
else:
    print("Found device")

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

ep1 = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

assert ep1 is not None

ep2 = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

assert ep2 is not None

for i in range(0, 20):
    try:
        ep2.write("test")
        data = dev.read(ep1.bEndpointAddress,ep1.wMaxPacketSize)
        #RxData = ''.join([chr(x) for x in data])
        time.sleep(1)
        print data
    except usb.core.USBError as e:
        data = None
        if e.args == ('Operation timed out',):
            continue
