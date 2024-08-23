import canopen
import can
import pathlib
import struct

MOTOR_NODE_ID = 0x12
EDS_PATH = pathlib.Path(_file_).parent / 'steer.eds'
CLUTCH_ID = 0x705

network = canopen.Network()
node = canopen.RemoteNode(MOTOR_NODE_ID, str(EDS_PATH))
network.add_node(node)

filters = [
  { 'can_id': CLUTCH_ID, 'can_mask': 0x7FF, }
]

data = []

try:
  with can.Bus('can0', 'socketcan', can_filters=filters) as can0, network.connect(bustype='socketcan', channel='can1'):
    print("Recording. Press CTRL+C to stop.")

    while True:
      msg = can0.recv()
      assert msg.arbitration_id == CLUTCH_ID
      pot = struct.unpack('<h', msg.data[4:6])[0] / 10
      mot = node.sdo['Position actual value'].raw
      
      sample = (pot, mot)
      if len(data) == 0 or data[-1] != sample:
        data.append(sample)

        if len(data) % 50 == 0:
          print(len(data), "samples")

except KeyboardInterrupt:
  print("Stopped recording.")



import numpy as np
import scipy.stats

data = np.array(data)
print(scipy.stats.linregress(data[:,0], data[:,1]))