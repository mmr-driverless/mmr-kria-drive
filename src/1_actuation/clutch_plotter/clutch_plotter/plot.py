import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mmr_base.msg import EcuStatus
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def create_elapsed_milliseconds(start):
  return lambda: (datetime.now() - start).seconds*1e3 + (datetime.now() - start).microseconds/1e3

get_elapsed_milliseconds = create_elapsed_milliseconds(datetime.now())

class Plotter(Node):
  def __init__(self):
    super().__init__('clutch_plotter')
    self.vals = []
    self.times = []
    self.__stop_timer = self.create_timer(
      1,
      self.stop_timer_callback
    )
    self.__ecu_subscription = self.create_subscription(
      EcuStatus,
      '/status/ecu',
      self.ecu_callback,
      QoSPresetProfiles.SENSOR_DATA.value
    )

  def get_ms_from_header(self, ecu_status: EcuStatus) -> float:
    return ecu_status.header.stamp.nanosec/1e6 + ecu_status.header.stamp.sec*1e3

  def stop_timer_callback(self):
    stop_condition = (
      len(self.vals) > 0
      and get_elapsed_milliseconds() - self.times[-1] > 1*1e3
    )

    if stop_condition:
      x, y = np.array(self.times), np.array(self.vals)
      plt.plot(x, y)
      plt.grid(visible=True)
      plt.yticks(np.linspace(0, 100, 11))
      plt.show()
      self.__stop_timer.destroy()
    

  def ecu_callback(self, ecu_status: EcuStatus):
    self.vals.append(ecu_status.clutch_percentage)
    self.times.append(get_elapsed_milliseconds())
    pms = get_elapsed_milliseconds()
    rms = self.get_ms_from_header(ecu_status)
    print(f"{pms=} vs {rms=}")
    


def main():
  rclpy.init()
  p = Plotter()

  try:
    rclpy.spin(p)
  except KeyboardInterrupt:
    rclpy.shutdown()

if __name__ == "__main__":
  main()