import VL53L1X
import time
class ToFVL53L1X(object):
    def __init__(self, address,bus_num):
        self.address = address
        # 1 = Short Range, 2 = Medium Range, 3 = Long Range 
        self.range = 1
        self.tof = VL53L1X.VL53L1X(i2c_bus=bus_num, i2c_address = self.address)
        self.start_sensor()
        
        self.my_roi = VL53L1X.VL53L1xUserRoi(tlx=6,tly=6,brx=9,bry=9)
        self.tof.set_user_roi(self.my_roi)
        
    def start_sensor(self):
        time.sleep(0.2)

        self.tof.open()
        self.tof.start_ranging(self.range)


    def stop_sensor(self):
        self.tof.stop_ranging()


    def get_distance(self):
        distance = 0.0
        distance = self.tof.get_distance() * 0.001 # mm to m conversion

        if distance >= 0:
            return distance
        else:
            return -1

if __name__ == '__main__':
    T = ToFVL53L1X(address=0x29,bus_num=7)