import L1_ina
import L1_log
import time

while(1):
    voltage = L1_ina.readVolts()
    #print(voltage)
    L1_log.tmpFile(voltage,"voltage.txt")
    time.sleep(.5)