import L1_ina as ina
import L1_log as log
import time
import L2_vector as vec
import L1_lidar as lid

while(1):
    log.tmpFile(ina.readVolts(),"voltage.txt")

    log.tmpFile(vec.getNearest()[0],"distance.txt")
    log.tmpFile(vec.getNearest()[1],"heading.txt")

    log.tmpFile(vec.polar2cart(vec.getNearest()[0],vec.getNearest()[1])[0],"x.txt")
    log.tmpFile(vec.polar2cart(vec.getNearest()[0],vec.getNearest()[1])[1],"y.txt")

    time.sleep(.1)
