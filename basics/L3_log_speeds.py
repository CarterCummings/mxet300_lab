import L1_ina as ina
import L1_log as log
import L2_kinematics as kine
import time

while(1):
    pdl = str(kine.getPdCurrent()[0])
    pdr = str(kine.getPdCurrent()[1])
    xdot = str(kine.getMotion()[0])
    thetadot = str(kine.getMotion()[1])
    print("PDL: "+pdl + " PDR: " + pdr + " Xdot: " + xdot + " Thetadot: " + thetadot)

    log.tmpFile(kine.getPdCurrent()[0],"PDL.txt")
    log.tmpFile(kine.getPdCurrent()[1],"PDR.txt")
    log.tmpFile(kine.getMotion()[0],"xdot.txt")
    log.tmpFile(kine.getMotion()[1],"thetadot.txt")

    log.tmpFile(ina.readVolts(),"voltage.txt")
    

    time.sleep(.1)


