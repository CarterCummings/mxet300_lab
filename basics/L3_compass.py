import L1_ina
import L1_log
import time
import L2_compass_heading

while(1):
    voltage = L1_ina.readVolts()
    heading = L2_compass_heading.get_heading()

    #print(voltage)
    L1_log.tmpFile(voltage,"voltage.txt")
    L1_log.tmpFile(heading,"heading.txt")
    
    direction = ""
    if (heading >= (-22.5) and heading <= (22.5)):
        direction = "North"
    elif (heading >= (22.5) and heading <= (22.5+45)):
        direction = "North West"
    elif (heading >= (22.5+45) and heading <= (22.5+90)):
        direction = "West"
    elif (heading >= (22.5+90) and heading <= (22.5+135)):
        direction = "South West"
    elif (heading >= (22.5+135) or heading <= (-180+22.5)):
        direction = "South"
    elif (heading >= (-180+22.5) and heading <= (-135+22.5)):
        direction = "South East"
    elif (heading >= (-135+22.5) and heading <= (-90+22.5)):
        direction = "East"
    elif (heading >= (-90+22.5) and heading <= (-45+22.5)):
        direction = "North East"       


    L1_log.stringTmpFile(direction,"direction.txt")
    
    time.sleep(.25)
    