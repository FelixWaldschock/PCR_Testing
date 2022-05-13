import time
import sensor

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3, 20)
Photodiode1 = sensor.Sensor("Photodiode", ['in2/gnd'],3.3, 20)
Photodiode2 = sensor.Sensor("Photodiode", ['in3/gnd'],3.3, 20)
sensors = [Temperature1, Photodiode1, Photodiode2]

def nMeasuresTimed(n, deltatms, SensorNr):
    #Start the timer
    
    tstart = time.perf_counter_ns()
    ttot = tstart

    for i in range(n):
        #Measure
        #sensors[SensorNr].readSensorValue(readADC(ADC, sensors[SensorNr].pin))
        #Wait in the loop until time is elapsed
        while (True):
            tend = time.perf_counter_ns()
            deltat = tend-tstart
            if (deltat >= deltatms*10e5):
                #print("t", i, ":", deltat)
                break
            #print("tend-tstart", tend-tstart)   #Test print to see how long it takes

        #Restart timer
        tstart = time.perf_counter_ns()
        #print("break", i)  #Test print to see how long it takes
    #Get the average
    # value = sensors[SensorNr].getValue()
    print("Total time:", time.perf_counter_ns()-ttot) #Test print for total
    return 1

nMeasuresTimed(40, 1, 1)