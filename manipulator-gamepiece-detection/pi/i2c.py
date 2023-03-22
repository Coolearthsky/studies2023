import board
from RPi import GPIO
from ntcore import NetworkTableInstance
import adafruit_vl53l4cd

def main():
    # Rated distance range for this model of sensor
    min_distance = 0  # Technically the rated min is "~1mm"
    max_distance = 130

    # Initialize NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    # print("Setting up NetworkTables client")
    ntinst.startClient4("sensor")
    ntinst.setServer("10.1.0.2")
    table = ntinst.getTable("Distances")
    nt_distance_a = table.getDoubleTopic("distance_a").publish()
    nt_distance_b = table.getDoubleTopic("distance_b").publish()

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.OUT)

    # Turn off the first sensor in the chain (it will become sensor B),
    # but right now they both have the same I2C address
    GPIO.output(4, 0)

    # Initialize the VL53L4CD sensor A
    i2c = board.I2C()
    try:
        # First try finding sensor at default I2C address
        sensorA = adafruit_vl53l4cd.VL53L4CD(i2c)
        sensorA.set_address(0x34)
    except:
        # Already set address since sensor power-on, so find it at 0x34
        sensorA = adafruit_vl53l4cd.VL53L4CD(i2c, address=0x34)
    sensorA.inter_measurement = 0
    sensorA.timing_budget = 50

    # Turn sensor B back on now that there are no address conflicts
    GPIO.output(4, 1) 
    
    # Initialize the VL53L4CD sensor B
    sensorB = adafruit_vl53l4cd.VL53L4CD(i2c)
    sensorB.inter_measurement = 0
    sensorB.timing_budget = 50

    # Start ranging
    sensorA.start_ranging()
    sensorB.start_ranging()

    names=["A", "B"]
    nt_distance = [nt_distance_a, nt_distance_b]
    while True:
        for i, sensor in enumerate([sensorA, sensorB]):
            if sensor.data_ready:
                sensor.clear_interrupt()
                distance = sensor.distance
                if distance > max_distance or distance < min_distance:
                    # Sensor is giving us data outside its rated range
                    nt_distance[i].set(-1)
                else:
                    nt_distance[i].set(distance)
                    print("Distance {}: {} cm".format(names[i], distance))


main()