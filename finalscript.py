#necessary imports
import RPi.GPIO as GPIO
from time import sleep, time
import threading
import serial

#setup GPIO pins
GPIO.setmode(GPIO.BCM)
#ignore warnings
GPIO.setwarnings(False)

#define GPIO pins for motor 1
in1_1 = 24
in2_1 = 23
en_1 = 26
temp1_1 = 1 

#define GPIO pins for motor 2
in1_2 = 17
in2_2 = 27
en_2 = 19
temp1_2 = 1

# motor 1 setup
GPIO.setup(in1_1, GPIO.OUT)
GPIO.setup(in2_1, GPIO.OUT)
GPIO.setup(en_1, GPIO.OUT)
GPIO.output(in1_1, GPIO.LOW)
GPIO.output(in2_1, GPIO.LOW)

# motor 2 setup
GPIO.setup(in1_2, GPIO.OUT)
GPIO.setup(in2_2, GPIO.OUT)
GPIO.setup(en_2, GPIO.OUT)
GPIO.output(in1_2, GPIO.LOW)
GPIO.output(in2_2, GPIO.LOW)

#define GPIO pin for pressure sensor
pressure_sensor_pin = 6

# PWM setup for motor 1, pwm stands for pulse width modulation
p_1 = GPIO.PWM(en_1, 1000)
p_1.start(75)

# PWM setup for motor 2
p_2 = GPIO.PWM(en_2, 1000)
p_2.start(75)

#define ultrasonic sensor GPIO pins
TRIG = 22
ECHO = 25

#ultrasonic sensor GPIO pins setup
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

#function which returns distance measured by ultrasonic sensor to detect if their are obstacles
def measure_distance():
    #set TRIG pin to low to ensure clean start
    GPIO.output(TRIG, False)
    sleep(0.2)
    #send a pulse to TRIG pin to initiate measurement
    GPIO.output(TRIG, True)
    sleep(0.00001)
    #turn off trigger pulse
    GPIO.output(TRIG, False)
    #measure time taken for the echo to return
    start_time = time()
    while GPIO.input(ECHO) == 0:
        start_time = time()
    while GPIO.input(ECHO) == 1:
        stop_time = time()
    #calculate total time elapsed
    time_elapsed = stop_time - start_time
    #calculate distance by using speed of sound which is 34300 cm/s and divide by 2 because sound travels to the object and back
    distance = (time_elapsed * 34300) / 2 
    #return distance detected in cms
    return distance

#function to control motor direction and PWM based on input direction
def control_motors(direction):
    global temp1_1, temp1_2
    #r is used at the start when robot starts, both motors move in the same direction at the same speed
    if direction == 'r':
        print("run")
        if temp1_1 == 1:
            GPIO.output(in1_1, GPIO.HIGH)
            GPIO.output(in2_1, GPIO.LOW)
            print("forward motor 1")
        else:
            GPIO.output(in1_1, GPIO.LOW)
            GPIO.output(in2_1, GPIO.HIGH)
            print("backward motor 1")
        if temp1_2 == 1:
            GPIO.output(in1_2, GPIO.HIGH)
            GPIO.output(in2_2, GPIO.LOW)
            print("forward motor 2")
        else:
            GPIO.output(in1_2, GPIO.LOW)
            GPIO.output(in2_2, GPIO.HIGH)
            print("backward motor 2")
            
    #robot turns right by having both wheels running but the second motor at a slower speed of 40
    elif direction == 'R':
        print("turn R")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        p_1.ChangeDutyCycle(100)
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        p_2.ChangeDutyCycle(40) 
    
    ##robot turns left by having both wheels running but the first motor at a slower speed of 40
    elif direction == 'L':
        print("turn L")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        p_1.ChangeDutyCycle(40)
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        p_2.ChangeDutyCycle(100)
        
    #robot turns right until it makes a 180
    elif direction == 'TR':
        print("180 TR")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        p_1.ChangeDutyCycle(75)
        GPIO.output(in1_2, GPIO.LOW)
        GPIO.output(in2_2, GPIO.HIGH)
        p_2.ChangeDutyCycle(75)
        
    #robot turns left until it makes a 180
    elif direction == 'TL':
        print("180 TL")
        GPIO.output(in1_1, GPIO.LOW)
        GPIO.output(in2_1, GPIO.HIGH)
        p_1.ChangeDutyCycle(75)
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        p_2.ChangeDutyCycle(75)

    #robot turns right more sharply when the antenna degrees is less than -50
    elif direction == 'R2':
        print("turn R2")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        p_1.ChangeDutyCycle(15)
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        p_2.ChangeDutyCycle(85)

    #robot turns left more sharply when the antenna degrees is greater than 50
    elif direction == 'L2':
        print("turn L2")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        p_1.ChangeDutyCycle(85)
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        p_2.ChangeDutyCycle(15)
        
    #robot stops moving, both motors are set to low 
    elif direction == 's':
        print("stop")
        GPIO.output(in1_1, GPIO.LOW)
        GPIO.output(in2_1, GPIO.LOW)
        GPIO.output(in1_2, GPIO.LOW)
        GPIO.output(in2_2, GPIO.LOW)

    #robot moves forward
    elif direction == 'f':
        print("forward")
        GPIO.output(in1_1, GPIO.HIGH)
        GPIO.output(in2_1, GPIO.LOW)
        temp1_1 = 1
        p_1.ChangeDutyCycle(100)  
        GPIO.output(in1_2, GPIO.HIGH)
        GPIO.output(in2_2, GPIO.LOW)
        temp1_2 = 1
        p_2.ChangeDutyCycle(100)
        
    #robot moves back
    elif direction == 'b':
        print("backward")
        GPIO.output(in1_1, GPIO.LOW)
        GPIO.output(in2_1, GPIO.HIGH)
        temp1_1 = 0
        p_1.ChangeDutyCycle(100)
        GPIO.output(in1_2, GPIO.LOW)
        GPIO.output(in2_2, GPIO.HIGH)
        temp1_2 = 0
        p_1.ChangeDutyCycle(100)
        
    #tempo speed inputs, mainly used for testing purposes
    elif direction == 'l':
        print("low")
        p_1.ChangeDutyCycle(25)
        p_2.ChangeDutyCycle(25)
    elif direction == 'm':
        print("medium")
        p_1.ChangeDutyCycle(50)
        p_2.ChangeDutyCycle(50)
    elif direction == 'h':
        print("high")
        p_1.ChangeDutyCycle(75)
        p_2.ChangeDutyCycle(75)

    else:
        print("<<< wrong data >>>")

#function to check pressure sensor state
def check_pressure_sensor():
    #setup GPIO pin for input
    GPIO.setup(pressure_sensor_pin, GPIO.IN)
    #flag variable to track pressure sensor state changes
    statechange = None
    while True:
        #check if sensor is triggered
        if GPIO.input(pressure_sensor_pin) == GPIO.LOW:
            #check if state has changed
            if statechange != 1: 
                print("Pressure sensor triggered!\n")
                #update statechange to indicate sensor state
                statechange = 1
        #check if sensor is released
        elif GPIO.input(pressure_sensor_pin) == GPIO.HIGH:
            if statechange != 2:
                print("Pressure sensor released!\n")
                #update statechange to indicate sensor state
                statechange = 2
        sleep(0.1)

#function to read and process serial data for user mode (when robot is heading from dock to user)
def read_and_process_serial_user(ser):
    while True:
        #check if there is data waiting to be read from antenna
        if ser.in_waiting:
            #run and decode the line of data
            line = ser.readline().decode('utf-8').rstrip()
            #threshold for angle of antenna
            LAoA_DEG_THRESHOLD = 5
            #threshold for distance measured making the robot stop
            D_CM_THRESHOLD = 10
            #variable to track last processed angle of antenna
            last_processed_LAoA_deg = None
            #variable to track last processed distance measured
            last_processed_D_cm = None
            if "LAoA_deg" in line and "D_cm" in line: 
                try:
                    #extract values from line of data
                    LAoA_deg = float(line.split('"LAoA_deg":')[1].split(',')[0])
                    D_cm = float(line.split('"D_cm":')[1].split(',')[0])
                    #check if thresholds are exceeded
                    if last_processed_LAoA_deg is None or abs(LAoA_deg - last_processed_LAoA_deg) >= LAoA_DEG_THRESHOLD:
                        last_processed_LAoA_deg = LAoA_deg
                        #if robot is less than or equal to 100 cm from the user antenna, then the robot should stop and make a 180 degree turn after which the we break out of the infinite loop, exiting the function
                        if float(D_cm) <= 100:
                            print("D_cm below threshold. Stopping motors.")
                            control_motors('s')
                            #robot turns left for 5 seconds then stops, which is approximately a 180 degree turn
                            control_motors('TL')
                            start_time = time()
                            while True:
                                time_ran = time() - start_time
                                if time_ran >= 5: 
                                    control_motors('s')
                                    break
                            #exit function as robot is ready to execute dock mode
                            break
                        #if robot is further than 100 cm from the user antenna
                        else:
                            print("D_cm aka distance measured is above threshold. Resuming movement.")
                            #ultrasonic measures distance
                            distance = measure_distance() 
                            #if there is an obstacle within 20 cm of robot then robot should turn 90 degrees to the left, then move forward, then turn 90 degrees right, and move forward to get past obstacle
                            if distance < 20:
                                print("Object detected within 10 cm! Stopping motors.")
                                #90 degrees turn left
                                control_motors('L')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 4: 
                                        control_motors('s')
                                        break
                                #robot moves forward
                                control_motors('f')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 3: 
                                        control_motors('s')
                                        break
                                #90 degrees turn right
                                control_motors('R')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 4: 
                                        control_motors('s')
                                        break
                                #robot moves forward
                                control_motors('f')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 3: 
                                        control_motors('s')
                                        break
                            if last_processed_D_cm is None or abs(D_cm - last_processed_D_cm) >= D_CM_THRESHOLD:
                                last_processed_D_cm = D_cm
                                #if angle of arrival from antennas is less than -50 degrees, then turn right sharply to increase degrees
                                if float(LAoA_deg) < -50:
                                    print('Turning R2')
                                    control_motors('R2')
                                #if angle of arrival from antennas is between -50 and -14, then turn right to increase degrees
                                elif float(LAoA_deg) >= -50 and float(LAoA_deg) <= -14:
                                    print('Going R')
                                    control_motors('R')
                                #if angle of arrival from antennas is between -15 and 15, go straight as the robot is moving in the right direction of having a perfect arrival at the user station
                                elif float(LAoA_deg) >= -15 and float(LAoA_deg) <= 15:
                                    print('Going Straight')
                                    control_motors('f')
                                #if angle of arrival from antennas is between 16 and 50, then turn left to decrease degrees
                                elif float(LAoA_deg) >= 16 and float(LAoA_deg) <= 50:
                                    print('Turning L')
                                    control_motors('L')
                                #if angle of arrival from antennas is more than 50 degrees, then turn left sharply to decrease degrees
                                else: #basically elif float(LAoA_deg) > 50:
                                    print('Turning L2')
                                    control_motors('L2')
                #handles exception
                except ValueError:
                    print("JSON does not contain expected data.")

#function to read and process serial data for dock mode (when robot is heading from user back to dock)
def read_and_process_serial_dock(ser):
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').rstrip()
            LAoA_DEG_THRESHOLD = 5
            D_CM_THRESHOLD = 10
            last_processed_LAoA_deg = None
            last_processed_D_cm = None
            if "LAoA_deg" in line and "D_cm" in line: 
                try:
                    LAoA_deg = float(line.split('"LAoA_deg":')[1].split(',')[0])
                    D_cm = float(line.split('"D_cm":')[1].split(',')[0])
                    if last_processed_LAoA_deg is None or abs(LAoA_deg - last_processed_LAoA_deg) >= LAoA_DEG_THRESHOLD:
                        last_processed_LAoA_deg = LAoA_deg
                        #if robot is less than or equal to 95 degrees from the dock antenna, then stop, and make a 180 degrees turn based on the angle of arrival and then reverse backwards into the dock station acheiving the starting position of the whole trip
                        if float(D_cm) <= 95:
                            control_motors('s')
                            #180 degrees turn to the left if angle of arrival is less than or equal to 0
                            if last_processed_LAoA_deg <= 0:
                                #robot turns left
                                control_motors('TL')
                                start_time = time()
                                while True: 
                                    time_ran = time() - start_time
                                    if time_ran >= 5:
                                        control_motors('s')
                                        break
                                #robot reverses into dock
                                control_motors('b')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 9: 
                                        control_motors('s')
                                        break
                            #180 degrees turn to the right if angle of arrival is more than 0
                            else:
                                #robot turns right
                                control_motors('TR')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 5: 
                                        control_motors('s')
                                        break
                                #robot reverses into dock
                                control_motors('b')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 9: 
                                        control_motors('s')
                                        break
                            #exit function as robot is back to starting position
                            break
                        #if robot is further than 95 cm from the dock antenna, then resume movement
                        else: 
                            print("D_cm above threshold. Resuming movement.")
                            #retrieve ultrasonic distance measured
                            distance = measure_distance() 
                            #if obstacle is detected within 20 cm, then robot avoids it using same movement as seen in read_and_process_serial_user function
                            if distance < 20:
                                print("Object detected within 10 cm! Stopping motors.")
                                control_motors('L')
                                start_time = time()
                                while True:
                                    time_ran = time() - start_time
                                    if time_ran >= 4: 
                                        control_motors('s')
                                        break
                                control_motors('f')
                                start_time = time()
                                while True: 
                                    time_ran = time() - start_time
                                    if time_ran >= 3: 
                                        control_motors('s')
                                        break
                                control_motors('R')
                                start_time = time()
                                while True: 
                                    time_ran = time() - start_time
                                    if time_ran >= 4: 
                                        control_motors('s')
                                        break
                                control_motors('f')
                                start_time = time()
                                while True: 
                                    time_ran = time() - start_time
                                    if time_ran >= 3: 
                                        control_motors('s')
                                        break
                            #same action robot takes as read_and_process_serial_user function to try and get to the user antenna at an angle to ensure smooth reverse into starting position
                            if last_processed_D_cm is None or abs(D_cm - last_processed_D_cm) >= D_CM_THRESHOLD:
                                last_processed_D_cm = D_cm
                                if float(LAoA_deg) < -50:
                                    print('Turning R2')
                                    control_motors('R2')
                                elif float(LAoA_deg) >= -50 and float(LAoA_deg) <= -15:
                                    print('Going R')
                                    control_motors('R')
                                elif float(LAoA_deg) >= -15 and float(LAoA_deg) <= 15:
                                    print('Going Straight')
                                    control_motors('f')
                                elif float(LAoA_deg) >= 15 and float(LAoA_deg) <= 50:
                                    print('Turning L')
                                    control_motors('L')
                                elif float(LAoA_deg) > 50:
                                    print('Turning L2')
                                    control_motors('L2')
                except ValueError:
                    print("JSON does not contain expected data.")

#function to flush serial input and output buffers when we switch from having to read data from the user antenna to the dock antenna. helps with avoiding delay and performing incorrect actions based on old data
def flush_serial_buffer(ser):
    ser.reset_input_buffer()
    ser.reset_output_buffer()

def main():
    #initialize serial communication, both antennas have the same serial so communication is differentiated by having one antenna on and the other antenna off at all times
    serial_pass = serial.Serial('/dev/ttyACM0', 115200)
    #start a background thread to continuously monitor the state of the pressure sensor
    sensor_thread = threading.Thread(target=check_pressure_sensor)
    sensor_thread.daemon = True 
    sensor_thread.start()

    #Going to user mode
    while True:
        #robot doesnt start moving until the pressure sensor is triggered indicating that the robot now carries the package
        if GPIO.input(pressure_sensor_pin) == GPIO.LOW:
            print("Heading to user")
            #start a nonbackground thread to read and process serial data for user mode
            serial_thread = threading.Thread(target=read_and_process_serial_user, args=(serial_pass,))
            serial_thread.start()
            #wait for thread to finish which is when the robot comes within 100 cm of the antenna and then performs a 180 degree turn indicating user mode has ended and that it is ready for dock mode
            serial_thread.join()
            print("User mode deactivated")
            break

    #flush serial buffer, where the user antenna data is deleted so that once the user turns on the dock data, the robot can begin reading data from the proper antenna
    flush_serial_buffer(serial_pass)

    #Going to dock mode
    while True:
        #robot waits for the package to be removed before it starts heading back to the dock
        if GPIO.input(pressure_sensor_pin) == GPIO.HIGH:
            print("Heading to dock")
            #start a nonbackground thread to read and process serial data for dock mode
            serial_thread = threading.Thread(target=read_and_process_serial_dock, args=(serial_pass,))
            serial_thread.start()
            #wait for thread to finish which is when robot is within 95 cm of dock antenna and then performs a 180 reverse based on its angle and reverses into the dock returning to its starting position
            serial_thread.join()
            print("Dock mode deactivated")
            break

    #GPIO pins reset to default state so that they are ready for future use
    GPIO.cleanup()
    print("GPIO Clean up")

if __name__ == "__main__":
    main()