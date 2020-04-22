#---------- FIRE FIGHTER ROBOT PYTHON 2.7 CODE ----------
#
# The robot detects fire using its camera module and approches
# it until the IR grid in the front detects the flames at a
# proximity of about 40cm. Then water is sprayed onto the flame
# until it is estinguished. Has a long range mode to shoot water
# from several meters away
#
# Authors: Ashwin Vinoo with support from Kuru


#---------- IMPORTING ALL THE NECESSARY MODULES ----------
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import numpy as np
from datetime import datetime
import time

# Last flame detected time is intitialized with current system time
last_flame_time = datetime.now()
# Creating a named window that can be resized as specified
cv2.namedWindow('Processed Flame Mask',cv2.WINDOW_NORMAL)
# Resizing the window to cover the entire width of a 720P screen
cv2.resizeWindow('Processed Flame Mask',1280, 640)
# Speifies that the pin number on the raspberry pi board in considered rather then that on the Broadcom chip
GPIO.setmode(GPIO.BOARD)
# Hides all GPIO warnings
GPIO.setwarnings(False)

# Function to get the milliseconds elapsed from passed datetime
def elapsedMillis(old_datetime):
    current_datetime = datetime.now()
    datetime_diff = current_datetime - old_datetime
    elapsed_millis = (datetime_diff.days*86400000) + (datetime_diff.seconds*1000) + (datetime_diff.microseconds/1000)
    return elapsed_millis

#---------- HYPERPARAMETERS ----------

# Motor PWM frequency
motor_pwm_frequency = 50000
# Motor PWM Duty Cycle (0-100)
motor_pwm_dutycycle = 65
# The picamera horizontal viewing angle
picamera_angle = 62.2
# The angle cone from the camera at which the flame should be detected for forward motion to occur
flame_forward_angle = 20
# Adjusts the time the robot turns towards a detected flame
rotation_offset = 0.36
# Determines the amount of time the robot waits for after rotating or moving towards a flame
motion_stop_millis = 200
# If a flame is no longer seen, the robot waits this much milliseconds before roating to find the next flame
flame_wait_millis = 2500
# The amount of time the robot incrementally rotates clockwise while searching for a flame
scan_rotation_millis = 400
# The amount of time the robot waits after incrementally rotating clockwise while searching for a flame
scan_wait_millis = 1000
# The amount of milliseconds the robot moves forward towards the camera detected flame without pause
camera_forward_millis = 3000
# The amount of milliseconds the robot moves forward towards the ir grid detected flame without pause
ir_forward_millis = 1500
# The amount of milliseconds the robot pumps water in long range
long_pump_millis = 1000
# The amount of milliseconds the robot waits after pumping water in long range
long_pump_wait = 400

#---------- BUZZER CONFIGURATION ----------

# Specifies the pin on the raspberry pi board that is linked to the buzzer
buzzer_pin = 16
# Specifies the GPIO pin usage mode as output and initial state as low (0V)
GPIO.setup(buzzer_pin, GPIO.OUT, initial = GPIO.LOW)
# Specifies the buzzer mode (0 is switched off, 1 is alarm at 4Hz and 2 is alarm at 2Hz)
buzzer_mode = 0
# Specifies the current buzzer state
buzzer_active = False
# Last buzzer toggled time is intitialized with current system time
buzzer_last_toggled_time = datetime.now()

#---------- SWITCH PIN ----------

#specifies the pin on the raspberry pi board that is connected to the switch that determines fire extinguising mode
switch_pin = 40
# Specifies the GPIO pin usage mode as input with pullup resistor activated
GPIO.setup(switch_pin, GPIO.IN, GPIO.PUD_UP)

#---------- LEFT AND RIGHT MOTOR CONFIGURATION ----------

# Specifies the pins on the raspberry pi board that are linked to the two motors
left_motor_1 = 11
left_motor_2 = 12
right_motor_1 = 13
right_motor_2 = 15

# Specifies the GPIO pin usage mode as output and initial state as low (0V)
GPIO.setup(left_motor_1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(left_motor_2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(right_motor_1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(right_motor_2, GPIO.OUT, initial = GPIO.LOW)

# Creates PWM objects to handle the PWM signals applied to the 4 pins
left_motor_1_pwm = GPIO.PWM(left_motor_1,motor_pwm_frequency)
left_motor_2_pwm = GPIO.PWM(left_motor_2,motor_pwm_frequency)
right_motor_1_pwm = GPIO.PWM(right_motor_1,motor_pwm_frequency)
right_motor_2_pwm = GPIO.PWM(right_motor_2,motor_pwm_frequency)

# list to store the motor lock status, datetime, time till next task/unlock, the current task and the next task
motor_lock_status = [False, 0, 0, "",""]

# Function that allows the user to control the movement of the robot using the two wheels
def motorControl(direction):
    if direction == "forward":
        motor_config = (1,0,0,1)
    elif direction == "reverse":
        motor_config = (0,1,1,0)
    elif direction == "right":
        motor_config = (1,0,1,0)
    elif direction == "left":
        motor_config = (0,1,0,1)
    else:
        motor_config = (0,0,0,0)

    # Write PWM signals to the pins of the two motors
    left_motor_1_pwm.start(motor_pwm_dutycycle*motor_config[0])
    left_motor_2_pwm.start(motor_pwm_dutycycle*motor_config[1])
    right_motor_1_pwm.start(motor_pwm_dutycycle*motor_config[2])
    right_motor_2_pwm.start(motor_pwm_dutycycle*motor_config[3])

#---------- PUMP CONFIGURATION ----------

# Indicates whether the long range mode attempted to extinguish a flame previously
long_range_attempted = False
# Specifies the pins on the raspberry pi board that are linked to the relay driving the pump
relay_pin = 7
# Specifies the GPIO pin usage mode as output and initial state as high (3.3V)
GPIO.setup(relay_pin, GPIO.OUT, initial = GPIO.HIGH)

# Function to spray water in sweeps with the specified number of iterations
def sprayWater(sweep_time=1.0, sweep_offset_time=0, iterations=2):
    current_iteration = 1
    motorControl("left")
    time.sleep(sweep_time/2 + sweep_offset_time)
    motorControl("stop")
    time.sleep(0.25)
    # Switching on the pump to start spraying water	
    GPIO.output(relay_pin, GPIO.LOW)
    while(current_iteration <= iterations):
        motorControl("right")
        time.sleep(sweep_time)
        motorControl("stop")
        time.sleep(0.25)
        motorControl("left")
        time.sleep(sweep_time)
        motorControl("stop")
        time.sleep(0.25)
        current_iteration = current_iteration+1
    # Now that the flame has been extinguished switch off the pump	
    GPIO.output(relay_pin, GPIO.HIGH)
    motorControl("left")
    time.sleep(sweep_time/2 + sweep_offset_time)
    motorControl("stop")
    time.sleep(0.25)

#---------- IR SENSOR ARRAY CONFIGURATION ----------

# Specifies the pins on the raspberry pi board that are linked to the IR sensor grid
ir_arry_1 = 35 # Rightmost IR Detector
ir_arry_2 = 33
ir_arry_3 = 37
ir_arry_4 = 29
ir_arry_5 = 31 # Leftmost IR Detector

# Specifies the GPIO pin usage mode as digital input
GPIO.setup(ir_arry_1, GPIO.IN)
GPIO.setup(ir_arry_2, GPIO.IN)
GPIO.setup(ir_arry_3, GPIO.IN)
GPIO.setup(ir_arry_4, GPIO.IN)
GPIO.setup(ir_arry_5, GPIO.IN)

# Function to detect angle at which the flame is located
def flameDetectIR():
    ir_grid = (GPIO.input(ir_arry_1), GPIO.input(ir_arry_2), GPIO.input(ir_arry_3), GPIO.input(ir_arry_4), GPIO.input(ir_arry_5))
    ir_detected = sum(ir_grid)
    if(ir_detected > 0):
        flame_angle = (ir_grid[0]*60 + ir_grid[1]*30 + ir_grid[3]*-30 + ir_grid[4]*-60)/ir_detected
    else:
        flame_angle = 0
    return [ir_detected, flame_angle]

#---------- CONFIGURING THE CAMERA FOR CAPTURING IMAGES ----------

# Defines the camera settings
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 30
camera.exposure_mode = 'snow'

# Specify the arry size matching the captured frame
rawCapture = PiRGBArray(camera, size=(320, 240))

#---------- CANDLE FLAME EXTINGUISING SECTION ----------

# for loop runs forever until the robot's power is swithced off
for frame in camera.capture_continuous(rawCapture, format = "rgb", use_video_port = True):

    # grab the raw NumPy array representing the image
    original_image = frame.array
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # Croping off the upper portion of the image
    original_image = original_image[120:240,:,:]
    # Splitting the image into blue, green and red channels
    blue, green, red = cv2.split(original_image)
	
    # The lower and upper pixel ranges are specified
    lower_pixels = np.array([220,200,140], dtype = "uint8")
    upper_pixels = np.array([255,255,255], dtype = "uint8")
    # Masking the pixels outside that range of brightness
    primary_mask = cv2.inRange(original_image, lower_pixels, upper_pixels)

    # Creating a secondary mask to compare levels of red, green and blue
    secondary_mask = np.zeros((120,320), dtype= "uint8")
    # Creating a secondary mask to compare levels of red, green and blue
    secondary_mask[(red >= (green+0)) & (green >= (blue+0))] = 255
    
    # Use bitwise-AND to obtain the final results
    primary_mask = cv2.bitwise_and(primary_mask,primary_mask, mask = secondary_mask)
	
    # Obtaining all the contours in the detected mask
    contours = cv2.findContours(primary_mask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Converts the primary mask to BGR format
    primary_mask = cv2.cvtColor(primary_mask,cv2.COLOR_GRAY2BGR)

    # These variables are used to detect the most likely path towards the flame
    average_x = 0
    total_area = 0
    flame_detected_camera = False
	
    # Browsing through the different contours and detecting possible flames
    for contour in contours:
        x,y,width,height = cv2.boundingRect(contour)
        area = width * height
        # Check to remove noise and other objects without a flame's aspect ratio
        if(area >= 1):
            flame_detected_camera = True
            average_x = average_x + area * (x + float(width)/2)
            total_area = total_area + area
            # Updates the primary mask with red boxes denoting detected flames
            cv2.drawContours(primary_mask,[contour],0,(0,0,255),3)
            
    # Calculating the weighted average value of average_x by dividing with total_area
    if(flame_detected_camera):
        average_x = average_x/total_area
    
    # Checking if flames were detected and at what angle     
    [flame_detected_IR, flame_angle_IR] = flameDetectIR()

    if(flame_detected_camera or flame_detected_IR>0):
        # Updating the time at which the flame was last detected
        last_flame_time = datetime.now()
        # Reading the mode of flame extinguishing
        long_range_mode = not GPIO.input(switch_pin)
        # Setting the buzzer mode based on the long range mode
        if(long_range_mode):
            buzzer_mode = 2
        else:
            buzzer_mode = 1

    # Actions to be performed if flames have been detected using the IR Sensor
    if((flame_detected_IR>0 and motor_lock_status[0] == False) or flame_detected_IR >= 3):
        # Converting flame detected angle into milliseconds that the motor will rotate
        ir_rotation_millis = (abs(flame_angle_IR)/1000.0) * rotation_offset
        # Since an action other than long range spraying is executed it is marked as not last attempted
        long_range_attempted = False
        if(flame_angle_IR <= -15):
            motorControl("left")
            motor_lock_status = [True, datetime.now(), ir_rotation_millis, "camera_left","camera_stop"]
        elif(flame_angle_IR >= 15):
            motorControl("right")
            motor_lock_status = [True, datetime.now(), ir_rotation_millis, "camera_right","camera_stop"]
        elif(flame_detected_IR <= 2 and not long_range_mode):
            motorControl("forward")
            motor_lock_status = [True, datetime.now(), ir_forward_millis, "camera_forward","camera_stop"]
        elif(flame_detected_IR >= 3 and not long_range_mode):
            # Switches the buzzer off while spraying water
            GPIO.output(buzzer_pin, GPIO.LOW)
            sprayWater()
        else:
            # Switches the buzzer off while spraying water
            GPIO.output(buzzer_pin, GPIO.LOW)
            sprayWater(sweep_time=0.4)
    # Actions to be taken if flames have been detected using the camera
    elif(flame_detected_camera and motor_lock_status[0] == False):
        # The necessary angle for the bot to move in order to face the flames
        rotation_angle = picamera_angle * float(average_x-159)/320
        # Converting it into milliseconds that the motor will rotate
        camera_rotation_millis = (abs(rotation_angle)/1000.0) * rotation_offset
        # Performing different actions based on the roation angle desired
        if(rotation_angle < -flame_forward_angle/2.0):
            motorControl("left")
            motor_lock_status = [True, datetime.now(), camera_rotation_millis, "camera_left","camera_stop"]
            long_range_attempted = False
        elif(rotation_angle > flame_forward_angle/2.0):
            motorControl("right")
            motor_lock_status = [True, datetime.now(), camera_rotation_millis, "camera_right","camera_stop"]
            long_range_attempted = False
        elif(long_range_attempted == False and long_range_mode):
            # Activates the pump
            GPIO.output(relay_pin, GPIO.LOW)
            motor_lock_status = [True, datetime.now(), long_pump_millis, "camera_long_pump","none"]
            long_range_attempted = True
        else:
            motorControl("forward")
            motor_lock_status = [True, datetime.now(), camera_forward_millis, "camera_forward","camera_stop"]
            long_range_attempted = False
    # Actions to be taken if no flames have been detected using the camera or IR sensor grid
    elif(motor_lock_status[0] == False):
        # Calculate the time since the last flame was detected
        elapsed_millis = elapsedMillis(last_flame_time)
        # If the time has elaspsed start rotating the robot in increments towards the right
        if(elapsed_millis > flame_wait_millis):
            buzzer_mode = 0
            GPIO.output(buzzer_pin, GPIO.LOW)
            motorControl("right")
            motor_lock_status = [True, datetime.now(), scan_rotation_millis, "empty_right","empty_stop"]
            long_range_attempted = False

    # Motor lock and control logic is implemented here
    if(motor_lock_status[0]):
        # Obtaining the time elapsed since the motor was locked for the task
        elapsed_millis = elapsedMillis(motor_lock_status[1])
        if(elapsed_millis > motor_lock_status[2]):
            if(motor_lock_status[3] in ["camera_forward","camera_left","camera_right"]):
                # Stops the motors
                motorControl("stop")
                motor_lock_status = [True, datetime.now(), motion_stop_millis, motor_lock_status[4],"none"]
            elif(motor_lock_status[3] in ["camera_long_pump"]):
                # Deactivates the pump
                GPIO.output(relay_pin, GPIO.HIGH)
                motor_lock_status = [True, datetime.now(), long_pump_wait, motor_lock_status[4],"none"]          
            elif(motor_lock_status[3] in ["empty_right"]):
                # Stops the motors
                motorControl("stop")
                motor_lock_status = [True, datetime.now(), scan_wait_millis, motor_lock_status[4],"none"]
            elif(motor_lock_status[3] in ["camera_stop","empty_stop","none"]):
                # Finally the lock is released for the next task to take over
                motor_lock_status[0] = False

    # Buzzer control logic is implemented here
    if(buzzer_mode == 1 or buzzer_mode == 2):
        elapsed_millis = elapsedMillis(buzzer_last_toggled_time)
        if((elapsed_millis >= 250 and buzzer_mode == 1) or (elapsed_millis >= 500 and buzzer_mode == 2)):
            buzzer_active = not buzzer_active
            GPIO.output(buzzer_pin, buzzer_active)
            buzzer_last_toggled_time = datetime.now()       

    # Concatenates the original image and primary mask side by side
    primary_mask = np.concatenate((original_image,primary_mask),axis=1)
    # Displays the original image and processed mask side by side
    cv2.imshow("Processed Flame Mask", primary_mask)
    # Processes GUI functions needed to show the image
    cv2.waitKey(1)
