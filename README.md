Description
Our robot is a four-wheel machine with dimensions 210mm x 120mm x 160mm. The robot car was bought at the Bubble Gum store, but it was completely redesigned, and we left only the wheelbase. The elements for the robot were 3D printed. The main color of our robot is green, which is why we named our team “Greencar”. In our robot, the rear wheels are driving, and the front wheels are turned by a small servo.
  The following components were used in our robot:
   1) Pyboard v1.1
   2) Raspberry Pi 4 model B
   3) Camera for object recognition
   4) Fan for cooling the processor Raspberry Pi 4 model B
   5) Voltmeter
   6) Stabilizer that supplies 5V
   7) Compartment for 3 batteries, each battery 3V
   8) A breadboard that houses some of the above components
   9) Button (blue) to start the program
   10) Key to start power
   11) Servo motor used to turn the steering wheels
   12) Motor (1000 rpm) driving the rear wheels
   13) Components printed on a 3D printer:
        1. The case on which some of the above components are located
        2. Gears
        3. Camera stand
        4. Shock absorbers (without spring)
        5. Parts for steering
        6. Stand for the Servo motor
   14) Rear wheel (is an optional component that serves for beauty and maintenance of aesthetics)
   What about the program of our robot, it was written by a programmer from our team.
We decided to use a strategy with two teams, that is, the first program will be used for the Qualifying race, and the second for the final one, where green and red boxes will be placed.
  All two programs are written on microcontrollers (Pyboard and Raspberry Pi 4 model B). The Raspberry Pi 4 model B is programmed to read the picture from the camera, namely, two sides and two stripes (blue and orange). After that, our machine (robot) reads the values from the camera and enters them into a special algorithm consisting of tasks and equations to regulate the movement of the machine. After that, the values from the Raspberry Pi 4 model B are transferred to the Pyboard, which is used to control the motors. Also, the Raspberry Pi 4 model B determines the side to which the robot wakes up. If he sees the first strip of orange, then he goes one way (counterclockwise), but if he sees the first strip of blue, then he goes in the other direction (clockwise). The Raspberry Pi 4 model B identifies obstacles in the form of parallelepipeds. If the robot sees a green box through the camera, then it goes around it on the left side, and if it sees a red box, it goes around it on the right side.

  
   I would also like to tell you about our team. There are three people in our team (Dmitry, Igor, Zakhar), each of whom is engaged in his own duties:
   Igor is our programmer who wrote exactly the same two codes for our machine.
   Dmitry - deals with electronics, as well as documentation (a special contribution to such a project as an electromechanical robot circuit)
   Zakhar is an assistant programmer, as well as the person who wrote the documentation about the car and helped in the design of the electromechanical circuit.
We all study at the CRD (center for the development of robotics), and Anton Alekseevich (employee of the CRD) trains us.
For our robot, we wrote two programs in the python programming language: one program for Pyboard and one for Raspberry Pi 4 model B. The algorithm of our program is as follows: Raspberry Pi 4 model B reads and processes the image from the camera, then sends the generated data packet to the pyboard with using pins rx, tx, and the pyboard, in turn, reads this data packet and controls the movement of the robot across the field.
Here is an example of our program for the Raspberry Pi 4 model B:
import time
t = time.time()
import serial
import RobotAPI as rapi
import cv2
import numpy as np

robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
print("Start RAW, RobotAPI loaded", round(time.time() - t, 2))
robot.manual_regim = 1

temp = 0

# Green
low=np.array([43,142,0])
up=np.array([82,256,256])

# Red
lowr=np.array([0,0,0])
upr=np.array([23,256,256])

# Orange
lowo=np.array([15,24,12])
upo=np.array([40,256,256])

# Black
lowb=np.array([20,100,0])
upb=np.array([90,256,170])

# Blue
lowbl=np.array([70,0,65])
upbl=np.array([140,256,120])

# right sensor
xd1, yd1 = 600, 280
xd2, yd2 = 640, 480

# left sensor
xd3, yd3 = 0, 280
xd4, yd4 = 40, 480
# [76 40 69] [104 256 120]
# blue line sensor
xdb1,ydb1=275,440
xdb2,ydb2=360,470

# green and red sensor
xgr1, ygr1 = 185, 350
xgr2, ygr2 = 455, 435

# regulator
e = 0
e_old = 0
kp = 0.5
kd = 2
u = 0
speed = 0
krug = 0

z = 0
z_for_speed=0

t = time.time()
tt = 0
t1 = 0
time_for_ob=0
t_for_color=0
timer_for_speed=0

Flag_speed=True
speedg=False
speedr=False
Flag_speed1=False
Flag_go_back=False

while 1:
    frame = robot.get_frame(wait_new_frame=1)
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)

# right wall sensor
    dat2 = frame[yd1:yd2, xd1:xd2].copy()
    cv2.rectangle(frame, (xd1, yd1), (xd2, yd2), (0, 255, 255), 2)
    hsv2 = cv2.cvtColor(dat2, cv2.COLOR_BGR2HSV)
    maskd2 = cv2.inRange(hsv2, lowb, upb)
    imd2, contoursd2, hod2 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    e2 = 0

    for contor in contoursd2:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 500:
            cv2.rectangle(frame, (x + xd1, y + yd1), (x + w + xd1, y + h + yd1), (255, 255, 0), 2)
            e2 = w * h

# left wall sensor
    dat1 = frame[yd3:yd4, xd3:xd4].copy()
    cv2.rectangle(frame, (xd3, yd3), (xd4, yd4), (0, 255, 255), 2)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowb, upb)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    e1 = 0

    for contor in contoursd1:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 500:
            cv2.rectangle(frame, (x + xd3, y + yd3), (x + w + xd3, y + h + yd3), (255, 255, 0), 2)
            e1 = w * h

    # green and red sensor
    frame_red_green = frame[ygr1:ygr2, xgr1:xgr2].copy()
    cv2.rectangle(frame, (xgr1, ygr1), (xgr2, ygr2), (255, 255, 255), 2)
    hsv = cv2.cvtColor(frame_red_green, cv2.COLOR_BGR2HSV)
    maskg = cv2.inRange(hsv, low, up)
    img, contoursg, hog = cv2.findContours(maskg, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    maskr = cv2.inRange(hsv, lowr, upr)
    imr, contoursr, hor = cv2.findContours(maskr, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contor in contoursg:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 1000:
            cv2.rectangle(frame, (x + xgr1, y + ygr1), (x + w + xgr1, y + h + ygr1), (0, 0, 255), 2)
            if w<90:
                speedg=True
                robot.text_to_frame(frame, "green"+" "+str(w), x + xgr1, y + ygr1 - 20, (255, 0, 0))
            else:
                robot.text_to_frame(frame, "go back", x + xgr1, y + ygr1 - 20, (255, 0, 0))
                Flag_go_back = True
    for contor in contoursr:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 1000:
            cv2.rectangle(frame, (x + xgr1, y + ygr1), (x + w + xgr1, y + h + ygr1), (0, 255, 0), 2)
            if w<90:
                speedr=True
                robot.text_to_frame(frame, "red"+" "+str(w), x + xgr1, y + ygr1 - 20, (255, 0, 0))
            else:
                robot.text_to_frame(frame, "go back", x + xgr1, y + ygr1 - 20, (255, 0, 0))
                Flag_go_back=True
# blue line sensor
    datbl = frame[ydb1:ydb2, xdb1:xdb2].copy()
    cv2.rectangle(frame, (xdb1, ydb1), (xdb2, ydb2), (0, 255, 255), 2)
    hsvbl= cv2.cvtColor(datbl, cv2.COLOR_BGR2HSV)
    maskdbl = cv2.inRange(hsvbl, lowbl, upbl)
    imdbl, contoursdbl, hodbl = cv2.findContours(maskdbl, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    blue = 0

# find or
    for contor in contoursdbl:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 100:
            cv2.rectangle(frame, (x + xdb1, y + ydb1), (x + w + xdb1, y + h + ydb1), (255, 255, 0), 2)
            blue = w * h
            Flag_speed1 = True

    masko = cv2.inRange(hsvbl, lowo, upo)
    imdbl, contoursdbl, hodbl = cv2.findContours(masko, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    orange = 0

    # find or
    for contor in contoursdbl:
        x, y, w, h = cv2.boundingRect(contor)
        if cv2.contourArea(contor) > 100:
            cv2.rectangle(frame, (x + xdb1, y + ydb1), (x + w + xdb1, y + h + ydb1), (255, 255, 255), 2)
            orange = w * h
            Flag_speed1 = True

# check blue
    if blue > 0 and time.time() > t+tt:
        krug += 1
        t = time.time()
        tt = 0.25

    # check side
    if Flag_speed and Flag_speed1:
        if e1 == 0:
            z = 200
            z_for_speed = 2800
            Flag_speed = False
        if e2 == 0:
            z = 2800
            z_for_speed = 200
            Flag_speed = False

#  protect
    if speedr or speedg:
        if speedr:
            u = 2800
            if time.time() > time_for_ob + t_for_color:
                speedr = False

        if speedg:
            u = 200
            if time.time() > time_for_ob + t_for_color:
                speedg = False

    else:
        time_for_ob = time.time()
        if e1 > 0 and e2 > 0:
            e = e1 - e2
            u = int(1500 + e * kp + (e - e_old) * kd)
            e_old = e
        elif e1 == 0 and e2 == 0:
            robot.text_to_frame(frame, "None", 560, 200, (0, 0, 255))
            u=z
        elif e1 == 0:
            u = 200
        elif e2 == 0:
            u = 2800

# check stop or go forward or back
    if krug < 12:
        speed = 100
    else:
        if time.time() > t1 + 0.7:
            speed = 0
            u = 1500

    # print text
    text_for_me=z
    text = str(speed) + " " + str(u) + " " + str(e1)+ " " + str(e2)+ " " + str(e) + " " + str(blue) + " " + str(krug)
    message = str(speed) + " " + str(u) + "\n"
    port.write(message.encode("utf-8"))
    robot.text_to_frame(frame, text, 20, 60)
    robot.text_to_frame(frame, text_for_me, 560, 60,(0,0,255))
    robot.set_frame(frame, 40)

