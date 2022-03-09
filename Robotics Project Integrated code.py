#!/usr/bin/python3
import threading
import math
from typing import OrderedDict
from Ax12 import Ax12
import time
import numpy as np
import RPi.GPIO as GPIO

"  Intialisation       >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"

print("--------------------------------------------------------------------------------------------------------------------------------------")
# sets baudrate and opens com port
Ax12.DEVICENAME = 'COM3'
Ax12.BAUDRATE = 1_000_000
Ax12.connect()

'Motor instances'
motorlist = []
motor_speed_list = []

#Motor 1 - rotation of the base
motor_1 = Ax12(1)
m1p = motor_1.get_present_position()
motorlist.append(motor_1)

#motor 2
motor_2 = Ax12(2)
m2p = motor_2.get_present_position()
motorlist.append(motor_2)

#motor 3
motor_3 = Ax12(3)
m3p = motor_3.get_present_position()
motorlist.append(motor_3)

#motor4
motor_4 = Ax12(4)
m4p = motor_4.get_present_position()
motorlist.append(motor_4)

"Link instances (length in cm)"
link_1 = 9.5
link_2 = 9.5
link_3 = 0
link_list = [link_1, link_2, link_3]

"Dispensor GPIO parameters"
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

servo_coffee=GPIO.PWM(7,50)
servo_sugar=GPIO.PWM(11,50)
servo_milk=GPIO.PWM(12,50)

servo_coffee.start(0)
servo_sugar.start(0)
servo_milk.start(0)

"  Functions   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
global adrlist
adrlist=[]
def BaristaThread():
    print("Thread 1 initialized...")            
    PORT = 6355
    s= socket(AF_INET, SOCK_STREAM)
    s.bind(("", PORT))
    s.listen(1)
    print ('Listening.....')
    while True:
        time.sleep(0.5)
        try:
            clientsocket,addr= s.accept()
            print ('Connected by:', addr)
            while True:
                msg=clientsocket.recv(1024)
                adrmsg=str(msg.decode("utf-8"))
                if len(adrlist)<5:
                    adrlist.append(adrmsg)
                else:
                    print("Too many order.....")
                #print(adrmsg)
                if not msg:
                    break
        except KeyboardInterrupt:
            print("Terminating.....")
            clientsocket.close()
            exit()
bt=threading.Thread(target=BaristaThread)                           #Create thread
bt.start()                                                          #Thread initialize

def return_to_neutral():
    rtn = True
    print("\nReturning to neutral position...\n")
    angle_list = calculate_angles(7, -1, 0, link_list)              #input coordinates x,y,z here
    step_list = angle_to_step(angle_list)
    execute_movement(step_list,rtn)

def calculate_individual_motor_speed(step_list, rtn):
    motor_speed_list = []
    current_position_list = []
    print(step_list)

    # time = float(input('Time Constraint (in seconds): '))

    for i in range(0, len(motorlist)):
        current_position = motorlist[i].get_present_position()
        current_position_list.append(current_position)
        print(current_position_list)
        step_diff = abs(step_list[i] - current_position_list[i])
        if rtn == False:
            steps_per_second = step_diff
        else:
            steps_per_second = step_diff/4
        motor_speed = round(steps_per_second/2)
        motor_speed_list.append(motor_speed)

    return motor_speed_list

def calculate_angles(x_coor_target, y_coor_target, z_coor_target, link_list):
    "Finding the angles for each joint"

    "Link lengths"
    AL = link_list[0]  #link A
    BL = link_list[1]  #link B
    CL = link_list[2]  #link C"
    rotation = 0

    "finding direction angle for motor_1"
    if (x_coor_target > 0 and y_coor_target) > 0:
        rotation = math.atan(x_coor_target/y_coor_target)
    elif (x_coor_target < 0 and y_coor_target) > 0:
        rotation = -math.atan(abs(x_coor_target)/y_coor_target)
    elif (x_coor_target > 0 and y_coor_target) < 0:
        rotation = math.pi - math.atan(x_coor_target/abs(y_coor_target))
    elif (x_coor_target < 0 and y_coor_target) < 0:
        rotation = -math.pi + math.atan(x_coor_target/y_coor_target)

    radius = math.sqrt(x_coor_target*x_coor_target + y_coor_target*y_coor_target)
    x_rel = radius - CL   # relative distance to distal end of link 2
    if x_rel < 0 or z_coor_target < -5:
        print("unable to reach coordinates - too low")
    if abs(rotation) > 2.61799:
        print("unable to reach coordinates - rotation out of range")
    else:
        minimum_radius = math.sqrt(AL*AL + BL*BL - 2*AL*BL * math.cos(0.523599))
        max_radius = AL + BL
        DL = math.sqrt(x_rel*x_rel + z_coor_target*z_coor_target) #distance from origin
        if DL > minimum_radius or DL < max_radius:
            "internal angles calculations based on constrains: link 3 is always horizontal"
            alpha = math.pi/2 - (math.acos((BL*BL - AL*AL - DL*DL)/(-2*AL*DL)) + math.tan(z_coor_target/x_rel))
            betta = math.pi - math.acos((AL*AL + BL*BL - DL*DL)/(2*AL*BL))
            charlie = math.pi/2 - (alpha + betta)
            print("\nAngles are: ", rotation*(180/math.pi), " ", alpha*(180/math.pi), " ", betta*(180/math.pi), " ", charlie*(180/math.pi))
            return [rotation*(180/math.pi), alpha*(180/math.pi), betta*(180/math.pi), charlie*(180/math.pi)]
        else:
            print("unable to reach coordinates - out of range")
            print("Limits are: min - ", minimum_radius, ", max - ", max_radius)

def angle_to_step(angle_list):
    "Takes angle (radian) as input and return step goal"

    "neutral position at approx 511 steps"
    angle_per_step = 300/1023
    rotation_step = int(round(-(angle_list[0] / angle_per_step) + 512))
    alpha_step = int(round(-(angle_list[1] / angle_per_step) + 512))
    betta_step = int(round(-(angle_list[2] / angle_per_step) + 512))
    charlie_step = int(round(-(angle_list[3] / angle_per_step) + 512))

    step_list = [rotation_step, alpha_step, betta_step, charlie_step]
    print("Steps: ", step_list[0], " ", step_list[1], " ", step_list[2], " ", step_list[3], "\n")

    "ensuring steps are valid and do not exceed rotatable values"
    for i in range(len(step_list)):
        if step_list[i] > 1023:
            step_list[i] = math.floor(step_list[i]) - 1
        if step_list[i] < 0:
            step_list[i] = math.ceil(step_list[i]) + 1
    return step_list

def execute_movement(step_list, rtn):
    motor_speed_list = calculate_individual_motor_speed(step_list, rtn)
    print("Executing motor movements...\n")

    start = time.time()

    for i in range(0, len(motorlist)):
        (motorlist[i]).set_moving_speed(motor_speed_list[i])
        (motorlist[i]).set_goal_position(int(step_list[i]))

    while Ax12(1).is_moving() == 1 or Ax12(2).is_moving() == 1 or Ax12(3).is_moving() == 1 or Ax12(4).is_moving() == 1:
        continue    

    end = time.time()
    time_elapsed = end - start
    print("\nTime elapsed: %fs" %(time_elapsed))

def quit_input():
    quit = input('\nWould you like to rerun the programme? (y/n) ')
    if quit == 'n':
        print(" ")
        return False
    else:
        return True

def Gripper_Actions(OpenGripper):
    if OpenGripper == True:
        "Function to open gripper"
        motor_6.set_moving_speed(80)
        motor_6.set_goal_position(math.floor(600 + 1023/4))
        time.sleep(2)
    else:
        "Function to close gripper"
        motor_6.set_moving_speed(80)
        motor_6.set_goal_position(475)
        time.sleep(2)


def cup(rtn, motorcoor, Grip):
    "motorcoor = [[pos1x,pos1y,pos1z],[pos2x,pos2y,pos2z],[pos3x,pos3y,pos3z]]"
    InvokeGripper = Grip[0]
    OpenGripper = Grip[1]
    P = 1

    for code in motorcoor:
        "code = [pos1x,pos1y,pos1z]"
        xcoor = code[0]
        ycoor = code[1]
        zcoor = code[2]
        angle_list = calculate_angles(xcoor, ycoor, zcoor, link_list)   # input coordinates x,y,z here
        step_list = angle_to_step(angle_list)
        execute_movement(step_list, rtn)
        time.sleep(2)
        
        if P == 2:
            if InvokeGripper == True:
                Gripper_Actions(OpenGripper)
        P = P + 1

def Dispensor_Machine(Ordertype, Order_Quantity, servo_coffee, servo_sugar, servo_milk):
    DispType = ""
    if Ordertype == "coffee":
        DispType = servo_coffee
    elif Ordertype == "sugar":
        DispType = servo_sugar
    elif Ordertype == "milk":
        DispType = servo_milk

    for i in range (Order_Quantity):
        time.sleep(2)
        DispType.ChangeDutyCycle(12)
        time.sleep(2)
        DispType.ChangeDutyCycle(2)
    


"   Main program      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
global Order
Order=[]
def main():
    print("Main initialize...")
    Operation = True
    CupSide = True                                                     # Prata flip for cupside, cus 2 cup slot
    Grip = [True,True]                                                 # 1st for invoking grip action (True to invoke), 2nd for open/close of gripper (True for open)
    Dispensor_Type = [coffee,sugar,milk]

    while Operation:
        if len(adrlist)>0:
            Order=list[0]
        rtn = False
        dispNo = 2
        motorcoor = []
        Order_Quantity = 0

        "Big circle 8.5^2 + 8.5^2 estimate to be 12^2, Small circle 5^2 + 5^2 estimate to be 7^2"
        Cuplocation = [
        [[7, 0, 0],[12, -1, -2],[7, 0, 0]],                             # Cup1 
        [[7, 0, 0],[12, 1, -2],[7, 0 ,0]],                              # Cup2
        [[5, 5, 0],[8.5, 8.5, 0],[5, 5, 0]],                            # Disp1
        [[0, 7, 0],[0, 12, 0],[0, 7, 0]],                               # Disp2
        [[-5, 5, 0],[-8.5,8.5, 0],[-5,5, 0]],                           # Disp3
        [[-7, 0, 0],[-12,0, -2],[-7,0, 0]]]                             # Person

        Grip[1] = False                                                 # Change state to close gripper (True, False)
        if CupSide == True:                                             # Pick up 1st cup, then change internally next time to take cup2
            motorcoor = Cuplocation[0]
            cup(rtn, motorcoor, Grip)
            CupSide = False
        else:
            motorcoor = Cuplocation[1]                                  # Pick up 2nd cup, then change internally next time to take cup1
            cup(rtn, motorcoor, Grip)
            CupSide = True
        
        Grip[0] = False                                                 # Stop invoking gripper action, gripper is closed (False, False)
        for numbering in range (3):                                     # To move to x dispensor location
            if Order[numbering] != 0:                                   # If order item is 0, skip moving to that area
                motorcoor = Cuplocation[dispNo]                         # Robotic arm to move to location
                cup(rtn, motorcoor, Grip)

                Order_Type = Dispensor_Type[numbering]                  # Check what to dispense and how many to dispense
                Order_Quantity = Order[numbering] 
                Dispensor_Machine(Ordertype, Order_Quantity, servo_coffee, servo_sugar, servo_milk)
            dispNo = dispNo + 1
        
        Grip = [True, True]                                             # Invoke Gripper and open gripper (True, True)
        motorcoor = Cuplocation[5]                                      # Move to person
        cup(rtn, motorcoor, Grip)

        return_to_neutral()                                             # Move to original location
        Operation = quit_input()

main()

# disconnect
for i in range(0, len(motorlist)):
    (motorlist[i]).set_torque_enable(0) 
Ax12.disconnect()
servo_coffee.stop()
servo_milk.stop()
servo_sugar.stop()
GPIO.cleanup()
print("--------------------------------------------------------------------------------------------------------------------------------------")

