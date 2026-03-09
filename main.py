
from robot import Robot
import time
import math
import random
import RPi.GPIO as GPIO

pin=4
GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.OUT)

r = Robot(False)

#configuration
SPEED= 50
HORIZONTAL=176 # plan horizontale percu par le robot
SHEET_Z_POS=-160

SHEETS=[  
    [448, 265, 0, HORIZONTAL, -118], #A1 #[280, 447, -32, 177, -116]
    [454, 94, 0, HORIZONTAL, -99], #A2
    [454, -84, 0, HORIZONTAL, -77], #A3
    [454, -255, 0, HORIZONTAL, -58] #A4
    #[-13, 497, -50, HORIZONTAL, -150] #B0
    #[51, 345, -76, HORIZONTAL, -133] #B1
    #[142, 184, -88, HORIZONTAL, -103] #B2
    #[229, 33, -97, HORIZONTAL, -58] #B3
    #[329, -123, -69, HORIZONTAL, -29] #B4
    #[411, -272, -42, HORIZONTAL, -23] #B5
    ]


def setup():
    ANGLE_DEG = -30  # Exemple -30°

    # Homing
    r.reset_and_home_joints()
    #r.send_message_and_wait_conf("$H")
    r.set_joint_speed(SPEED)

    # Récupérer la position actuelle
    tool_pos = r.get_tool_pose()
    X_target, Y_target, Z_target, Pitch, Roll = tool_pos  # Z_target inclus
    print("Current tool position:", tool_pos)

    # --- Fonction : convertit un angle en coordonnées X,Y ---
    def angle_to_xy(angle_deg, x0, y0):
        theta = math.radians(angle_deg)
        norm = math.sqrt(x0*x0 + y0*y0)
        X = norm * math.cos(theta)
        Y = norm * math.sin(theta)
        return X, Y

    # Calcul des coordonnées pour l'angle choisi
    X_target, Y_target = angle_to_xy(ANGLE_DEG, X_target, Y_target)
    print(f"Coordinates for {ANGLE_DEG}° rotation: X={X_target}, Y={Y_target}")

    # --- Home truqué / reset coords pour rotation ---
    r.send_message_and_wait_conf(f"G92X{X_target}Y{Y_target}Z{Z_target}A{Pitch}B{Roll}")
    global FAKE_HOME
    FAKE_HOME = [X_target, Y_target, Z_target, Pitch, Roll]
    
    time.sleep(0.5)

def go_up_sheet(sheet_pos, slow=False):
    slow = slow
    if not slow:
        r.set_joint_speed(SPEED)
    else:
        r.set_joint_speed(SPEED*0.4)
    r.go_to_pose(sheet_pos) #up sheet
    
def touch_sheet(sheet_pos):
    r.set_joint_speed(SPEED*0.2)
    r.go_to_pose([sheet_pos[0], sheet_pos[1], -100, sheet_pos[3], sheet_pos[4]]) #get down
    r.set_joint_speed(SPEED*0.05)
    r.probe([sheet_pos[0], sheet_pos[1], SHEET_Z_POS, sheet_pos[3], sheet_pos[4]]) #probe slowly

def suck_sheet(sheet_pos):
    r.set_joint_speed(SPEED*0.10)
    #-> enable aspiration
    GPIO.output(4, GPIO.HIGH)
    #vacuum cup adjstment
    r.jog([-5, 0, 0, 0, 0]) 
    r.jog([0, 0, -5, 0, 0])
    r.jog([-5, 0, 0, 0, 0]) 
    r.jog([0, 0, -10, 0, 0]) 
    r.jog([-15, 0, -5, 0, 0]) 
    r.jog([17, 0, 0, 0, 0]) 
    time.sleep(2) 
    r.jog([0, 0, 20, 0, 0]) #vacuum cup adjstment
    r.set_joint_speed(SPEED*0.2)
    r.go_to_pose(sheet_pos) #go up sheet zone

def drop_sheet(sheet_pos):
    #disable aspiration
    GPIO.output(4, GPIO.LOW)
    time.sleep(2)
    r.set_joint_speed(SPEED*0.2)
    r.go_to_pose(sheet_pos) #go up sheet zone

def foetus_position(home):
    home = home
    r.set_joint_speed(SPEED)
    r.go_to_pose(FAKE_HOME) #safety pos
    if home : 
        r.send_message_and_wait_conf("$H")


setup()
"""
#r.send_message_and_wait_conf("$H")
go_up_sheet(SHEETS[3])
#touch_sheet(SHEETS[0])
time.sleep(2)
#go_up_sheet(SHEETS[0])
"""


#"""
#r.send_message_and_wait_conf("$H")
# --- Process starting


cycles = 2  # 1 cycle = suck (source) to drop (target)
sources = [1, 2]
targets = [0, 3]
iteration=0 # 1 iteration = 1 move (suck or drop)
random_enabled = True
for cycle in range(cycles):
    for target_index in targets:
        if random_enabled:
            source_index = random.choice(sources)
        else:
            source_index = sources[iteration % len(sources)]
        # ---- PICK depuis la source ----
        go_up_sheet(SHEETS[source_index], iteration)  # fast only for first cycle
        touch_sheet(SHEETS[source_index])
        suck_sheet(SHEETS[source_index])
        iteration+=1
        # ---- DROP vers la target ----
        go_up_sheet(SHEETS[target_index], iteration)  # always slow between targets
        touch_sheet(SHEETS[target_index])
        drop_sheet(SHEETS[target_index])
#"""
foetus_position(True)