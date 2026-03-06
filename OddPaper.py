from robot import Robot
import time
import math
import random

COM_PORT = "COM4"
r = Robot(port_name=COM_PORT, debug=True)

#configuration
SPEED= 30
HORIZONTAL=176 # plan horizontale percu par le robot
SHEET_Z_POS=-160

SHEETS=[  
    [448, 265, 0, HORIZONTAL, -120], #A1
    [454, 94, 0, HORIZONTAL, -101], #A2
    [454, -84, 0, HORIZONTAL, -79], #A3
    [454, -255, 0, HORIZONTAL, -60] #A4
    ]


def setup():
    ANGLE_DEG = -30  # Exemple -30°

    # Homing
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