import socket, struct, sys, json, time, os.path, threading, math, random
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt

PID_Items_BASIC = ["Chassis_Upper", "Chassis_Lower","Yaw_Upper","Yaw_Lower","Pitch_Upper","Pitch_Lower","Feeding_Upper","Feeding_Lower"]
PID_Item_No = 2
rounds = 1
start_point = [0,0,0]
start_initialized = [False,False,False]
threshold = 1
point_threshold = [10, 10, 10]

pid_inited = False
target_point = [0,0,0]
PID_set = [[],[],[]]
set_no = 0
for i in range(3):
    for j in range(13):
        PID_set[i].append(0.0)
count = 0
new_set = False
pid_updated = False
robot_ready = False
current_point = [0,0,0]
stand_by = False

def motor_name(no):
    if no == 2:
        return "dX"
    if no == 4:
        return "dY"

def on_connect(client, userdata, flags, rc):
    print("MQTT Interface Bind Success.")
    client.subscribe("/GIMBAL/SET")
    client.subscribe("/GIMBAL/TRAINING/SET")
    client.subscribe("/PID_FEEDBACK/CAN")
    client.subscribe("/UWB/POS")
    client.subscribe("/CHASSIS/AHRS/ALIG")
    client.subscribe("/PID_REMOTE/")

    print("MQTT Subscribe Success")
    # t = threading.Thread(target = CAN_RCV_LOOP)
    # t.start()

def at_point(p1, p2):
    for i in range(3):
        error = p1[i] - p2[i]
        if i == 2:
            if error > 180:
                error = error - 360
            elif error < -180:
                error = error + 360
        if abs(error) > point_threshold[i]:
            return False
    return True

def if_ready():
    global stand_by
    global start_initialized
    if not stand_by:
        for i in range(3):
            if not start_initialized[i]:
                return False
        print("Initialization complete, start_point: [ %f, %f, %f ], current_point: [ %f, %f, %f]")
    stand_by = True
    return at_point(current_point, start_point)

def on_message(client, userdata, msg):
    global new_set
    global PID_set
    global set_no
    global pid_updated
    global count
    global current_point
    global robot_ready
    global pid_inited
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/GIMBAL/TRAINING/SET":
        if set_no != payload["No"]:
            PID_set[0][PID_Item_No] = payload["Kp"]
            PID_set[1][PID_Item_No] = payload["Ki"]
            PID_set[2][PID_Item_No] = payload["Kd"]
            set_no = payload["No"]
            # PID_set[3][PID_Item_No] = payload["Gi"]
            print("<<<<<< New set received, No: %d, P: %f, I: %f, D: %f" % (set_no, PID_set[0][PID_Item_No], PID_set[1][PID_Item_No], PID_set[2][PID_Item_No]))
            pid_updated = False
            new_set = True

    elif msg.topic == "/PID_REMOTE/":
        for i in range(13):
            if i != PID_Item_No:
                PID_set[0][i] = payload["Ps"][i]
                PID_set[1][i] = payload["Is"][i]
                PID_set[2][i] = payload["Ds"][i]
        pid_inited = True


    elif msg.topic == "/PID_FEEDBACK/CAN":
        
        if not pid_updated:
            if abs(payload["Ps"][PID_Item_No
            ] - PID_set[0][PID_Item_No]) < 0.01:
                if abs(payload["Is"][PID_Item_No] - PID_set[1][PID_Item_No]) < 0.01:
                    if abs(payload["Ds"][PID_Item_No] - PID_set[2][PID_Item_No]) < 0.01:
                        pid_updated = True
                        return
            pid_updated = False
    elif msg.topic == "/GIMBAL/SET":
        if payload["Type"] == "Image":
            wanted = motor_name(PID_Item_No)
            if abs(payload[wanted]) < threshold:
                count = count + 1
    elif msg.topic == "/UWB/POS":
        if start_initialized[0]:
            current_point[0] = payload["posX"]
        else:
            start_point[0] = payload["posX"]
            start_initialized[0] = True
        if start_initialized[1]:
            current_point[1] = payload["posY"]
        else:
            start_point[1] = payload["posY"]
            start_initialized[1] = True
        robot_ready = if_ready()
        
    elif msg.topic == "/CHASSIS/AHRS/ALIG":
        if start_initialized[2]:
            current_point[2] = payload["Yaw"]
            robot_ready = if_ready()
        else:
            start_point[2] = payload["Yaw"]
            start_initialized[2] = True

def robot_stop():
    client.publish("/CHASSIS/SET", json.dumps({"Type": "velocity", "XSet": 0, "YSet": 0, "PhiSet": 0}))


def reset_robot():
    while not robot_ready:
        client.publish("/CHASSIS/SET", json.dumps({"Type": "position", "XSet": start_point[0], "YSet": start_point[1], "PhiSet": start_point[2]}))

def degreeFixer(angle):
    if angle >= 360:
        return angle - 360
    if angle < 0:
        return angle + 360
    return angle

def test_task():
    global target_point
    global current_point
    global start_point
    target_point = [start_point[0], start_point[1], degreeFixer(start_point[2] + 45)]
    while not at_point(target_point, current_point):
        time.sleep(0.1)
        client.publish("/CHASSIS/SET", json.dumps({"Type": "position", "XSet": target_point[0], "YSet": target_point[1], "PhiSet": target_point[2]}))
    target_point = [start_point[0], start_point[1], degreeFixer(start_point[2] - 45)]
    while not at_point(target_point, current_point):
        time.sleep(0.1)
        client.publish("/CHASSIS/SET", json.dumps({"Type": "position", "XSet": target_point[0], "YSet": target_point[1], "PhiSet": target_point[2]}))
    target_point = [start_point[0], start_point[1], degreeFixer(start_point[2])]
    while not at_point(target_point, current_point):
        time.sleep(0.1)
        client.publish("/CHASSIS/SET", json.dumps({"Type": "position", "XSet": target_point[0], "YSet": target_point[1], "PhiSet": target_point[2]}))

def do_test():
    global count
    global pid_updated
    if new_set:
        pid_updated = False
        print("Test starts, No: %d" % set_no)
        while not pid_updated:
            client.publish("/PID_REMOTE/", json.dumps({"Ps": PID_set[0], "Is": PID_set[1], "Ds": PID_set[2]}))
            time.sleep(0.2)
        print("PID update success")
        reset_robot()
        count = 0
        for i in range(rounds):
            print("Task Round %d" % i)
            test_task()
        print(">>>>>> Publishing result: %d" % count)
        client.publish("/GIMBAL/TRAINING/RESULT", json.dumps({"Result": count}))
        robot_stop()

class TestThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            do_test()







client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("MQTT Interface Start Binding.")

client.connect("127.0.0.1", 1883, 60)

client.loop_start()

while not pid_inited:
    client.publish("/PID_REMOTE/REQUEST", json.dumps({"Request": "Request"}))
    time.sleep(0.2)

print("PiD_SET initialized")

Test_thread = TestThread()
Test_thread.start()


