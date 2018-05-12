import socket, struct, sys, json, time, os.path, threading, math, random
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt


Motor_Num = 4
rounds = 1
start_point = [0,0,0]
threshold = 1
point_threshold = [10, 10, 2]

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

def motor_name(no):
    if no == 4:
        return "dx"
    if no == 5:
        return "dy"

def on_connect(client, userdata, flags, rc):
    print("MQTT Interface Bind Success.")
    client.subscribe("/GIMBAL/SET")
    client.subscribe("/GIMBAL/TRAINING/SET")
    client.subscribe("/PID_FEEDBACK/CAN")
    client.subscribe("/UWB/PUS")

    print("MQTT Subscribe Success")
    # t = threading.Thread(target = CAN_RCV_LOOP)
    # t.start()

def if_ready():
    for i in range(3):
        if current_point[i] - start_point[i] > point_threshold[i]:
            return False
    return True

def on_message(client, userdata, msg):
    global new_set
    global PID_set
    global set_no
    global pid_updated
    global count
    global current_point
    global robot_ready
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/GIMBAL/TRAINING/SET":
        if set_no != payload["No"]:
            PID_set[0][Motor_Num] = payload["Kp"]
            PID_set[1][Motor_Num] = payload["Ki"]
            PID_set[2][Motor_Num] = payload["Kd"]
            set_no = payload["No"]
            # PID_set[3][Motor_Num] = payload["Gi"]
            print("New set received, No: %d, P: %f, I: %f, D: %f", set_no, PID_set[0][Motor_Num], PID_set[1][Motor_Num], PID_set[2][Motor_Num])
            pid_updated = False
            new_set = True
    elif msg.topic == "/PID_FEEDBACK/CAN":
        if payload["Ps"][Motor_Num] == PID_set[0][Motor_Num] and payload["Is"][Motor_Num] == PID_set[1][Motor_Num] and payload["Ds"][Motor_Num] == PID_set[2][Motor_Num]:
            pid_updated = True
    elif msg.topic == "/GIMBAL/SET":
        if payload["Type"] == "Image":
            wanted = motor_name(Motor_Num)
            if abs(payload[wanted]) < threshold:
                count = count + 1
    elif msg.topic == "/UWB/PUS":
        current_point[0] = payload["posX"]
        current_point[1] = payload["posY"]
        # current_point[2] = payload["posPhi"]
        robot_ready = if_ready()
    
def robot_stop():
    pass

def reset_robot():
    return
    while not robot_ready:
        pass

def test_task():
    global count
    time.sleep(5)
    count = random.randint(0, 100)

def do_test():
    if new_set:
        print("Test starts, No: %d",set_no)
        while not pid_updated:
            client.publish("/PID_REMOTE/", json.dumps({"Ps": PID_set[0], "Is": PID_set[1], "Ds": PID_set[2]}))
            time.sleep(0.2)
        reset_robot()
        count = 0
        for i in range(rounds):
            print("Task Round %d", i)
            test_task()
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

Test_thread = TestThread()
Test_thread.start()

client.loop_forever()