import socket, struct, sys, json, time, os.path, threading, math
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB

rob = ROB.robot()

Motor_Num = 4
rounds = 1
start_point = [0,0,0]
threshold = 1
point_threshold = [10, 10, 2]

PID_set = [[],[],[]]
set_no = 0
for i in range(3):
    for j in range(len(rob.PID_Items)):
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
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/GIMBAL/SET")
    client.subscribe("/GIMBAL/TRAINING/SET")
    client.subscribe("/PID_FEEDBACK/CAN")
    client.subscribe("/UWB/PUS")

    print(BSP_ERROR.notice("MQTT Subscribe Success"))
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
    while not robot_ready:
        pass

def test_task():
    pass  

def do_test():
    if new_set:
        while not pid_updated:
            client.publish("/PID_REMOTE/", json.dumps({"Ps": PID_set[0], "Is": PID_set[1], "Ds": PID_set[2]}))
            time.sleep(0.2)
        reset_robot()
        count = 0
        for i in range(rounds):
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

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

Test_thread = TestThread()
Test_thread.start()

client.loop_forever()