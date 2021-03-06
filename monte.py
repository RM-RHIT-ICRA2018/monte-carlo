import random, threading, time
import numpy as np
from math import exp,sqrt
import paho.mqtt.client as mqtt
import json
# x = [[i,i] for i in range(100)]
# y = [2 for i in range(100)]
# ysum = sum(y)
# y = [yi / ysum for yi in y]
num_points=100
num_iter=1000
# ===== Pure python methods

new_result = -1
num = 0
record = [0, 0, 0.0, 0.0, 0.0] #[result, no, p, i, d]
current_point = [0.0, 0.0, 0.0]

def on_connect(client, userdata, flags, rc):
    print("MQTT Interface Bind Success.")
    client.subscribe("/GIMBAL/TRAINING/RESULT")
    client.subscribe("/GIMBAL/TRAINING/REQUEST")

    print("MQTT Subscribe Success")

def on_message(client, userdata, msg):
    global new_result
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/GIMBAL/TRAINING/RESULT":
        new_result = payload["Result"]
        if new_result > record[0]:
            record[0] = new_result
            record[1] = num
            record[2] = current_point[0]
            record[3] = current_point[1]
            record[4] = current_point[2]
        print("New result received: %d" % new_result)
    elif msg.topic == "/GIMBAL/TRAINING/REQUEST":
        print(">>>>>>>>>>>> Score: %d, No: %d, P: %f, I: %f, D: %f <<<<<<<<<<<<" % (record[0],record[1],record[2],record[3],record[4]))

def weighted_random(x, y):
    """
    https://stackoverflow.com/questions/3679694/a-weighted-version-of-random-choice
    """
    space = {}
    current = 0
    for choice, weight in zip(x,y):
        if weight > 0:
            space[current] = choice
            current += weight
    rand = random.uniform(0, current)
    for key in sorted(list(space.keys()) + [current]):
        if rand < key:
            return choice
        choice = space[key]
    return None

def uniform_random_pertubation(point,i):
    return([random.uniform(max(k_lower[j],point[j]-k_range[j]/(2*(i+1))),
                          min(k_upper[j],point[j]+k_range[j]/(2*(i+1)))) for j in range(3)])

def random_pertubation(point,i):
    return([min(k_upper[j],max(k_lower[j],random.normalvariate(point[j],(k_range[j]/(1.5*(i+1)))/2.33))) for j in range(3)])

def test_points(point):
    global num
    global new_result
    num = num + 1
    print("Publishing new set, No: %d, P: %f, I: %f, D: %f" % (num, point[0], point[1], point[2]))
    client.publish("/GIMBAL/TRAINING/SET", json.dumps({"No": num, "Kp": point[0], "Ki": point[1], "Kd": point[2]}))
    new_result = -1
    while new_result < 0:
        client.publish("/GIMBAL/TRAINING/SET", json.dumps({"No": num, "Kp": point[0], "Ki": point[1], "Kd": point[2]}))
        time.sleep(0.2)
    print("New result comfirmed: %d" % new_result)
    return new_result
    # return((point[0]-0.1)**2+(point[1]-0.2)**2+(point[2]-0.3)**2)
# kp_lower=0
# kp_upper=1
# kp_range=kp_upper-kp_lower
# kd_lower=0
# kd_upper=1
# kd_range=kd_upper-kd_lower
# ki_lower=0
# ki_upper=1
# ki_range=ki_upper-ki_lower
k_lower=[0.0,0.0,0.0]
k_upper=[100.0,100.0,1.0]
k_range=[(k_upper[i]-k_lower[i]) for i in range(3)]
points=[[random.uniform(k_lower[i],k_upper[i]) for i in range(3)] for j in range(num_points)]
new_points=[[random.uniform(k_lower[i],k_upper[i]) for i in range(3)] for j in range(num_points)]
loss=[0 for i in range(num_points)]



def main_loop():
    global loss
    global new_points
    global points
    global current_point
    for i in range(num_iter):
        max_loss=0
        sum_loss=0
        for j in range(num_points):
            current_point = points[j]
            loss[j]=test_points(points[j])
            sum_loss+=loss[j]
            if loss[j]>max_loss:
                max_loss=loss[j]
        print(sum_loss/num_points)

        sum_loss=0.000000000001
        for j in range(num_points):
            loss[j]=(max_loss-loss[j])/max_loss
            sum_loss+=loss[j]
        for j in range(num_points):
            loss[j]=loss[j]/sum_loss
        for j in range(num_points):
            new_points[j]=random_pertubation(weighted_random(points,loss),i)
        #print(weighted_random(points,loss))
        points=new_points
        #print(points)

class TestThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            main_loop() 

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("MQTT Interface Start Binding.")

client.connect("127.0.0.1", 1883, 60)

Test_thread = TestThread()
Test_thread.start()

client.loop_forever()



