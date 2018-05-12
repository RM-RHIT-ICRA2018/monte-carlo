import random
import numpy as np
from math import exp,sqrt
# x = [[i,i] for i in range(100)]
# y = [2 for i in range(100)]
# ysum = sum(y)
# y = [yi / ysum for yi in y]
num_points=100
num_iter=1000
# ===== Pure python methods

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
    return((point[0]-0.1)**2+(point[1]-0.2)**2+(point[2]-0.3)**2)
# kp_lower=0
# kp_upper=1
# kp_range=kp_upper-kp_lower
# kd_lower=0
# kd_upper=1
# kd_range=kd_upper-kd_lower
# ki_lower=0
# ki_upper=1
# ki_range=ki_upper-ki_lower
k_lower=[0,0,0]
k_upper=[1,1,1]
k_range=[(k_upper[i]-k_lower[i]) for i in range(3)]
points=[[random.uniform(k_lower[i],k_upper[i]) for i in range(3)] for j in range(num_points)]
new_points=[[random.uniform(k_lower[i],k_upper[i]) for i in range(3)] for j in range(num_points)]
loss=[0 for i in range(num_points)]
for i in range(num_iter):
    max_loss=0
    sum_loss=0
    for j in range(num_points):
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



