import paho.mqtt.client as mqtt
import numpy as np # used for arrays
import gym # pull the environment
import time # to get the time
import math # needed for calculations
import matplotlib.pyplot as plt
import time
from utils import plot_learning_curve
##kp RL
LEARNING_RATE = 0.2
DISCOUNT = 0.8
total = 0
total_reward = 0
prior_reward = 0
#Observation = [30, 30, 50, 50] #position,velocity,angle,angular velocity
Observation = [101]#10<kp<70,0<ki<20
#np_array_win_size = np.array([0.25, 0.25, 0.01, 0.1])
np_array_win_size=np.array([0.5])
epsilon = 1
#qtable
q_table = np.random.uniform(low=0, high=1, size=(Observation + [3]))
q_table.shape
firsttime=0
episodes=1
timesteps=0
timesteps_reward=[]
timesteps_KP=[]
scores=[]
flag=0
##reward
errvalues=[]

Q=0.0001
R=0.05
xh0=0
xh1=0
P00=1
P01=0
P10=0
P11=1
dt=0.001
score=0


##suscriber callback->RL calculation---->value,pitchdot,pitch,kp,,,,, episodes,reward
def AGENT(client, userdata, message):
    global epsilon
    global timesteps
    global episodes
    global total_reward
    global timesteps_reward
    global timesteps_KP,scores,score,flag
    global errvalues
    global discrete_state
    global action
    global Q,R,xh0,xh1,P00,P01,P10,P11,dt

    start_time=time.time()

    if message.topic=="robot/info":
        arr=str(message.payload.decode("utf-8"))  #[reward,angle,kp]
        print(message.topic,end=', ')
        print(arr,end=', ')
        print(episodes)
        arr=arr.split(',')
        state=float(arr[0])
        done=float(arr[1])

        if done==0:
            reward=rewardmethod(errvalues)
            print(reward)
            errvalues=[]
            score=score+reward
            timesteps_reward.append(reward)
            timesteps_KP.append(state)
            timesteps=timesteps+1
            new_discrete_state = get_discrete_state(state)
            flag=0
            if timesteps==1:
                discrete_state=new_discrete_state
                print("timesteps is",timesteps,end=', ')
                if np.random.random() > epsilon:
                    action = np.argmax(q_table[discrete_state]) #take cordinated action
                else:
                    action = np.random.randint(0, 3)
            else:
                print("timesteps is",timesteps,end=', ')
                max_future_q = np.max(q_table[new_discrete_state])
                current_q = q_table[discrete_state + (action,)]
                new_q = (1 - LEARNING_RATE) * current_q + \
                LEARNING_RATE * (reward + DISCOUNT * max_future_q)
                q_table[discrete_state + (action,)] = new_q
                discrete_state = new_discrete_state
                if np.random.random() > epsilon:
                    action = np.argmax(q_table[discrete_state]) #take cordinated action
                else:
                    action = np.random.randint(0, 3) #0 or 1 do a random action->
            print("action is",action)
            client1.publish("robot/action",str(action))
            if timesteps==200:
                episodes=episodes+1
                scores.append(score)
                score=0
                flag=1
                timesteps=0
        elif done==1 and flag==0:
            episodes=episodes+1
            scores.append(score)
            score=0
            flag=1
            timesteps=0

        epsilon=10/episodes

        end_time=time.time()
        #print("time is",(end_time-start_time)*1000)

    elif message.topic=="robot/plot":
        filename = 'kpvalue_reward_2020.png'
        plot_learning_curve(timesteps_KP,timesteps_reward,episodes,scores,filename)

    elif message.topic=="robot/data": #stack err value to calculate variance and average value of err
        arr=str(message.payload.decode("utf-8"))
        arr=arr.split(',')
        mdata0=float(arr[0])
        mdata1=float(arr[1])
        mdata2=float(arr[2])
        mdata4=float(arr[3])
        acc=np.arctan2(mdata1,np.sqrt(mdata0*mdata0+mdata2*mdata2))*180/np.pi

        xhm0 = xh0 + dt * (mdata4-xh1)
        xhm1 = xh1
        Pm00 = P00 - dt * (P01 + P10) + dt * dt * P11 + Q
        Pm01 = P01 - dt * P11
        Pm10 = P10 - dt * P11
        Pm11 = P11 + Q
        K0 = Pm00 / (Pm00 + R)
        K1 = Pm10 / (Pm00 + R)

        xh0 = xhm0 + K0 * (acc - xhm0)
        xh1 = xhm1 + K1 * (acc - xhm0)

        P00 = Pm00 - K0 * Pm00
        P01 = Pm01 - K0 * Pm01
        P10 = Pm10 - K1 * Pm00
        P11 = Pm11 - K1 * Pm01

        errvalues.append(xh0)
        #print(float(xh0))


def get_discrete_state(state):
    #discrete_state = state/np_array_win_size+ np.array([15,10,1,10])
    discrete_state = state/np_array_win_size+ np.array([-20])

    return tuple(discrete_state.astype(int))

def rewardmethod(errvalues):
    variance=np.var(errvalues)
    average=np.average(errvalues)

    return (100/(average*average+20*variance+0.2))

##mqtt 통신, 루프문
broker_address = "192.168.124.100";

client1 = mqtt.Client("python")
client1.connect(broker_address,1883)
client1.subscribe("robot/info")
client1.subscribe("robot/plot")
client1.subscribe("robot/data")
client1.on_message = AGENT

client1.loop_forever()


