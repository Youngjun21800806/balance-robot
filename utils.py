import cv2
import numpy as np
import matplotlib.pyplot as plt
import gym

def plotLearning(scores, filename, x=None, window=5):
    N = len(scores)
    running_avg = np.empty(N)
    for t in range(N):
        running_avg[t] = np.mean(scores[max(0, t-window):(t+1)])
        if x is None:
            x = [i for i in range(N)]
            plt.ylabel('score')
            plt.xlabel('game')
            plt.plot(x, running_avg)
            plt.savefig(filename)

def plot_learning_curve(timesteps_KP,timesteps_reward,episodes,scores,filename,lines=None):
    fig=plt.figure()
    ax=fig.add_subplot(311,label='1', frame_on=False)
    ax2=fig.add_subplot(312,label="2", frame_on=False)
    ax3=fig.add_subplot(313,label="3", frame_on=False)

    x1= [i for i in range(len(timesteps_KP))]
    ax.plot(x1,timesteps_KP, color="C0")
    ax.set_xlabel("Training Steps", color="C0")
    ax.set_ylabel("kp", color="C0")
    ax.tick_params(axis='x',colors="C0")
    ax.tick_params(axis='y',colors="C0")

    N = len(timesteps_reward)
    running_avg = np.empty(N)
    '''
    for t in range(N):
        if(timesteps_reward[t]>=50):
            timesteps_reward[t]=12
    '''
    for t in range(N):
        running_avg[t] = np.mean(timesteps_reward[max(0,t-2000):(t)])
    x2= [i for i in range(len(timesteps_reward))]
    ax2.plot(x2, running_avg, color="C1")
    ax2.set_xlabel("timeSteps", color="C1")
    ax2.yaxis.tick_right()
    ax2.set_ylabel('reward',color="C1")
    ax2.yaxis.set_label_position('right')
    ax2.tick_params(axis='y',colors="C1")

    N = len(scores)
    running_avg1 = np.empty(N)
    for t in range(N):
        running_avg1[t] = np.mean(timesteps_reward[max(0,t-100):(t)])
    x3= [i for i in range(episodes-1)]
    ax3.plot(x3, running_avg1, color="C2")
    ax3.set_xlabel("episodes", color="C2")
    ax3.yaxis.tick_right()
    ax3.set_ylabel('ep_reward',color="C2")
    ax3.yaxis.set_label_position('right')
    ax3.tick_params(axis='y',colors="C2")

    if lines is not None:
        for line in lines:
            plt.axvline(x=line)

    plt.savefig(filename)

