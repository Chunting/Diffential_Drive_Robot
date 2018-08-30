# Hello World program in Python
import math

    
#state(x,y,theta of the adv and four trolleys)    

def model(state,actuators):
    d1=1
    d2=1
    d3=1
    d4=1
    dt=1
    new_state=[]
    
    x0=state[0]+actuators[0]*dt*math.cos(state[2])
    y0=state[1]+actuators[0]*dt*math.sin(state[2])
    t0=state[2]+actuators[1]*dt
    new_state.append(x0)
    new_state.append(y0)
    new_state.append(t0)
    #print(new_state)
    
    
    t1=state[5]+actuators[0]/d1 * math.sin(state[5]-state[2])
    x1=new_state[0]-d1*math.cos(t1)
    y1=new_state[1]-d1*math.sin(t1)
    #t1=state[5]+actuators[0]/d1 * math.sin(state[5]-state[2])
    #print(math.sin(state[2]-state[5]))
    #print(t1)
    new_state.append(x1)
    new_state.append(y1)
    new_state.append(t1)
    #print(new_state)
    
    t2=state[8]+actuators[0]/d2 *math.cos(state[2]-state[5])* math.sin(state[5]-state[8] )
    x2=new_state[3]-d2*math.cos(t2)
    y2=new_state[4]-d2*math.sin(t2)
    #t2=state[8]+actuators[0]/d2 *math.cos(state[2]-state[5])* math.sin(state[5]-state[8] )
    new_state.append(x2)
    new_state.append(y2)
    new_state.append(t2)
    #print(new_state)
    
    t3=state[11]+actuators[0]/d3 *(math.cos(state[5]+state[7])+math.cos(state[2]-state[5]))* math.sin(state[8]-state[11] )
    x3=new_state[6]-d3*math.cos(t3)
    y3=new_state[7]-d3*math.sin(t3)
    
    new_state.append(x3)
    new_state.append(y3)
    new_state.append(t3)
    #print(new_state)
    t4=state[14]+actuators[0]/d4 *(math.cos(state[7]+state[9])+math.cos(state[5]+state[7])+math.cos(state[2]-state[5]))* math.sin(state[11]-state[14] )
    x4=new_state[9]-d4*math.cos(t4)
    y4=new_state[10]-d4*math.sin(t4)
    
    new_state.append(x4)
    new_state.append(y4)
    new_state.append(t4)
    #print(new_state)
    return new_state
    
state=[10,5,0,9,5,0,8,5,0,7,5,0,6,5,0]
actuators=[1,0]

new_state= model(state,actuators)
print(new_state)

new_state1= model(new_state,actuators)
print(new_state1)

new_state2=model(new_state1,actuators)
print(new_state2)

new_state3=model(new_state2,actuators)
print(new_state3)

new_state4=model(new_state3,actuators)
print(new_state4)


import matplotlib.pyplot as plt
lst1=[new_state4[0],new_state4[3],new_state4[6],new_state4[9],new_state4[12]]
lst2=[new_state4[1],new_state4[4],new_state4[7],new_state4[10],new_state4[13]]

plt.plot(lst1, lst2, 'ro')
plt.axis([0, 20, 0, 20])
plt.show()


