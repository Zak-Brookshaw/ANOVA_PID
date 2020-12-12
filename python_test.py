import numpy as np
from python.prototype import AnovaPID
import matplotlib.pyplot as plt
print("hello")

def control_action(tspan):
    A = 1.0
    toa = 100
    t_delay = 10

    output = A*(1 - np.exp(-(tspan - t_delay)/toa))*(tspan >= t_delay)

    return output

predH = 500
tspan = np.arange(0, predH, 1)

normal_c = control_action(tspan)
print(normal_c)
Kp = .01
toaI = 100
dt = 1
PID = AnovaPID(Kp, toaI, 0, dt)

Kp_step = Kp*.01
toaI_step = toaI*.05
grad_step = 0.01

anova_args = (Kp_step, toaI_step, grad_step)
PID.anova_settings(100, *anova_args)

control_var = np.zeros(predH)
setpt = 100
kp = []
toai= []
er = []
time = []
i=0
while i< 7*10**4:
    error = setpt - control_var[0]
    m_act = PID.compute(error)

    homo_control = np.append(control_var[1:], control_var[-1])
    control_var = homo_control + normal_c*m_act

    
    kp.append(PID.Kp)
    toai.append(PID.toaI)
    er.append(error)
    time.append(i*dt)
    i+=1


plt.figure(1)
plt.plot(time, kp)

plt.figure(2)
plt.plot(time, toai)
plt.figure(3)
plt.plot(time, er)


