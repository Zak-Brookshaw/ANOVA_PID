import numpy as np

pi_control = False
pid_settings = np.array([np.zeros(3)])

while len(pid_settings) < 8:
    
    while True:
        
        if pi_control:
            Kp_= 2*np.random.randint(0, 2) - 1  # randomly -1 or 1
            toaI_ = 2*np.random.randint(0, 2) - 1
            setting_ = np.array([[Kp_, toaI_]])
            
            
        else:
            Kp_= 2*np.random.randint(0, 2) - 1  # randomly -1 or 1
            toaI_ = 2*np.random.randint(0, 2) - 1
            toaD_ = 2*np.random.randint(0, 2) - 1
            setting_ = np.array([[Kp_, toaI_, toaD_]])
            
        rows, cols = pid_settings.shape
        if rows != np.max(np.matmul(pid_settings, setting_.T)):
            pid_settings = np.append(pid_settings, setting_, axis=0)
            break

X = np.ones((len(pid_settings), 1))
X = np.concatenate((X, pid_settings), axis=1)

if pi_control:
    X = np.concatenate((X, pid_settings[:, 0]*pid_settings[:, 1]), 
                       axis=1)
else:
    
    x12 = (pid_settings[:, 0]*pid_settings[:, 1])[:, np.newaxis]
    x13 = (pid_settings[:, 0]*pid_settings[:, 2])[:, np.newaxis]
    x23 = (pid_settings[:, 1]*pid_settings[:, 2])[:, np.newaxis]
    x123 = (pid_settings[:, 0]*pid_settings[:, 1]*pid_settings[:, 2])[:, np.newaxis]
    
    X = np.concatenate((X, x12), axis=1)            
    X = np.concatenate((X, x13), axis=1)
    X = np.concatenate((X, x23), axis=1)
    X = np.concatenate((X, x123), axis=1)
