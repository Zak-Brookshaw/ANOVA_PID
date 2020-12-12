import numpy as np


class AnovaPID():
    
    pi_acc = 0
    error_prev = 0  
    
    def __init__(self, Kp, toaI, toaD, dt, control_PI=True) -> None:
        r"""
        

        Parameters
        ----------
        Kp : TYPE
            PID proportional constant
        toaI : TYPE
            PID integral constant
        toaD : TYPE
            PID derivative constant

        Returns
        -------
        None.

        """
        self.Kp = Kp
        self._cKp = Kp  # center for Kp
        self.toaI = toaI
        self._ctoaI = toaI  # center for toaI
        self.toaD = toaD
        self._ctoaD = toaD  # center for toaD
        self.dt = dt
        self.pi_control = control_PI
        self.pi_acc = 0
        self.error_prev = 0
        
        
    def anova_settings(self, mv_freq, *args) -> None:
        r"""
        
        This method stores values for the PID action done before continuous loop

        Parameters
        ----------

        mv_freq : int
            The number of discrete steps before moving to the next DOE setting

        args : tuple
            step sizes for kp, toaI, toaD and gradient descent
    
        Returns
        -------
        None
            DESCRIPTION.

        """
        if len(args) == 3:
            self.Kp_step, self.toaI_step, self.grad_step = \
                args  # step size in DOE and optimization
            self.pid_settings = np.array([np.zeros(2)])# initially centered
            self.num_settings = 5
            
        elif len(args)==4:
            self.Kp_step, self.toaI_step, self.toaD_step, self.grad_step = \
                args  # step size in DOE and optimization
            self.pid_settings = np.array([np.zeros(3)])# initially centered
            self.num_settings = 9
            
        self.mv_freq = mv_freq
        self.mv_count = 0
        self.anova_error = np.zeros((1,1))
        
        
    def mv_pid_DOE(self):
        r"""
        Randomly change the settings of the PID settings with the DOE space.
        Once all DOE points have been collected, the function moves to a new DOE center and repeats

        Returns
        -------
        zero : int
            Just a zero, using a return call to exit function 

        """
        
        if self.num_settings == len(self.pid_settings):
            self.anova_calc()
            return 0
        
        # randomly decides on the encoded PID parameters to move to next
        while True:
            
            if self.pi_control:
                Kp_= 2*np.random.randint(0, 2) - 1  # randomly -1 or 1
                toaI_ = 2*np.random.randint(0, 2) - 1
                setting_ = np.array([[Kp_, toaI_]])
                
            else:
                Kp_= 2*np.random.randint(0, 2) - 1  # randomly -1 or 1
                toaI_ = 2*np.random.randint(0, 2) - 1
                toaD_ = 2*np.random.randint(0, 2) - 1
                setting_ = np.array([[Kp_, toaI_, toaD_]])
            
            if 2 != np.max(np.matmul(self.pid_settings, setting_.T)):
                self.pid_settings = np.append(self.pid_settings, setting_, 
                                              axis=0)
                break
            
        # new element in the array to add error to
        self.anova_error = np.append(self.anova_error, np.zeros((1,1)), axis=0)
        # move the PID parameters
        actual_settings = self._encode2actual(setting_)
        #update pid settings -- this is readable not optimal
        if self.pi_control:
            self.Kp = actual_settings[0]
            self.toaI = actual_settings[1]

        else:

            self.Kp = actual_settings[0]
            self.toaI = actual_settings[1]
            self.toaD = actual_settings[2]
        return 0
    
    def anova_calc(self):
        r"""
        This calculates the direction the next PID center to be

        Returns
        -------
        None.

        """
        
        X = np.ones((len(self.pid_settings), 1))
        X = np.concatenate((X, self.pid_settings), axis=1)
        
        if self.pi_control:
            x12 = (self.pid_settings[:, 0]*self.pid_settings[:, 1])[:, np.newaxis]
            X = np.concatenate((X, x12), axis=1)
        else:
            x12 = (self.pid_settings[:, 0]*self.pid_settings[:, 1])[:, np.newaxis]
            x13 = (self.pid_settings[:, 0]*self.pid_settings[:, 2])[:, np.newaxis]
            x23 = (self.pid_settings[:, 1]*self.pid_settings[:, 2])[:, np.newaxis]
            x123 = (self.pid_settings[:, 0]*self.pid_settings[:, 1]*self.pid_settings[:, 2])[:, np.newaxis]
            
            X = np.concatenate((X, x12), axis=1)            
            X = np.concatenate((X, x13), axis=1)
            X = np.concatenate((X, x23), axis=1)
            X = np.concatenate((X, x123), axis=1)
        
        
        XTX = np.matmul(X.T, X)
        invXTX = np.linalg.inv(XTX)
        invXTX_XT = np.matmul(invXTX, X.T)
        a = np.matmul(invXTX_XT, self.anova_error)
        
        # maximum vector of descent at center point
        if self.pi_control:
            enc_gvec = -np.array([
                a[1], 
                a[2]
                ]) 
            mag = np.sqrt(np.matmul(enc_gvec.T, enc_gvec))
            enc_gvec = enc_gvec/mag
        else:
            enc_gvec = np.array([
                a[1], 
                a[2],
                a[3]
                ]) 
            mag = np.sqrt(enc_gvec.dot(enc_gvec))
            enc_gvec = enc_gvec/mag
        # TODO : center settings function, turn _cPID to PID and vice versa 
        
        mv_vec = self.grad_step * enc_gvec
        self.mv_pid_settings(mv_vec)
        
        self.Kp = self._cKp
        self.toaI = self._ctoaI
        self.toaD = self._ctoaD
        # clear the error log
        self.anova_error = np.zeros((1, 1))
        self.pid_settings = np.zeros((1, 2))

    def mv_pid_settings(self, enc_vec : np.array):
        r"""
        
        This method moves the PID settings from their current encoded positions 
        by vector addition of the encoded input vector
        
        Parameters
        ----------
        enc_vec : np.array
            Encoded addition vector to move the current encoded pid settings

        Returns
        -------
        None.

        """
        
        # hard coded
        
        
        if self.pi_control:
            self._cKp += enc_vec[0]*self.Kp_step
            self._ctoaI += enc_vec[1]*self.toaI_step
        else:
            self._cKp += enc_vec[0]*self.Kp_step
            self._ctoaI += enc_vec[1]*self.toaI_step
            self._ctoaD += enc_vec[2]*self.toaD_step
        
    def compute(self, error):
        r"""
        
        This method calculates the manipulated variable output

        Parameters
        ----------
        error : TYPE
            error between current reading and setpoint

        Returns
        -------
        None.

        """
        
        self.pi_acc+=error*self.dt
        
        if self.pi_control:
            # PI output calculation
            output = self.Kp*error + self.Kp*self.pi_acc/self.toaI
            
        else:
            # PID output calculation
            output = self.Kp*error + self.Kp*self.pi_acc/self.toaI \
                + self.Kp*self.toaD*(error - self.error_prev)/self.dt
            # store this error as the previous
            self.error_prev = error
        
        self.mv_count+=1
        
        self.anova_error[-1, 0] += self.mv_count*self.dt*abs(error)*self.dt
        if self.mv_count > self.mv_freq:
            _zero = self.mv_pid_DOE() 
            self.mv_count = 0
            self.pi_acc = 0
            
        return output
    
# %% Basic functions to move from encoded to actual and vice versa   
            
    def _actual2encode(self, actual_setting):
        r"""
        Converts the actual PID settings (Kp, toaI...) to encoded values
        (1, -1, ...)

        Parameters
        ----------
        actual_setting : np.array
            actual pid settings

        Returns
        -------
        encode_setting : np.array
            encoded pid settings

        """
        if self.pi_control:
            kp, toai = actual_setting
            _kp = (kp - self._cKp)/self.Kp_step
            _toai = (toai - self._ctoaI)/self.toaI_step
            encode_setting = np.array([_kp, _toai])
        else:
            kp, toai, toad = actual_setting
            _kp = (kp - self._cKp)/self.Kp_step
            _toai = (toai - self._ctoaI)/self.toaI_step
            _toad = (toad - self._ctoaD)/self.toaD_step
            encode_setting = np.array([_kp, _toai, _toad])
            
        return encode_setting
            
    def _encode2actual(self, encode_setting):
        r"""
        
        

        Parameters
        ----------
        encode_setting : TYPE
            DESCRIPTION.

        Returns
        -------
        actual_setting : TYPE
            DESCRIPTION.

        """
        
        if self.pi_control:
            _kp, _toaI = encode_setting[0, 0], encode_setting[0, 1]
            
            kp = _kp*self.Kp_step + self._cKp
            toai = _toaI*self.toaI_step + self._ctoaI
            actual_setting = np.array([kp, toai])
            
        else:
            _kp, _toaI, _toad = encode_setting[0, 0], encode_setting[0, 1], encode_setting[0, 2]
            kp = _kp*self.Kp_step + self._cKp
            toai = _toaI*self.toaI_step + self._ctoaI
            toad = _toad*self.toaD_step + self._ctoaD
            actual_setting = np.array([kp, toai, toad])
            
        return actual_setting
            
            
            
            
            
            
            
            
            
            
            
