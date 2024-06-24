import math

class PID_Controller(object):
    def __init__(self, Kp, Ki, Kd):
        # PID 參數
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # 狀態變數
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
    def tune(self,KpNew,KiNew,KdNew):
        self.Kp = KpNew
        self.Ki = KiNew
        self.Kd = KdNew
    def getCorrection(self, target, actual, dt=1):
        E = target - actual
        # dE/dt
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        self.Stdt += E*dt if self.t > 0 else 0 # (E + self.Eprev)*dt if self.t > 0 else 0
        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
        self.t += 1
        self.Eprev = E
        return correction

#PID demand循跡控制 
class Demand_PID_Controller(PID_Controller):
    def __init__(self, Kp, Kd, Ki=0, demand_noise_threshold=.01):
        PID_Controller.__init__(self, Kp, Ki, Kd)
        self.noise_threshold = demand_noise_threshold

        self.prevAbsDemand = 1      
        self.target        = None

        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
 
    def getCorrection(self, sensorValue, demandValue, timestep=1):
        correction = 0
        if abs(demandValue) < self.noise_threshold:
            if self.prevAbsDemand > self.noise_threshold:
                self.target = sensorValue
            correction = PID_Controller.getCorrection(self, self.target, sensorValue, timestep)
        self.prevAbsDemand = abs(demandValue)                                  
        return correction

class GPS_PID_Controller(PID_Controller):   
    def __init__(self, Kp, Kd, Ki=0):
        PID_Controller.__init__(self, Kp, Ki, Kd)
        self.noise_threshold = 0.01
        self.prevAbsDemand = 1 
        self.target        = None
        self.Eprev = 0.0
        self.Stdt = 0.0
        self.t = 0.0

    def getCorrection(self, targetValue, sensorValue, dt=1):
        correction = 0   
        E = targetValue - sensorValue
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        self.Stdt += E*dt if self.t > 0 else 0# (E + self.Eprev)*dt if self.t > 0 else 0
        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
        self.t += 1
        self.Eprev = E       
        return correction if abs(correction) < 1 else correction*1/abs(correction)

class Stability_PID_Controller(PID_Controller):
    def __init__(self, Kp, Kd, Ki=0):  
        PID_Controller.__init__(self, Kp, Ki, Kd)      
    def getCorrection(self, actualAngle, timestep=1):
        return PID_Controller.getCorrection(self, 0, actualAngle, timestep)

class Yaw_PID_Controller(Demand_PID_Controller):
    def __init__(self, Kp, Kd, Ki,  demand_noise_threshold=.01):
        Demand_PID_Controller.__init__(self, Kp, Kd, Ki, demand_noise_threshold)
        
    def getCorrection(self, yawAngle, yawDemand, timestep=1):
        correction =  Demand_PID_Controller.getCorrection(self, -yawAngle, yawDemand, timestep)
        return correction if abs(correction) < 10 else 0 

class Hover_PID_Controller(PID_Controller):
    def __init__(self, Kp, Kd=0, Ki=0, max_correction = 0.5):
        PID_Controller.__init__(self, Kp, Ki, Kd)
        self.position_prev = None
        self.max_correction = max_correction
        
    def getCorrection(self, position, target=None, timestep=1):
        velocity = 0
        correction = 0
        if self.position_prev:
            velocity = (position - self.position_prev) / timestep
        self.position_prev = position
        correction = PID_Controller.getCorrection(self, target, position, timestep) \
                     if target \
                     else PID_Controller.getCorrection(self, 0, velocity, timestep) \
                     
        return min(max(correction, -self.max_correction), +self.max_correction)
