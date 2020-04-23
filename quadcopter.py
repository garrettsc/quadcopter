
import numpy as np

class Quadcopter(object):
    #phi - roll
    #theta - pitch
    #psi - yaw

    def __init__(self):


        self.x = np.array([[0],[0],[0]])
        self.xdot = np.array([[0],[0],[0]])

        self.THETA = np.array([[0],[0],[0]]) #Roll, pitch, yaw
        self.THETA_DOT = np.array([[0],[0],[0]]) #Roll rate, pitch rate, yaw rate

        self.omega = np.array([[0],[0],[0]]) #angular velocity
        

        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        self.dt = 0.01

        self.g = np.array([[0],[0],[-9.81]])
        self.m = 1
        self.kd = 1 #drag coefficient
        self.kt = 1 #thrust coefficient
        self.km = 1 #Motor/prop coefficient (contains kv,ktau, kt)
        self.I = np.zeros((3,3))
        self.I[0,0] = 1
        self.I[1,1] = 1
        self.I[2,2] = 1
        self.L = 1 #moment arm length, i.e arm length
        self.b = 1 #aerodynamic constant models propeller properties related to drag

        self.W = np.array([10,10,10,10]) #propeller angular velocities

    
    def R(self):
        sr = np.sin(self.roll)
        sp = np.sin(self.pitch)
        sy = np.sin(self.yaw)

        cr = np.cos(self.roll)
        cp = np.cos(self.pitch)
        cy = np.cos(self.yaw)

        R = np.array([[cr*cy-cp*sr*sy,-cy*sr-cr*cp*sy, sp*sy],
                     [cp*cy*sr+cr*sy, cr*cp*cy-sr*sy,-cy*sp],
                     [sr*sp,cr*sp,cp]])

        return R
        
    def acceleration(self):

        Fd = -self.kd * self.xdot
        Tb = self.kt*np.array([[0],[0],[self.W.sum()]])
        xdotdot = self.g + 1/self.m * (np.dot(self.R(),Tb) + Fd)

        return xdotdot

    def acceleration_angular(self):

        tau_phi = self.L*self.km*(self.w1**2-self.w3**2)
        tau_theta = self.L*self.km*(self.w4**2-self.w2**2)
        tau_psi = self.b*(self.w1**2-self.w2**2+self.w3**2-self.w4**2)

        Ixx = self.I[0,0]
        Iyy = self.I[1,1]
        Izz = self.I[2,2]

        Ixx_i = 1/Ixx
        Iyy_i = 1/Iyy
        Izz_i = 1/Izz


        c1 = np.array([[tau_phi*Ixx_i],
                    [tau_theta*Iyy_i],
                    [tau_psi*Izz_i]])

        c2 = np.zeros((3,1))

        c2[0] = (Iyy-Izz)/Ixx * (self.wy*self.wz)
        c2[1] = (Izz-Ixx)/Iyy * (self.wx*self.wz)
        c2[2] = (Ixx-Iyy)/Izz * (self.wx*self.wy)

        return c1+c2

    def update(self):


        omega = self.thetadot2omega() #Map the current rpy rates to angular acceleration
        omega_dot = self.acceleration_angular() #Compute angular acceleration based on model

        
        self.omega = omega + omega_dot*self.dt #Compute updated angular velocity
        self.THETA_DOT = self.omega2thetadot() #Map back to rpy rates
        self.THETA = self.THETA+self.THETA_DOT*self.dt #Use rpy rates to compute new Euler angles

        a = self.acceleration() #Compute translational acceleration based on model
        self.xdot = self.xdot+a*self.dt
        self.x = self.x+self.xdot*self.dt

        self.roll = self.THETA[0][0]
        self.pitch = self.THETA[1][0]
        self.yaw = self.THETA[2][0]


    def thetadot2omega(self):

        T = np.array([[1,0,-np.sin(self.pitch)],
                     [0,np.cos(self.roll),np.cos(self.pitch)*np.sin(self.roll)],
                     [0,-np.sin(self.roll),np.cos(self.pitch)*np.cos(self.roll)]])

        return np.dot(T,self.THETA_DOT)

    
    def omega2thetadot(self):
        T = np.array([[1,0,-np.sin(self.pitch)],
                     [0,np.cos(self.roll),np.cos(self.pitch)*np.sin(self.roll)],
                     [0,-np.sin(self.roll),np.cos(self.pitch)*np.cos(self.roll)]])

        Tinv = np.linalg.inv(T)
        return np.dot(T,self.omega)


    
    @property
    def wx(self):
        return self.omega[0]
    
    @property
    def wy(self):
        return self.omega[1]

    @property
    def wz(self):
        return self.omega[2]


    @property
    def w1(self):
        return self.W[0]
    @property
    def w2(self):
        return self.W[1]

    @property
    def w3(self):
        return self.W[2]
    @property
    def w4(self):
        return self.W[3]


if __name__ == '__main__':

    q = Quadcopter()
    print(q.acceleration())

    print(q.acceleration_angular())
