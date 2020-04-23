
import numpy as np

class Frame(object):
    def __init__(self,x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
        
        self.x = x
        self.y = y
        self.z = z

        self._pitch = pitch
        self._roll = roll
        self._yaw = yaw

        self.color = 'k'

    def R(self):
        sr = np.sin(self._roll)
        sp = np.sin(self._pitch)
        sy = np.sin(self._yaw)

        cr = np.cos(self._roll)
        cp = np.cos(self._pitch)
        cy = np.cos(self._yaw)

        R = np.array([[cr*cy-cp*sr*sy,-cy*sr-cr*cp*sy, sp*sy],
                     [cp*cy*sr+cr*sy, cr*cp*cy-sr*sy,-cy*sp],
                     [sr*sp,cr*sp,cp]])

        return R


    def show(self,ax):

        R = self.R()

        P = np.array([self.x,self.y,self.z])

        i = np.matmul(R,np.array([1,0,0]))+P
        j = np.matmul(R,np.array([0,1,0]))+P
        k = np.matmul(R,np.array([0,0,1]))+P

        i_hat = np.vstack((i,P)).T
        j_hat = np.vstack((j,P)).T
        k_hat = np.vstack((k,P)).T

        ax.plot(i_hat[0],i_hat[1],i_hat[2],color=self.color)
        ax.plot(j_hat[0],j_hat[1],j_hat[2],color=self.color)
        ax.plot(k_hat[0],k_hat[1],k_hat[2],color=self.color)

    
    def show_vec(self,vector,ax):
        R = self.R()
        P = np.array([self.x,self.y,self.z])
        v = np.matmul(R,vector)+P
        vp = np.vstack((v,P)).T
        ax.plot(vp[0],vp[1],vp[2],color='b')


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    frame = Frame()
    frame2 = Frame()
    frame2.color = 'r'

    
    frame2.x = 10
    frame2.y = 10
    frame2.z = 5
    frame2._pitch = np.deg2rad(45)

    frame.show(ax)
    v1 = np.array([3,0,0])
    v2 = np.array([-3,0,0])
    v3 = np.array([0,3,0])
    v4 = np.array([0,-3,0])


    for idx in range(100):
        plt.cla()
        ax.set_xlim3d(0, 15)
        ax.set_ylim3d(0,15)
        ax.set_zlim3d(0,15)
        frame2.x = idx/10
        frame2._pitch = np.deg2rad(idx)
        frame2._yaw = frame2._pitch
        frame2.show(ax)


        for v in [v1,v2,v3,v4]:
            frame2.show_vec(v)
        
        plt.pause(0.001)

    
    frame2.show(ax)


    plt.show()