import numpy as np
import math
import scipy.integrate
import time
import datetime
import threading

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0

    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972

class Quadcopter():
    # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self,quad,gravity=9.81,b=0.0245):
        self.quad = quad
        self.g = gravity
        self.b = b
        self.thread_object = None
        self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
        self.time = datetime.datetime.now()
        self.quad['state'] = np.zeros(12)
        self.quad['state'][0:3] = self.quad['position']
        self.quad['state'][6:9] = self.quad['orientation']
        self.quad['m1'] = Propeller(self.quad['prop_size'][0],self.quad['prop_size'][1])
        self.quad['m2'] = Propeller(self.quad['prop_size'][0],self.quad['prop_size'][1])
        self.quad['m3'] = Propeller(self.quad['prop_size'][0],self.quad['prop_size'][1])
        self.quad['m4'] = Propeller(self.quad['prop_size'][0],self.quad['prop_size'][1])
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx=((2*self.quad['weight']*self.quad['r']**2)/5)+(2*self.quad['weight']*self.quad['L']**2)
        iyy=ixx
        izz=((2*self.quad['weight']*self.quad['r']**2)/5)+(4*self.quad['weight']*self.quad['L']**2)
        self.quad['I'] = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.quad['invI'] = np.linalg.inv(self.quad['I'])
        self.run = True

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def state_dot(self, time, state):
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = self.quad['state'][3]
        state_dot[1] = self.quad['state'][4]
        state_dot[2] = self.quad['state'][5]
        # The acceleration
        x_dotdot = np.array([0,0,-self.quad['weight']*self.g]) + np.dot(self.rotation_matrix(self.quad['state'][6:9]),np.array([0,0,(self.quad['m1'].thrust + self.quad['m2'].thrust + self.quad['m3'].thrust + self.quad['m4'].thrust)]))/self.quad['weight']
        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]
        # The angular rates(t+1 theta_dots equal the t theta_dots)
        state_dot[6] = self.quad['state'][9]
        state_dot[7] = self.quad['state'][10]
        state_dot[8] = self.quad['state'][11]
        # The angular accelerations
        omega = self.quad['state'][9:12]
        tau = np.array([self.quad['L']*(self.quad['m1'].thrust-self.quad['m3'].thrust), self.quad['L']*(self.quad['m2'].thrust-self.quad['m4'].thrust), self.b*(self.quad['m1'].thrust-self.quad['m2'].thrust+self.quad['m3'].thrust-self.quad['m4'].thrust)])
        omega_dot = np.dot(self.quad['invI'], (tau - np.cross(omega, np.dot(self.quad['I'],omega))))
        state_dot[9] = omega_dot[0]
        state_dot[10] = omega_dot[1]
        state_dot[11] = omega_dot[2]
        return state_dot

    def update(self, dt):
        self.ode.set_initial_value(self.quad['state'],0).set_f_params()
        self.quad['state'] = self.ode.integrate(self.ode.t + dt)
        self.quad['state'][6:9] = self.wrap_angle(self.quad['state'][6:9])
        self.quad['state'][2] = max(0,self.quad['state'][2])

    def set_motor_speeds(self,speeds):
        if speeds is None:
            return
        self.quad['m1'].set_speed(speeds[0])
        self.quad['m2'].set_speed(speeds[1])
        self.quad['m3'].set_speed(speeds[2])
        self.quad['m4'].set_speed(speeds[3])

    def get_position(self):
        return self.quad['state'][0:3]

    def get_linear_rate(self):
        return self.quad['state'][3:6]

    def get_orientation(self):
        return self.quad['state'][6:9]

    def get_angular_rate(self):
        return self.quad['state'][9:12]

    def get_state(self):
        return self.quad['state']

    def set_position(self,position):
        self.quad['state'][0:3] = position

    def set_orientation(self,orientation):
        self.quad['state'][6:9] = orientation

    def get_time(self):
        return self.time

    def thread_run(self,dt,time_scaling):
        rate = time_scaling*dt
        last_update = self.time
        while(self.run==True):
            time.sleep(0)
            self.time = datetime.datetime.now()
            if (self.time-last_update).total_seconds() > rate:
                self.update(dt)
                last_update = self.time

    def start_thread(self,dt=0.002,time_scaling=1):
        self.thread_object = threading.Thread(target=self.thread_run,args=(dt,time_scaling))
        self.thread_object.start()

    def stop_thread(self):
        self.run = False
