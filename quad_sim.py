#!/usr/bin/python

import quadcopter,gui,controller,arbiter
import signal
import sys, time
import argparse

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds
run = True
threads = []

def Single_Point2Point():
    # Set goals to go to
    GOALS = [(1,1,2),(1,-1,4),(-1,-1,2),(-1,1,4)]
    YAWS = [0,3.14,-1.54,1.54]
    # Define the quadcopters
    QUADCOPTER={'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2}
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }
    channels = 3
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = quadcopter.Quadcopter(QUADCOPTER)
    gui_object = gui.GUI(quad=QUADCOPTER)
    arb = arbiter.Arbiter(quad.get_time,quad.set_motor_speeds,channels,1)
    ctrls = []
    ctrl = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,lambda M: arb.feed(0, M),params=CONTROLLER_PARAMETERS)
    ctrls.append(ctrl)
    ctrl = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,lambda M: arb.feed(1, M),params=CONTROLLER_PARAMETERS)
    ctrls.append(ctrl)
    ctrl = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,lambda M: arb.feed(2, M),params=CONTROLLER_PARAMETERS)
    ctrls.append(ctrl)
    # for i in range(channels):
    #     ctrl = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,lambda M: arb.feed(i, M),params=CONTROLLER_PARAMETERS)
    #     ctrls.append(ctrl)
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    threads.append(quad)
    for i in range(channels):
        ctrls[i].start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        threads.append(ctrls[i])
    arb.start_thread()
    threads.append(arb)
    cursor = 0
    # Update the GUI while switching between destination poitions
    while(run==True):
        time.sleep(0)
        for goal,y in zip(GOALS,YAWS):
            for i in range(channels):
                ctrls[i].update_target(goal)
                ctrls[i].update_yaw_target(y)
            for i in range(100):
                if not run:
                    break
                gui_object.quad['position'] = quad.get_position()
                gui_object.quad['orientation'] = quad.get_orientation()
                gui_object.update()
            for i in range(channels):
                if i == cursor:
                    ctrls[i].halt = True
                else:
                    ctrls[i].halt = False

    for thread in threads:
        thread.stop_thread()
    for thread in threads:
        thread.thread_object.join()
    print('Stopped')
    sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def signal_handler(signal, frame):
    global run
    run = False
    print('Stopping')
    for thread in threads:
        thread.stop_thread()

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    Single_Point2Point()
