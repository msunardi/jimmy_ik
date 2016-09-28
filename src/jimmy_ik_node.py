#!/usr/bin/env python
'''
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
import sys, threading
import numpy as np
import pyglet
import time
import serial, math

import rospy, rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String, Bool

import Arm

from arbotix_msgs.srv import *

# def resetServos():
#     for i in range(4):
#         pol.set_acceleration(i, 3)
#         pol.set_speed(i, 0)
#         pol.set_target(i, 1500)

# def plot(): 
#     """A function for plotting an arm, and having it calculate the 
#     inverse kinematics such that given the mouse (x, y) position it 
#     finds the appropriate joint angles to reach that point."""

#     # create an instance of the arm
#     arm = Arm.Arm3Link(L = np.array([300,200,100]))

#     # make our window for drawin'
#     window = pyglet.window.Window(width=640, height=640)

#     label = pyglet.text.Label('Mouse (x,y)', font_name='Times New Roman', 
#         font_size=36, x=window.width//2, y=window.height//2,
#         anchor_x='center', anchor_y='center')

#     def get_joint_positions():
#         """This method finds the (x,y) coordinates of each joint"""

#         x = np.array([ 0, 
#             arm.L[0]*np.cos(arm.q[0]),
#             arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]),
#             arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]) + 
#                 arm.L[2]*np.cos(np.sum(arm.q)) ]) + window.width/2

#         y = np.array([ 0, 
#             arm.L[0]*np.sin(arm.q[0]),
#             arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]),
#             arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]) + 
#                 arm.L[2]*np.sin(np.sum(arm.q)) ])

#         return np.array([x, y]).astype('int')
    
#     window.jps = get_joint_positions()

#     @window.event
#     def on_draw():
#         window.clear()
#         label.draw()
#         for i in range(3): 
#             pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
#                 (window.jps[0][i], window.jps[1][i], 
#                  window.jps[0][i+1], window.jps[1][i+1])))

#     @window.event
#     def on_mouse_motion(x, y, dx, dy):
#     #def on_mouse_release(x, y, button, modifiers):
#         # call the inverse kinematics function of the arm
#         # to find the joint angles optimal for pointing at 
#         # this position of the mouse 
#         label.text = '(x,y) = (%.3f, %.3f)'%(x,y)
#         # arm.q = arm.inv_kin([x - window.width/2, y]) # get new arm angles
#         arm.q = [no_nan(x) for x in arm.inv_kin([x - window.width/2, y])] # get new arm angles
#         window.jps = get_joint_positions() # get new joint (x,y) positions
#         print arm.q
#         # pos = [rad_to_pololu(q) for q in arm.q]
#         # pol.set_multiple(3, 0, pos[0], pos[1], pos[2])
#         # pol.serial.flush()

#     def no_nan(x):
#         if math.isnan(x):
#             return 0.001
#         return x

#     def rad_to_pololu(rad):
#         # pololu range 1050 - 2050
#         pmax = 2000
#         pmin = 1000
#         pos = pmin + (rad/math.pi * pmax)
#         if pos < 0:
#             pos = abs(pos)
#         if pos < pmin:
#             pos = pmin
#         elif pos > pmax:
#             pos = pmax
#         return int(pos)

#     pyglet.app.run()

# ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

# resetServos()
# plot()

class JimmyIk(threading.Thread):
    def __init__(self):
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=10)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=10)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=10)

        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        self.left_elbow_enable = rospy.ServiceProxy(
            '/left_elbow/enable', Enable)
        self.left_sho_pitch_enable = rospy.ServiceProxy(
            '/left_sho_pitch/enable', Enable)
        self.left_sho_roll_enable = rospy.ServiceProxy(
            '/left_sho_roll/enable', Enable)

        self.joints = {'L_ELBOW': self.left_elbow,
            'L_SHO_PITCH': self.left_sho_pitch,
            'L_SHO_ROLL': self.left_sho_roll}

        self.joint_pos = {'L_ELBOW': 512,
            'L_SHO_PITCH': 512,
            'L_SHO_ROLL': 512}

        self.init_ready = [False]*8

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(50)
        self.plot()

    def joint_callback(self, data):        
        # rospy.loginfo("Positions: %s" % position)
        # self.ready = False
        # self.enable_ready = True
        
        names = data.name
        # position = [self.pospos(x) for x in data.position]
        position = data.position
        fubar = zip(names, position) # create pairs            
        rospy.loginfo("data: %s" % fubar)

        for joint in fubar:
            # rospy.loginfo(joint)
            joint_name = joint[0]
            joint_pos = self.pospos(joint[1])
            if joint_name == 'left_sho_roll':
                self.joint_pos['L_SHO_ROLL'] = joint_pos
                self.init_ready[0] = True
                rospy.loginfo(">>>>> L_SHO_ROLL (%s)" % joint_pos)
            if joint_name == 'left_sho_pitch':
                self.joint_pos['L_SHO_PITCH'] = joint_pos
                self.init_ready[1] = True
                rospy.loginfo(">>>>> L_SHO_PITCH (%s)" % joint_pos)
            if joint_name == 'left_elbow':
                self.joint_pos['L_ELBOW'] = joint_pos
                self.init_ready[2] = True
                rospy.loginfo(">>>>> L_ELBOW (%s)" % joint_pos)
            
    def pospos(self, x):
        # Convert joint_pos from -2.6 - 2.6 to 0 - 1020
        return min(1020, max(0, int((x + 2.6)/2.6 * 512)))

    def pos_to_rad(self, x):
        return (x - 510)/510.0 * 2.6

    def run(self):
        self.plot()
        while not rospy.is_shutdown():
            self.sleeper.sleep()

    def plot(self): 
        """A function for plotting an arm, and having it calculate the 
        inverse kinematics such that given the mouse (x, y) position it 
        finds the appropriate joint angles to reach that point."""

        # create an instance of the arm
        q_init = [self.joint_pos['L_SHO_PITCH'], self.joint_pos['L_SHO_ROLL'], self.joint_pos['L_ELBOW']]
        q_init = [self.pos_to_rad(x) for x in q_init]
        # q_init = [0.0, 0.0, 0.0]
        arm = Arm.Arm3Link(L = np.array([200,200,100]), q=q_init)

        # make our window for drawin'
        window = pyglet.window.Window(width=1280, height=640)

        label = pyglet.text.Label('Mouse (x,y)', font_name='Times New Roman', 
            font_size=20, x=window.width//2, y=window.height//2,
            anchor_x='center', anchor_y='center')

        # joint_label = pyglet.text.Label('Joint angles: (a, b, c)', font_name='Times New Roman', 
        #     font_size=12, x=window.width//2, y=window.height//2 + 40,
        #     anchor_x='center', anchor_y='center')

        def get_joint_positions():
            """This method finds the (x,y) coordinates of each joint"""

            x = np.array([ 0, 
                arm.L[0]*np.cos(arm.q[0]),
                arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]),
                arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]) + 
                    arm.L[2]*np.cos(np.sum(arm.q)) ]) + window.width/2

            y = np.array([ 0, 
                arm.L[0]*np.sin(arm.q[0]),
                arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]),
                arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]) + 
                    arm.L[2]*np.sin(np.sum(arm.q)) ]) + window.height/2

            return np.array([x, y]).astype('int')
        
        window.jps = get_joint_positions()

        @window.event
        def on_draw():
            window.clear()
            label.draw()
            for i in range(3): 
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
                    (window.jps[0][i], window.jps[1][i], 
                     window.jps[0][i+1], window.jps[1][i+1])))

        @window.event
        # def on_mouse_release(x, y, dx, dy):
        def on_mouse_drag(x, y, dx, dy, button, modifiers):
            arm.q = [no_nan(x) for x in arm.inv_kin([x - window.width/2, y])] # get new arm angles
            window.jps = get_joint_positions() # get new joint (x,y) positions
            print arm.q
            label.text = "(a, b, c) =(%.3f, %.3f, %.3f)" % (arm.q[0], arm.q[1], arm.q[2])

            self.left_sho_pitch.publish(Float64(arm.q[0]))
            self.left_elbow.publish(Float64(arm.q[2]))

        # @window.event
        # def on_mouse_motion(x, y, dx, dy):
        # # def on_mouse_release(x, y, button, modifiers):
        #     # call the inverse kinematics function of the arm
        #     # to find the joint angles optimal for pointing at 
        #     # this position of the mouse 
        #     label.text = '(x,y) = (%.3f, %.3f)'%(x,y)
        #     # arm.q = arm.inv_kin([x - window.width/2, y]) # get new arm angles
        #     arm.q = [no_nan(x) for x in arm.inv_kin([x - window.width/2, y])] # get new arm angles
        #     window.jps = get_joint_positions() # get new joint (x,y) positions
        #     print arm.q
        #     # label.text = "(a, b, c) =(%.3f, %.3f, %.3f)" % (arm.q[0], arm.q[1], arm.q[2])

        #     self.left_sho_pitch.publish(Float64(arm.q[0]))
        #     self.left_elbow.publish(Float64(arm.q[2]))
            # if arm.q[1] < 0:
            #     self.left_sho_pitch.publish(Float64(max(arm.q[0], -2.6)))
            # else:
            #     self.left_sho_pitch.publish(Float64(min(arm.q[0], 2.6)))
            # if arm.q[2] < 0:
            #     self.left_elbow.publish(Float64(max(arm.q[1], -1.8)))
            # else:
            #     self.left_elbow.publish(Float64(min(arm.q[1], 1.8)))
            # pos = [rad_to_pololu(q) for q in arm.q]
            # pol.set_multiple(3, 0, pos[0], pos[1], pos[2])
            # pol.serial.flush()

        def no_nan(x):
            if math.isnan(x):
                return 0.01
            return x

        def rad_to_pololu(rad):
            # pololu range 1050 - 2050
            pmax = 2000
            pmin = 1000
            pos = pmin + (rad/math.pi * pmax)
            if pos < 0:
                pos = abs(pos)
            if pos < pmin:
                pos = pmin
            elif pos > pmax:
                pos = pmax
            return int(pos)

        pyglet.app.run()

def main(args):
    rospy.init_node('jimmy_ik_node', anonymous=True)
    # plot()
    jimmy_ik = JimmyIk()
    jimmy_ik.start()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
