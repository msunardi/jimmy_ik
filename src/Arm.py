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

import math
import numpy as np
import scipy.optimize  

class Arm3Link:

    inv_out = []
    inv_default = [0.01, 0.01, 0.01]
    
    def __init__(self, q=None, q0=None, L=None):
        """Set up the basic parameters of the arm.
        All lists are in order [shoulder, elbow, wrist].
        
        :param list q: the initial joint angles of the arm
        :param list q0: the default (resting state) joint configuration
        :param list L: the arm segment lengths
        """
        # initial joint angles
        if q is None: q = [math.pi/4, math.pi/4, 0]
        self.q = q
        # some default arm positions
        if q0 is None: q0 = np.array([math.pi/4, math.pi/4, 0]) 
        self.q0 = q0
        # arm segment lengths
        if L is None: L = np.array([1, 1, 1]) 
        self.L = L
        
        # self.max_angles = [math.pi, math.pi, math.pi/4]
        self.max_angles = [math.pi, math.pi, math.pi]
        self.min_angles = [0, 0, -math.pi/4]

    def get_xy(self, q=None):
        """Returns the corresponding hand xy coordinates for 
        a given set of joint angle values [shoulder, elbow, wrist], 
        and the above defined arm segment lengths, L
        
        :param list q: the list of current joint angles
        :returns list: the [x,y] position of the arm
        """
        if q is None: q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, xy):
        """This is just a quick write up to find the inverse kinematics
        for a 3-link arm, using the SciPy optimize package minimization function.

        Given an (x,y) position of the hand, return a set of joint angles (q)
        using constraint based minimization, constraint is to match hand (x,y), 
        minimize the distance of each joint from it's default position (q0).
        
        :param list xy: a tuple of the desired xy position of the arm
        :returns list: the optimal [shoulder, elbow, wrist] angle configuration
        """

        def distance_to_default(q, *args): 
            """Objective function to minimize
            Calculates the euclidean distance through joint space to the default
            arm configuration. The weight list allows the penalty of each joint 
            being away from the resting position to be scaled differently, such
            that the arm tries to stay closer to resting state more for higher 
            weighted joints than those with a lower weight.
            
            :param list q: the list of current joint angles
            :returns scalar: euclidean distance to the default arm position
            """
            # weights found with trial and error, get some wrist bend, but not much
            weight = [1, 1, 1.3] 
            # weight = [0.5, 0.5, 0.7] 
            return np.sqrt(np.sum([(qi - q0i)**2 * wi
                for qi,q0i,wi in zip(q, self.q0, weight)]))

        def x_constraint(q, xy):
            """Returns the corresponding hand xy coordinates for 
            a given set of joint angle values [shoulder, elbow, wrist], 
            and the above defined arm segment lengths, L
            
            :param list q: the list of current joint angles
            :returns: the difference between current and desired x position
            """
            x = ( self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) + 
                self.L[2]*np.cos(np.sum(q)) ) - xy[0]
            return x

        def y_constraint(q, xy): 
            """Returns the corresponding hand xy coordinates for 
            a given set of joint angle values [shoulder, elbow, wrist], 
            and the above defined arm segment lengths, L
            
            :param list q: the list of current joint angles
            :returns: the difference between current and desired y position
            """
            y = ( self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) + 
                self.L[2]*np.sin(np.sum(q)) ) - xy[1]
            return y
        # print "XY: ", xy
        # xc = x_constraint(self.q, xy)
        # yc = y_constraint(self.q, xy)
        # print "x_constraint: ", xc
        # print "y_constraint: ", yc

        out = scipy.optimize.fmin_slsqp( func=distance_to_default, 
            x0=self.q0, eqcons=[x_constraint, y_constraint], 
            args=(xy,), iprint=1) # iprint=0 suppresses output
        # out, fx, its, lmode, smode = scipy.optimize.fmin_slsqp( func=distance_to_default, 
        #     x0=self.q, eqcons=[x_constraint, y_constraint], 
        #     args=(xy,), iprint=1, full_output=True, acc=0.2, epsilon=0.01) # iprint=0 suppresses output
        # out = np.array([x%1.8 if x >=0 else x%-1.8 for x in out])
        # print "OUT: %s (%s)" % (out, lmode)
        print "OUT: %s" % out
        # self.inv_out = out
        # if xc > 10 or xc < -10 or yc > 10 or yc < -10:
        #     if not self.inv_out.any():
        #         return self.inv_default
        #     return self.inv_out

        return out
        # out, fx, its, lmode, smode = scipy.optimize.fmin_slsqp( func=distance_to_default,
        #     x0=self.q, eqcons=[x_constraint, y_constraint], args=(xy,)) # iprint=0 suppresses output
        # print fx, its, lmode, smode

        # return out

def test():
    ############Test it!##################

    arm = Arm3Link()

    # set of desired (x,y) hand positions
    x = np.arange(-.75, .75, .05)
    y = np.arange(0, .75, .05)

    # threshold for printing out information, to find trouble spots
    thresh = .025

    count = 0
    total_error = 0
    # test it across the range of specified x and y values
    for xi in range(len(x)):
        for yi in range(len(y)):
            # test the inv_kin function on a range of different targets
            xy = [x[xi], y[yi]]
            # run the inv_kin function, get the optimal joint angles
            q = arm.inv_kin(xy=xy)
            # find the (x,y) position of the hand given these angles
            actual_xy = arm.get_xy(q)
            # calculate the root squared error
            error = np.sqrt((np.array(xy) - np.array(actual_xy))**2)
            # total the error 
            total_error += error
            
            # if the error was high, print out more information
            if np.sum(error) > thresh or True:
                print '-------------------------'
                print 'Initial joint angles', arm.q 
                print 'Final joint angles: ', q
                print 'Desired hand position: ', xy
                print 'Actual hand position: ', actual_xy
                print 'Error: ', error
                print '-------------------------'
            
            count += 1

    print '\n---------Results---------'
    print 'Total number of trials: ', count
    print 'Total error: ', total_error
    print '-------------------------'
