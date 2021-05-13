#!/usr/bin/env python

import numpy as np
from math import pi, sin, cos, radians, sqrt

from agents.navigation.local_planner import RoadOption
from agents.navigation.local_waypoint import LocalWaypoint
from agents.tools.misc import get_poly_y
from math import *


# This class generate bezier curve for left and right turns

class BezierTurn():

    def __init__(self, world, right_flag=True):
        self._world_obj = world
        self._map = self._world_obj.world.get_map()
        self.right_flag = right_flag

    def get_waypoints(self, ref1, ref2):
        
        total_points = 50
        t = np.linspace(0, 1, total_points)
        lane_change_plan=[]

        ################################## Parameters that you can change ###########################################
        param_x = 0.25                                    
        param_y = 0.25
        #################################### for different turn curves ##############################################
        
        x0 = ref1[0]
        y0 = ref1[1]
        x2 = ref2[0]
        y2 = ref2[1]
        h0 = tan(np.deg2rad(ref1[2]))
        hf = tan(np.deg2rad(ref2[2]))

        if (((x2-x0)>0 and (y2-y0)>0) or ((x2-x0)<0 and (y2-y0)<0)) and self.right_flag:
            x1 = ((1-param_x)*x0 + param_x*x2)
            y1 = ( y0*param_y + y2*(1-param_y))   

        if (((x2-x0)<0 and (y2-y0)>0) or ((x2-x0)>0 and (y2-y0)<0)) and self.right_flag:
            x1 = ((1-param_x)*x2 + param_x*x0)
            y1 = ( y2*param_y + y0*(1-param_y))   

        if (((x2-x0)<0 and (y2-y0)>0) or ((x2-x0)>0 and (y2-y0)<0)) and (not self.right_flag):
            
            x1 = ((1-param_x)*x0 + param_x*x2)
            y1 = ( y0*param_y + y2*(1-param_y))   

        if (((x2-x0)>0 and (y2-y0)>0) or ((x2-x0)<0 and (y2-y0)<0)) and (not self.right_flag):
            print(self.right_flag)
            x1 = ((1-param_x)*x2 + param_x*x0)
            y1 = ( y2*param_y + y0*(1-param_y))   

        x_temp = x1
        y_temp = y1  

        # x1 = -(y0 - y2 - h0*x0 + hf*x2)/(h0 - hf)
        # y1 = (h0*y2 - hf*y0 + h0*hf*x0 - h0*hf*x2)/(h0 - hf)
  
        if (abs(x1-x_temp) + abs(y1-y_temp)) > 15:
            x1 = x_temp
            y1 = y_temp  

        wp_x = ((1-t)**2)*x0 +  2*t*(1-t)*x1 + (t**2)*x2
        wp_y = ((1-t)**2)*y0 +  2*t*(1-t)*y1 + (t**2)*y2

        dx = 2*(-x0*(1-t)-2*x1*t + x1 + x2*t)
        dy = 2*(-y0*(1-t)-2*y1*t + y1 + y2*t)
        wp_yaw = np.rad2deg(np.arctan2(dy,dx))

        coord = np.matmul(np.identity(3), np.stack((wp_x, wp_y, wp_yaw)))
        for i in range(total_points):
            waypoint = LocalWaypoint(coord[0][i], coord[1][i], self._world_obj.player.get_location().z, coord[2][i])
            lane_change_plan.append((waypoint, RoadOption.RIGHT))

        return lane_change_plan
        

class BezierOverTake():

    def __init__(self, world, right_flag=True):
        self._world_obj = world
        self._map = self._world_obj.world.get_map()
        self.right_flag = right_flag

    def get_waypoints(self, ref1):
        
        total_points = 50
        t = np.linspace(0, 1, total_points)
        lane_change_plan=[]
        param_x = 1
        param_y = 1
        x0 = ref1[0]
        y0 = ref1[1]
        x2 = x0 + 50
        y2 = y0 - 3.2
        h0 = tan(np.deg2rad(ref1[2]-10))
        hf = tan(np.deg2rad(ref1[2]))

        r =  sqrt((x0-x2)**2 + (y0-y2)**2)
        theta = atan2((y2-y0), (x2-x0))

        if (((x2-x0)>0 and (y2-y0)>0) or ((x2-x0)<0 and (y2-y0)<0)) and self.right_flag:
            x1 = ((1-param_x)*x0 + param_x*x2)
            y1 = ( y0*param_y + y2*(1-param_y))   

        if (((x2-x0)<0 and (y2-y0)>0) or ((x2-x0)>0 and (y2-y0)<0)) and self.right_flag:
            x1 = ((1-param_x)*x2 + param_x*x0)
            y1 = ( y2*param_y + y0*(1-param_y))   

        if (((x2-x0)<0 and (y2-y0)>0) or ((x2-x0)>0 and (y2-y0)<0)) and (not self.right_flag):
            x1 = ((1-param_x)*x0 + param_x*x2)
            y1 = ( y0*param_y + y2*(1-param_y))   

        if (((x2-x0)>0 and (y2-y0)>0) or ((x2-x0)<0 and (y2-y0)<0)) and (not self.right_flag):
            x1 = ((1-param_x)*x2 + param_x*x0)
            y1 = ( y2*param_y + y0*(1-param_y))   

        x_temp = x1
        y_temp = y1  

        # x1 = -(y0 - y2 - h0*x0 + hf*x2)/(h0 - hf)
        # y1 = (h0*y2 - hf*y0 + h0*hf*x0 - h0*hf*x2)/(h0 - hf) 

        if (abs(x1-x_temp) + abs(y1-y_temp)) > 15:
            x1 = x_temp
            y1 = y_temp  

        wp_x = ((1-t)**2)*x0 +  2*t*(1-t)*x1 + (t**2)*x2
        wp_y = ((1-t)**2)*y0 +  2*t*(1-t)*y1 + (t**2)*y2

        dx = 2*(-x0*(1-t)-2*x1*t + x1 + x2*t)
        dy = 2*(-y0*(1-t)-2*y1*t + y1 + y2*t)
        wp_yaw = np.rad2deg(np.arctan2(dy,dx))

        ############################################ Second curve ################################################

        x0 = wp_x[-1]
        y0 = wp_y[-1]
        x2 = x0 + 30
        y2 = y0 + 4.4
        h0 = tan(np.deg2rad(ref1[2]-10))
        hf = tan(np.deg2rad(ref1[2]))
        x1 = x0 + 10
        y1 = y2

        wp_x_new = ((1-t)**2)*x0 +  2*t*(1-t)*x1 + (t**2)*x2
        wp_y_new = ((1-t)**2)*y0 +  2*t*(1-t)*y1 + (t**2)*y2

        dx = 2*(-x0*(1-t)-2*x1*t + x1 + x2*t)
        dy = 2*(-y0*(1-t)-2*y1*t + y1 + y2*t)
        wp_yaw_new = np.rad2deg(np.arctan2(dy,dx))

        wp_x = np.concatenate((wp_x, wp_x_new))
        wp_y = np.concatenate((wp_y, wp_y_new))
        wp_yaw = np.concatenate((wp_yaw, wp_yaw_new))
        print(wp_x.shape)
        coord = np.matmul(np.identity(3), np.stack((wp_x, wp_y, wp_yaw)))
        for i in range(total_points*2):
            waypoint = LocalWaypoint(coord[0][i], coord[1][i], self._world_obj.player.get_location().z, coord[2][i])
            lane_change_plan.append((waypoint, RoadOption.RIGHT))

        return lane_change_plan