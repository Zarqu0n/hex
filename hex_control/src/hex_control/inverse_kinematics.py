#! /usr/bin/env python3
from math import sin,cos,asin,acos,atan,sqrt,degrees,radians, pi
import os

class InverseKinematics:
        
    def __init__(self) -> None:
        self.coxa    = float(os.getenv("HEX_COXA_LENGTH"))
        self.femur   = float(os.getenv("HEX_FEMUR_LENGTH"))
        self.tibia   = float(os.getenv("HEX_TIBIA_LENGTH"))

        self.coxa_to_base = float(os.getenv("HEX_BASE_COXA_LENGTH"))


        self.offset_angles = [0, 60, 120, 180, 240, 300] 

    def coordinateToRad(self,x, y): # function to convert coordinates to angles from the x-axis (0~360)
        x += 0.00001 # this is to avoid zero division error in case x == 0
    
        if x >= 0 and y >= 0:   # first quadrant
            angle = atan(y/x)
        elif x < 0 and y >= 0:  # second quadrant
            angle = pi + atan(y/x)
        elif x < 0 and y < 0:   # third quadrant
            angle = pi + atan(y/x)
        elif x >= 0 and y < 0:  # forth quadrant
            angle = 2*pi + atan(y/x)
        return round(angle,1)



    def IK(self,pos):
    
        x, y, z = pos[0], pos[1], pos[2]
        x += 0.0000001 

        x = x - self.coxa_to_base
        
        theta1 = self.coordinateToRad(x, y)   

    
        x -= self.coxa*cos(theta1)
        y -= self.coxa*sin(theta1)
        
        if theta1 > pi: 
            theta1 -= 2*pi

        P = sqrt(x**2 + y**2)
            
        if sqrt(x**2 + y**2 + z**2) > self.femur + self.tibia: 
            print("MATH ERROR: coordinate too far")
            return [theta1, 0, 0]
        
        alpha = atan(z/P)

        c = sqrt(P**2 + z**2)
        
        beta = acos((self.femur**2+c**2-self.tibia**2)/(2*self.femur*c))
        theta2 = beta + alpha
        theta3 = acos((self.tibia**2+self.femur**2-c**2)/(2*self.tibia*self.femur)) - pi
        
        return [round(theta1,5), round((theta2),5), round((theta3),5)]

    def IK_leg(self,pos, LegID=1):
        x, y, z = pos[0], pos[1], pos[2]
        x += 0.00000001 # this is to avoid zero-division math error
#  <- put your original values; there is not limit on numbers of leg, but they should all be between 0 ~ 360 degrees

        theta1 = self.coordinateToRad(x, y)    # angle from x-axis in anticlockwise rotation

        # remove the offset due to the length of coxa    
        x -= self.coxa*cos(theta1)
        y -= self.coxa*sin(theta1)

        # subtract the offset angle from the x axis
        theta1 -= self.offset_angles[LegID-1] # theta1 = theta1 - offset_angles[LegID-1]
        
        if theta1 > pi: # keep the theta1 between -180 <= theta1 <= 180
            theta1 -= 2*pi

        P = sqrt(x**2 + y**2) # calculate length P
            
        if sqrt(x**2 + y**2 + z**2) > self.femur + self.tibia: # detect math error
            print("MATH ERROR: coordinate too far")
            return [theta1, 0, 0]
        
        alpha = atan(z/P) # calculate angle alpha
        c = sqrt(P**2 + z**2) # calculate length c
        
        beta = acos((self.femur**2+c**2-self.tibia**2)/(2*self.femur*c)) # calculate angle beta
        theta2 = beta + alpha # find theta2
        theta3 = acos((self.tibia**2+self.femur**2-c**2)/(2*self.tibia*self.femur)) - pi # find theta3
        
        return round(theta1,5), round((theta2),5), round((theta3),5)
