# import packages

import rtde_control
import rtde_receive
import numpy as np
import gc

IP_ADDRESS = "172.28.60.3"
SPEED = 0.5
ACCELERATION = 1.2


class UrHandler:
    """UR Handler."""

    def __init__(self):
        self.receive_handler = rtde_receive.RTDEReceiveInterface(IP_ADDRESS)
        self.control_handler = rtde_control.RTDEControlInterface(IP_ADDRESS)
        self.orienatations = [[np.pi, 0, 0],
                              [2.261, -2.182, 0],
                              [0, np.pi, 0],
                              [2.182, 2.261, 0]]
    def get_position(self):
        x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = self.receive_handler.getActualTCPPose()

        return x_m, y_m, z_m, rx_rad, ry_rad, rz_rad

    def move_to_position(self, x_m, y_m, z_m, rx_rad, ry_rad, rz_rad):
        self.control_handler.moveL([x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], SPEED, ACCELERATION)

    def move_to_xyz_position(self, x_m, y_m, z_m):
        _, _, _, rx_rad, ry_rad, rz_rad = self.get_position()
        self.move_to_position(x_m, y_m, z_m, rx_rad, ry_rad, rz_rad)
        
    def move_to_corner(self, corner):
        self.move_to_xyz_position(corner.x, corner.y, corner.z)
        
    def orient_to_corner(self, orientation):
        x, y, z, _, _, _ = self.get_position()
        
        self.move_to_position(x, y, z, orientation[0], orientation[1], orientation[2])

class Corner:
    
    def __init__(self, x, y, z, side_1, side_2):
        self.x = x
        self.y = y
        self.z = z
        self.side_1 = side_1
        self.side_2 = side_2
    

def main() -> None:
    ur_handle = UrHandler()
    x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = ur_handle.get_position()

    # Create corners
    c1 = Corner(x=0.6621265919871204, 
                y=0.15445634174478098, 
                z=0.18823503429719188, 
                side_1=[0, -1], 
                side_2=[1, 0])
    
    c12 = Corner(x=0.653673471631566, 
                y=0.1540474965631938, 
                z=0.18823930031477493, 
                side_1=[0, -1], 
                side_2=[1, 0])
    
    c2 = Corner(x=0.6536188831313374, 
                y=0.3192992812114947, 
                z=0.18821135640862066,  
                side_1=[1, 0], 
                side_2=[0, 1])
    
    c22 = Corner(x=0.6538902873123501, 
                y=0.3121422194759388, 
                z=0.18819481250463477,  
                side_1=[1, 0], 
                side_2=[0, 1])
    
    c3 = Corner(x=0.8223156117250344, 
                y=0.3188142584322208, 
                z=0.18820907593595637,  
                side_1=[0, 1], 
                side_2=[-1, 0])
    
    c32 = Corner(x=0.8169870139827284, 
                y=0.3187929740321534, 
                z=0.18824343555883266,  
                side_1=[0, 1], 
                side_2=[-1, 0])
    
    c4 = Corner(x=0.8200987406532276, 
                y=0.16090666048714972, 
                z=0.18823930031477493,  
                side_1=[-1, 0], 
                side_2=[0, -1])
    
    c42 = Corner(x=0.821769414129218, 
                y=0.21248780678179371, 
                z=0.18825754272719747,  
                side_1=[-1, 0], 
                side_2=[0, -1])
    
    c5 = Corner(x=0.8488116727485047, 
                y=0.12467206222874708, 
                z=0.2623464296440813,  
                side_1=[-1, 0], 
                side_2=[0, -1])
    z_off = 0.000
    
    for obj in gc.get_objects():
        if isinstance(obj, Corner):
            obj.z = obj.z + z_off
        
    ur_handle.move_to_corner(c4)
    ur_handle.orient_to_corner(ur_handle.orienatations[1])
    ur_handle.move_to_corner(c12)
    ur_handle.orient_to_corner(ur_handle.orienatations[2])
    ur_handle.move_to_corner(c1)
    ur_handle.move_to_corner(c2)
    ur_handle.orient_to_corner(ur_handle.orienatations[3])
    ur_handle.move_to_corner(c22)
    ur_handle.move_to_corner(c3)
    ur_handle.orient_to_corner(ur_handle.orienatations[0])
    ur_handle.move_to_corner(c32)
    ur_handle.move_to_corner(c42)
    
    c42.x = c42.x + 0.01
    ur_handle.move_to_corner(c42)

    
    ur_handle.move_to_corner(c5)

    ur_handle.orient_to_corner(ur_handle.orienatations[3])

    ur_handle.orient_to_corner(ur_handle.orienatations[2])
    ur_handle.orient_to_corner(ur_handle.orienatations[1])
    

   
    
    


if __name__ == '__main__':
    
    main()
