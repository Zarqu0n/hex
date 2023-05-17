import math
import numpy as np

class LegKinematics:
    def __init__(self) -> None:
        self.__declare_parameters()

    def __declare_parameters(self):
        # Declare initial parameters
        self._coxa_length = 0.6
        self._femur_length = 0.555
        self._tibia_length = 0.755

        self.base_coxa_dist = 0.6
        self.fi = 0
        self.O_12 = 0
        self.O_23 = 0
        self.O_34 = 0

        self.__dh_matrix()

    def __dh_matrix(self):
        # Declare DH parameters
        # alfa_ij, a_ij, d_ij, O_ij
        self._dh_matrix = np.array([
            [0, self.base_coxa_dist, 0, self.fi],
            [math.pi/2, self._coxa_length, 0, self.O_12],
            [0, self._femur_length, 0, self.O_23],
            [0,  self._tibia_length,0, self.O_34]
        ])

    def __fk_matrix(self, alfa_ij, a_ij, d_ij, O_ij):
        # Declare forward kinematics matrix
        self._fk_matrix = np.array([
            [math.cos(O_ij), -math.sin(O_ij)*math.cos(alfa_ij), math.sin(O_ij)*math.sin(alfa_ij), a_ij*math.cos(O_ij)],
            [math.sin(O_ij), math.cos(O_ij)*math.cos(alfa_ij), -math.cos(O_ij)*math.sin(alfa_ij), a_ij*math.sin(O_ij)],
            [0, math.sin(alfa_ij), math.cos(alfa_ij), d_ij],
            [0, 0, 0, 1]
        ])

        return self._fk_matrix

    def get_T0(self,fi=0):
        # Get T0 matrix
        return  self.__fk_matrix(self._dh_matrix[0][0], self._dh_matrix[0][1], self._dh_matrix[0][2], fi)
 
    def get_T01(self,O_12=0):
        # Get T1 matrix
        T0 = self.get_T0()
        T1 = self.__fk_matrix(self._dh_matrix[1][0], self._dh_matrix[1][1], self._dh_matrix[1][2], O_12)
        return np.matmul(T0,T1)
    
    def get_T02(self,O_12=0,O_23=0):
        # Get T2 matrix
        T01 = self.get_T01(O_12)
        T2 = self.__fk_matrix(self._dh_matrix[2][0], self._dh_matrix[2][1], self._dh_matrix[2][2], O_23)
        return np.matmul(T01,T2)
    
    def get_T03(self,O_12=0,O_23=0,O_34=0):
        # Get T3 matrix
        T02 = self.get_T02(O_12,O_23)
        T3 = self.__fk_matrix(self._dh_matrix[3][0], self._dh_matrix[3][1], self._dh_matrix[3][2], O_34)
        return np.matmul(T02,T3)
    

    def get_pose(self,O_12=0,O_23=0,O_34=0):
        # Get pose
        T03 = self.get_T03(O_12,O_23,O_34)
        return np.array([
            self.base_coxa_dist*math.cos(self.fi) + math.cos(self.fi + O_12)*(self._coxa_length + self._femur_length*math.cos(O_23) + self._tibia_length*math.cos(O_23 + O_34)),
            self.base_coxa_dist*math.sin(self.fi) + math.sin(self.fi + O_12)*(self._coxa_length + self._femur_length*math.cos(O_23) + self._tibia_length*math.cos(O_23 + O_34)),
            self._femur_length*math.sin(O_23) + self._tibia_length*math.sin(O_23 + O_34)
        ])

if __name__ == "__main__":
    leg = LegKinematics()
    print(leg.get_T03())
    print(leg.get_pose())
