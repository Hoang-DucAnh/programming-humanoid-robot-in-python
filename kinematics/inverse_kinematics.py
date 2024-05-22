'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, matrix,linalg

from math import atan2
from scipy.linalg import pinv
import numpy as np

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        def from_trans(m):
            '''get x, y, theta from transform matrix'''
            return [m[-1, 0],  m[-1, 1],  m[-1, 2],  atan2(m[2, 1], m[2, 2]) ]



        
        

        joint_angles = {joint : self.perception.joint[joint] for joint in self.chains[effector_name]}
        #print(joint_angles)

        for joint in self.joint_names:
            if joint not in joint_angles:
                joint_angles[joint] = 0


        target = from_trans(transform)

        lambda_ = 1
        max_step = 0.1

        for i in range (1000):
        
            self.forward_kinematics(joint_angles)
            Ts = [value for value in self.transforms.values()]
            Te = matrix(
                [from_trans(Ts[-1])]
                ).T
        
            error = target - Te
            error[error > max_step] = max_step
            error[error < -max_step] = -max_step

            

            T = matrix([from_trans(i) for i in Ts[:-1]]).T
            J = Te - T
            dT = Te - T


            J[0, :] = dT[2, :]  # x
            J[1, :] = dT[1, :]  # y
            J[3, :] = dT[0, :]  # z ???
            J[-1, :] = 1        # angular


            d_theta = lambda_ * pinv(J).dot(error)
            for j, k in enumerate(self.chains[effector_name]):
                joint_angles[k] += np.asarray(d_theta.T)[0][j]

            if linalg.norm(error) < 1e-4:
                break

        joint_angles = [v for v in joint_angles.values()]
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        #names := [str, ...]  # list of joint names
        names = self.chains[effector_name]

        #times := [[float, float, ...], [float, float, ...], ...]
        # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
        times = [[1.0, 2.0]] * len(names)

        #keys := [[float, [int, float, float], [int, float, float]], ...]
        keys = [ [
                    [self.perception.joint[name], [0, 0, 0], [0, 0, 0]],
                    [self.inverse_kinematics(effector_name, transform)[i], [0, 0, 0], [0, 0, 0]],
                ] for i, name in enumerate(names)
                ]


        #keyframe := (names, times, keys)
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in
        #print(self.keyframes[0])
        #print(self.keyframes[1])
        #print(self.keyframes[2])

        #self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
