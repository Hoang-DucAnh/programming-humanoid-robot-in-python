'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import numpy as np
from numpy.matlib import matrix, identity
from recognize_posture import PostureRecognitionAgent # type: ignore


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE

                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 
                                #'LWristYaw', 'LHand'
                                ],
                        
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 
                                #'RWristYaw', 'RHand'
                                ],
                    

                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 
                                'LKneePitch', 'LAnklePitch', 'RAnkleRoll'],

                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 
                                'RKneePitch', 'RAnklePitch', 'LAnkleRoll']

                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        self.jointOffsets = {
            "Head": [
                [0.0, 0.0, 126.5],  #   Torso   to HeadYaw
                [0.0, 0.0, 0.0  ],  #   HeadYaw to HeadPitch
            ],
            "LArm": [
                [0.0,   98.0,   100.0], # Torso             to LShoulderPitch
                [0.0,   0.0,    0.0],   # LShoulderPitch    to LShoulderRoll
                [105.0, 15.0,   0.0],   # LShoulderRoll     to LElbowYaw
                [0.0,   0.0,    0.0],   # LElbowYaw         to LElbowRoll
                [55.95, 0.0,    0.0],   # LElbowRoll        to LWristYaw
            ],
            "RArm": [
                [0.0,   -98.0,  100.0], # Torso             to RShoulderPitch
                [0.0,   0.0,    0.0],   # RShoulderPitch    to RShoulderRoll
                [105.0, -15.0,  0.0],   # RShoulderRoll     to RElbowYaw
                [0.0,   0.0,    0.0],   # RElbowYaw         to RElbowRoll
                [55.95, 0.0,    0.0],   # RElbowRoll        to RWristYaw
            ],
            "LLeg": [
                [0.0, 50.0, -85.0   ],  # Torso         to LHipYawPitch
                [0.0, 0.0,  0.0     ],  # LHipYawPitch  to LHipRoll
                [0.0, 0.0,  0.0     ],  # LHipRoll      to LHipPitch
                [0.0, 0.0,  -100.0  ],  # LHipPitch     to LKneePitch
                [0.0, 0.0,  -102.9  ],  # LKneePitch    to LAnklePitch
                [0.0, 0.0,  0.0     ],  # LAnklePitch   to LAnkleRoll
            ],
            "RLeg": [
                [0.0, -50.0, -85.0  ],  # Torso         to RHipYawPitch
                [0.0, 0.0,   0.0    ],  # RHipYawPitch  to RHipRoll
                [0.0, 0.0,   0.0    ],  # RHipRoll      to RHipPitch
                [0.0, 0.0,   -100.0 ],  # RHipPitch     to RKneePitch
                [0.0, 0.0,   -102.9 ],  # RKneePitch    to RAnklePitch    
                [0.0, 0.0,   0.0    ],  # RAnklePitch   to RAnkleRoll
            ]
        }


        if joint_name.endswith("Roll"):
            T = matrix([[1,     0,                      0,                      0], 
                        [0,     np.cos(joint_angle),    -np.sin(joint_angle),   0],  
                        [0,     np.sin(joint_angle),    np.cos(joint_angle),    0],  
                        [0,     0,                      0,                      1]])

        elif joint_name.endswith("Pitch"):
            T = matrix([[np.cos(joint_angle),   0,  np.sin(joint_angle),    0], 
                        [0,                     1,  0,                      0], 
                        [-np.sin(joint_angle),  0,  np.cos(joint_angle),    0], 
                        [0,                     0,  0,                      1]])

        elif joint_name.endswith("Yaw"):
            T = matrix([[np.cos(joint_angle),   -np.sin(joint_angle),   0, 0], 
                        [np.sin(joint_angle),   np.cos(joint_angle),    0, 0], 
                        [0,                     0,                      1, 0], 
                        [0,                     0,                      0, 1]])

        for chain in self.chains:
            if joint_name in self.chains[chain]:
                index = self.chains[chain].index(joint_name)
                for i in range(3):
                    T[i, 3] = self.jointOffsets[chain][index][i]


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                #T = T @ Tl
                T = np.dot(T, Tl)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
