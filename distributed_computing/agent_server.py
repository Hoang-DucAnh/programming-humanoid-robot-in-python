'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer
from threading import Thread
import json as json


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def __init__(self):
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer(("localhost", 8080))


        client_functions = {"get_angle", "set_angle", "get_posture", "execute_keyframes", "get_transform", "set_transform"}
        for i in client_functions:
            exec("self.server.register_function(self.{0}, \"{0}\")".format(i))

        thread = Thread(target=self.server.serve_forever)
        thread.start()
        print("Server Aktiv")

    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.start_time = None
        self.keyframes = keyframes
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return json.dumps(self.transforms[name].tolist())

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.target_joints = self.inverse_kinematics(effector_name, json.loads(transform))

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

