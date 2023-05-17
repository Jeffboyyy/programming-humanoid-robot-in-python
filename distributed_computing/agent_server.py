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
import json
from xmlrpc.server import SimpleXMLRPCServer
from threading import Thread

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer(("localhost", 9000))
        self.server.register_function(self.get_angle, "admin_get_angle")
        self.server.register_function(self.set_angle, "admin_set_angle")
        self.server.register_function(self.get_posture, "admin_get_posture")
        self.server.register_function(self.execute_keyframes, "admin_execute_keyframes")
        self.server.register_function(self.get_transform, "admin_get_transform")
        self.server.register_function(self.set_transform, "admin_set_transform")
        t = Thread(target=self.server.serve_forever)
        t.start()
    
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller'''
        # YOUR CODE HERE
        self.perception.target_joints[joint_name] = angle
        
    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.starter = None
        self.keyframes = keyframes

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

