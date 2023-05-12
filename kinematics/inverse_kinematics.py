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
from numpy.matlib import identity
import numpy as np
from scipy.linalg import pinv


class InverseKinematicsAgent(ForwardKinematicsAgent):
    #first 3 stats of array are for the rotation in x,y,z and the 4th is the degree with which we rotate
    def rotationsfunc(self, matrix):
        return [matrix[3, 0],matrix[3, 1],matrix[3, 2],np.arctan2(matrix[2, 1], matrix[2, 2])]
        
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        errorsize = 1
        errormarker = 1e-5 
        alpha = 0.2
        joint_angles = {}
        ergebnis = []
        #get the current angles 
        for i in self.chains[effector_name]:
            joint_angles[i] = self.perception.joint[i]
        
        #useless joints -> 0
        for j in self.joint_names:
            if j not in joint_angles:
                joint_angles[j] = 0
    
        for i in range(4000):
            self.forward_kinematics(joint_angles)
            TS = []
            for i in self.transforms:
                TS.append(self.transforms[i])
            TE = np.matrix([self.rotationsfunc(TS[-1])]).T
            
            
            error = self.rotationsfunc(transform) - TE
            error[error > errorsize] = errorsize
            error[error < errorsize] = -errorsize
            
            T = np.matrix([self.rotationsfunc(j) for j in TS[:-1]]).T
            Jacobian = TE - T
            deltaT = TE - T
            Jacobian[0, :] = deltaT[2, :]
            Jacobian[1, :] = deltaT[1, :]
            Jacobian[2, :] = deltaT[0, :]
            Jacobian[3, :] = 1
            
        
            alphaangle = alpha * pinv(Jacobian) * error
            
            #alphaangle has to be a scalar for my calculation
            for joint in self.chains[effector_name]:
                joint_angles[joint] += np.asarray(alphaangle.T)[0][0]
            
            if(np.linalg.norm(error) < errormarker):
                break
            
        for i in joint_angles.values():
            ergebnis.append(i)
        return ergebnis

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name]
        timesarray = []
        keysarray = []
        for i, partname in enumerate(names):
            timesarray.append([3.0, 10.0])
            keysarray.append([[self.perception.joint[partname], [3.0, 0.0, 0.0], [3.0, 0.0, 0.0]],[joint_angles[i], [3.0, 0.0, 0.0], [3.0, 0.0, 0.0]]])

        self.keyframes = (names, timesarray, keysarray)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
