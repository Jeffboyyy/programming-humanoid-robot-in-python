'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.starter = None
        self.animationcompleted = True

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        try: 
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        except:
            pass
        
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezierInterpolation(self, p0, p1, p2, p3, i_paraVorlesung):
        return ((1 - i_paraVorlesung)**3*p0 + 3*(1 - i_paraVorlesung)**2 * i_paraVorlesung*p1 + 3*(1 - i_paraVorlesung)*i_paraVorlesung**2 * p2 + (i_paraVorlesung**3)*p3)
    
    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]

        if (self.starter is None):
            self.starter = perception.time  # first start
        realtime = perception.time - self.starter  # time since start

        for i in range(len(names)):
            singlename = names[i]
            singlearraytime = times[i]
            singlearraykey = keys[i]
            if singlename not in self.joint_names:
                continue
            for j in range(len(singlearraytime) - 1):
                if(realtime < singlearraytime[0]):
                    p0 = perception.joint[singlename]
                    p1 = perception.joint[singlename]
                    p2 = singlearraykey[0][0] + singlearraykey[j][2][1] * singlearraykey[j][2][2]
                    p3 = singlearraykey[0][0]
                    i_paraVorlesung = realtime/singlearraytime[1]
                    target_joints[singlename] = self.bezierInterpolation(p0, p1, p2, p3, i_paraVorlesung)
                elif (realtime > singlearraytime[j] and realtime < singlearraytime[j+1]):
                    p0 = singlearraykey[j][0]
                    p1 = p0 + singlearraykey[j][2][2] * singlearraykey[j][2][1]
                    p2 = singlearraykey[j+1][0] + singlearraykey[j+1][1][2] * singlearraykey[j+1][1][1]
                    p3 = singlearraykey[j+1][0]
                    i_paraVorlesung = (realtime - singlearraytime[j])/(singlearraytime[j+1] - singlearraytime[j])
                    target_joints[singlename] = self.bezierInterpolation(p0, p1, p2, p3, i_paraVorlesung)
                
                    
        if(target_joints == {}):
            self.animationcompleted = True
        else:
            self.animationcompleted = False
        
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()