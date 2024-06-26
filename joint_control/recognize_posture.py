'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('joint_control/robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        
        # Extrahiere die Gelenkwerte aus der perception
        joint_values = [
            perception.joint.get('LHipYawPitch', 0),
            perception.joint.get('LHipRoll', 0),
            perception.joint.get('LHipPitch', 0),
            perception.joint.get('LKneePitch', 0),
            perception.joint.get('RHipYawPitch', 0),
            perception.joint.get('RHipRoll', 0),
            perception.joint.get('RHipPitch', 0),
            perception.joint.get('RKneePitch', 0),
            perception.imu[0],  # AngleX
            perception.imu[1]   # AngleY
        ]

        # Verwende den geladenen Klassifizierer, um die Haltung vorherzusagen
        posture = self.posture_classifier.predict([joint_values])

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
