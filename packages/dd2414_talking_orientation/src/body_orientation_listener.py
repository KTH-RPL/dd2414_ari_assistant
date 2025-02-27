#!/usr/bin/env python3
import actionlib.msg
import rospy
import numpy as np
import pyhri
import sys
import actionlib
import move_base_msgs.msg


from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

from tf import transformations as t
from tf import TransformBroadcaster as tf_B
from tf import TransformListener
from tf import LookupException,ExtrapolationException,ConnectivityException
from tf2_ros import Buffer
from hri_msgs.msg import IdsList, LiveSpeech
from geometry_msgs.msg import Point, PointStamped
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String
from ollama import Client

"""

class Listener(object):
    def __init__(self):
        self.listener_sub = rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.listener_result)
        self.tts_client = SimpleActionClient("/tts",TtsAction)
        self.tts_client.wait_for_server()
        self.language = "en_US"
        self.translate = False
        self.api = Client(
            host='http://192.168.0.106:11434',
            headers={'x-some-header': 'some-value'}
        )
        self.system_prompt = "Translate a sentence. Return only the translation"
        rospy.loginfo("Listening...")

    def listener_result(self,msg):
        sentence = msg.final
        rospy.loginfo("Understood sentence: " + sentence)

        if (sentence == "begin translation"):
            ## MOVE CLOSE TO THE USER
            self.tts_output("Begining translation")
            self.translate == True

        if self.translate:
            self.translate_sentence(sentence)
            self.translate = False

    def tts_output(self,answer):
        self.tts_client.cancel_goal()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = str(answer)
        self.tts_client.send_goal_and_wait(goal)

    def translate_sentence(self,sentence):
        model_ollama = "mistral:latest" #llama3.2:latest, mistral:latest, deepseek-r1:latest
        try:
            completion = self.api.chat(
                model=model_ollama, 
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": "Translate the following sentence to Spanish: "+str(sentence)},
                    ] 
            )
            response = completion.message.content
            rospy.loginfo(f"DeepSeek Response: {response}")
            self.tts_output(response)

        except Exception as e:
            rospy.logerr(f"Error generating response: {e}")
            return "Sorry, I couldn't process your request."

"""
# Activar comportamiento con "Translate this conversation"
# Mover hacia el sujeto identificado, el sujeto tiene que dar la instrucción mientras ve a ARI a los ojos
# Rotar a ARI para que vea a la otra persona
# Comenzar la traducción diciendo "begin translation"
# Finalizar comportamiento con "end translation"


class BodyOrientationListener:

    def __init__(self, base_frame = "head_front_camera_link", threshold=30):
        self.hri_listener = pyhri.HRIListener()
        self.base_frame = base_frame
        self.threshold = threshold
        self.rate = rospy.Rate(1)
        self.br = tf_B()
        self._tf_buffer = Buffer()
        self._tf_Listener = TransformListener()

    def quaternion_euler(self,quaternion):
        euler = t.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
        return euler[2]

    def nav_move_base(self,x,y,z,rx,ry,rz,rw):
        move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        move_client.wait_for_server()
        rospy.loginfo("Move base client ready")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = rx
        goal.target_pose.pose.orientation.y = ry
        goal.target_pose.pose.orientation.z = rz
        goal.target_pose.pose.orientation.w = rw

        rospy.loginfo("Sending goal")
        move_client.send_goal(goal)
        move_client.wait_for_result()

        rospy.loginfo(f"Goal state:{move_client.get_state()}")
        rospy.loginfo(move_client.get_goal_status_text())
        
        return move_client.get_state()


    def run(self):
        
        while not rospy.is_shutdown():
            bodies = self.hri_listener.bodies.items()
            bodies_facing_robot = []
            
            for body in bodies:
                print("body_" + body[0])
                right_euler = t.quaternion_from_euler(0,0,np.pi/3)
                left_euler  = t.quaternion_from_euler(0,0,-np.pi/3)

                self.br.sendTransform((0.5,0.8,0.0),
                                 (left_euler[0],left_euler[1],left_euler[2],left_euler[3]),
                                 rospy.Time.now(),
                                 "left_" + body[0],
                                 "body_" + body[0])
                
                self.br.sendTransform((0.5,-0.8,0.0),
                                 (right_euler[0],right_euler[1],right_euler[2],right_euler[3]),
                                 rospy.Time.now(),
                                 "right_" + body[0],
                                 "body_" + body[0])
                
                try:
                    (left_trans, left_rot) = self._tf_Listener.lookupTransform("map","left_"+body[0],rospy.Time(0))
                    (right_trans, right_rot) = self._tf_Listener.lookupTransform("map","right_"+body[0],rospy.Time(0))
                    transform_valid = True
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    print(e)
                    print("Transform not published yet")
                    transform_valid = False
                    continue

                transform = body[1].transform(self.base_frame)
                
                trans = transform.transform.translation
                rot = transform.transform.rotation

                translation_matrix = t.translation_matrix((trans.x, trans.y, trans.z))
                quaternion_matrix = t.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
                transform = t.concatenate_matrices(translation_matrix, quaternion_matrix)
                inv_transform = t.inverse_matrix(transform)
                b2r_translation_x = inv_transform[0, 3]
                b2r_translation_y = inv_transform[1, 3]
                b2r_xy_norm = np.linalg.norm([b2r_translation_x, b2r_translation_y], ord = 2)

                if np.arccos(b2r_translation_x/b2r_xy_norm) < (self.threshold/180*np.pi) and b2r_translation_x > 0:
                    print("Starting movement")
                    bodies_facing_robot.append(body[0])

                    try:
                        if transform_valid:
                            
                            result = self.nav_move_base(left_trans[0],left_trans[1],0,left_rot[0],left_rot[1],left_rot[2],left_rot[3])

                            if result == actionlib.GoalStatus.SUCCEEDED:
                                rospy.loginfo("Arrived at target!")
                                
                            else:
                                rospy.loginfo("Trying right position!")
                                result = self.nav_move_base(right_trans[0],right_trans[1],0,right_rot[0],right_rot[1],right_rot[2],right_rot[3])
                        else:
                            rospy.loginfo("Transform not available!")
                    except rospy.ROSInterruptException:
                        rospy.loginfo("Finished.")

            self.rate.sleep()


if __name__=="__main__":
    rospy.init_node("body_orientation_listener")
    bol = BodyOrientationListener(threshold=20)
    bol.run()