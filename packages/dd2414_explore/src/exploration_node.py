#!/usr/bin/env python3

import rospy
import actionlib
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain

class ExplorationNode:
    def __init__(self):
        self.current_goal = None
        rospy.loginfo("[EXPLORE        ]: Initialized")
        self.string_header = "[EXPLORE        ]:"

    def action(self, goal):
        result = brain.BrainResult()

        rospy.loginfo(f"{self.string_header} Starting exploration.")

        for room in self.rooms:
            #################################### TODO Get rooms from zois in the map and make the list
            if self.current_goal != goal.goal and self.current_goal is not None:
                result.result = "Failure"
                self.preempted()
                return result

            rospy.loginfo(f"{self.string_header} Exploring room: {room}")
            self.go_to_door(room)
            
            if self.is_door_open(room):
                rospy.loginfo(f"{self.string_header} Door open, entering.")
                self.go_through_door()
            else:
                rospy.loginfo(f"{self.string_header} Door closed, asking for help.")
                self.ask_for_help()
                continue

            self.go_to_center(room)

        result.result = "Success"

        self.current_goal = None
        return result

    def execute_cb(self, goal):
        feedback = ExploreFeedback()
        result = ExploreResult()

        rospy.loginfo("Exploración iniciada...")
        
        # Lista de habitaciones de ejemplo
        rooms = ["sala_reuniones", "despacho", "cocina"]
        for room in rooms:
            rospy.loginfo(f"Yendo a la habitación: {room}")
            
            # Paso 1: ir a la puerta
            self.go_to_door(room)
            
            # Paso 2: revisar si está abierta (simulado)
            if self.is_door_open(room):
                rospy.loginfo(f"Puerta de {room} abierta.")
                self.go_through_door()
            else:
                rospy.loginfo(f"Puerta de {room} cerrada. Pidiendo ayuda...")
                self.ask_for_help()
                continue  # Saltar esta habitación por ahora
            
            # Paso 3: ir al centro o a un punto libre
            self.go_to_center(room)

            # Enviar feedback al cliente
            feedback.feedback_message = f"Explorado: {room}"
            self.server.publish_feedback(feedback)

            # Comprobar cancelación
            if self.server.is_preempt_requested():
                rospy.loginfo("Exploración cancelada.")
                self.server.set_preempted()
                return

        result.result_message = "Exploración completa."
        self.server.set_succeeded(result)

    def go_to_door(self, room):
        rospy.loginfo(f"--> Navegando hasta la puerta de {room}")
        rospy.sleep(2)  # Simula el tiempo de navegación

    def is_door_open(self, room):
        # Aquí puedes usar sensores, cámara, costmap, etc.
        return True  # Simulación: siempre abierta

    def go_through_door(self):
        rospy.loginfo("--> Cruzando la puerta")
        rospy.sleep(1)

    def ask_for_help(self):
        rospy.loginfo("--> '¿Puedes abrir la puerta, por favor?'")
        rospy.sleep(2)

    def go_to_center(self, room):
        rospy.loginfo(f"--> Yendo al centro de {room}")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('exploration_node')
    server = ExplorationNode()
    rospy.spin()
