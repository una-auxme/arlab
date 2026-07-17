#!/usr/bin/env python3
"""
Force Monitor Node to recognize load drops of the Mia Hand Force Sensors.

This Node provides an activation service for decision making,
subscribes to the mia hand force stream, reads the forces when being activated, 
checks for sudden force drops and provides an service if a force 
drop was detected

Maintainer:
    Marc Stumpp <marc.stumpp@uni-a.de>
"""

# angesprochene Specification: DM-A-02 Fehlerfall erkennen

#Ziel: Kraftabfall beim Tragen erkennen und melden. Decision Making reagiert dann.

#bisher:
#JobRunner: schaltet den Stream. Pick: an nach Close(). Place: aus vor Open().
#force_monitor.py: bietet SetBool-Service armed (true/false), subscribed Kraftdaten, 
#published Drop-Event.
#Decision Making: schaltet scharf, lauscht auf das Event, entscheidet.

#Detektor: Baseline aus ersten Werten, Alarm erst nach N Messungen in Folge 
#unter Schwelle (Zähler, Reset bei Wert drüber). 
#Ein Sturz = Event. Schwelle + N als ROS2-Parameter.

#Zusatz 16.07. in der Uni
#Test hat den Code grundsätzlich bestätigt, aber ein problem ist das das detektieren
#von allen 3 Sensoren beim Pinch Grip bspw. nicht funktionieren wird. 
#Also musste der code von "and" mit allen 3 sensoren mit dem Kraftabfall zu "or"
#geändert werden. Dann werden aber potentiell nicth mehr alle fälle abgefangen
#Lösung: Gripunterscheidung muss her-> wie noch unklar 

#Auserdem sollte der Drop fall im orchestrator auch abgefangen werden
#ggf. soll die funktion auch vom orchestrator aktiviert werden 

from rclpy.node import Node 
from mia_hand_msgs.msg import ForceData
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from GetObjectDropped.srv import GetDropStatus
import rclpy 

class force_monitor(Node):
    """ nochmal eine beschreibung einfügen bittö"""

    def __init__(self):
        """überall..."""

        super().__init__("force_monitor")
        
        self.armed = False
        self.median_complete = False
        self.drop_reported = False

        self.allowed_force_jitter = 50
        self.allowed_reports = 50

        # This value should be uneven
        self.values_to_calculate_median = 21

        self.report_counter = 0

        self.thumb_nforce_list = []
        self.index_nforce_list = []
        self.mrl_nforce_list = []

        self.thumb_nforce_median = 0
        self.index_nforce_median = 0
        self.mrl_nforce_median = 0

        self.create_subscription(ForceData, '/mia_hand/data_streams/fingers/forces/data', self.get_data, 10)
        self.activation_status = self.create_service(SetBool, '/force_monitor/activate', self.activation_callback)
        self.dropped_service = self.create_service(GetDropStatus, '/object_dropped', self.object_dropped_response)

    def activation_callback(self, request, response):

        is_activatet = request.data

        if is_activatet:
            self.armed = True
            self.thumb_nforce_list.clear()
            self.index_nforce_list.clear()
            self.mrl_nforce_list.clear()
            self.thumb_nforce_median = 0
            self.index_nforce_median = 0
            self.mrl_nforce_median = 0
            self.report_counter = 0
            self.median_complete = False
            self.drop_reported = False
            response.success = True
            response.message = "Force monitor activated"
            return response
        else:
            self.armed = False
            response.success = True
            response.message = "Force monitor deactivated"
            return response

    def get_data(self, msg):
        """Gets the Values from the current iteration from the ForceData Topic
        first is thumb_nforce, second index_nforce, third mrl_nforce. """

        # Save some computation here maybe if Object is already dropped
        if not self.armed or self.drop_reported:
            return

        # Werte für Median sammeln und dann den Median berechnen lassen
        if not self.median_complete:
            self.thumb_nforce_list.append(msg.thumb_nfor)
            self.index_nforce_list.append(msg.index_nfor)
            self.mrl_nforce_list.append(msg.mrl_nfor)
            if len(self.thumb_nforce_list) == self.values_to_calculate_median:
                self.calculate_median()
            return

        dropped = (
            abs(self.thumb_nforce_median - msg.thumb_nfor) > self.allowed_force_jitter
            or abs(self.index_nforce_median - msg.index_nfor) > self.allowed_force_jitter
            or abs(self.mrl_nforce_median - msg.mrl_nfor) > self.allowed_force_jitter
        )
        
        if dropped:
            self.report_counter += 1
        else:
            self.report_counter = 0

        if self.report_counter >= self.allowed_reports and not self.drop_reported:
            self.get_logger().warn("Force drop detected. Object lost")
            self.drop_reported = True
                
    def calculate_median(self):
        """ calculates the median..."""
        self.thumb_nforce_list.sort()
        self.index_nforce_list.sort()
        self.mrl_nforce_list.sort()
        self.thumb_nforce_median = self.thumb_nforce_list[self.values_to_calculate_median // 2]
        self.index_nforce_median = self.index_nforce_list[self.values_to_calculate_median // 2]
        self.mrl_nforce_median = self.mrl_nforce_list[self.values_to_calculate_median // 2]
        self.median_complete = True

    def object_dropped_response(self, request, response):
        """Provides a service to respose wether object got dropped or not """
        response.GetDropStatus = self.drop_reported

def main(args=None):
    rclpy.init(args=args)
    node = force_monitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()