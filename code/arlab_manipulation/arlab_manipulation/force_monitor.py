#!/usr/bin/env python3
"""
Force Monitor Node for detecting load drops on the mia hand force sensors.

This Node provides an activation service for the orchestrator,
subscribes to the mia hand force stream, creates a baseline from the first
values after being armed, checks the incoming values for sudden force 
drops and provides a service if a force drop was detected

Maintainer:
    Marc Stumpp <marc.stumpp@uni-a.de>
"""

from rclpy.node import Node 
from mia_hand_msgs.msg import ForceData
from arlab_common_interfaces.srv import GetObjectDropped, ActivateForceMonitor
import rclpy 

class force_monitor(Node):
    """ROS2 Node for monitoring the Mia Hand force sensors for load drops.

    Responsibilities:
        - Accepts arm and disarm requests from the service ActivateForceMonitor.
        - Subscribes to the mia hand force stream.
        - Creates a baseline force for each sensor from a fixed value count.
        - Compares values against that baseline, depending on grip type.
        - Saves the status if an object was dropped.
        - Provides a service to get the drop status from the service GetObjectDropped.

    A drop is reported only after the forces and the saved baseline have a certain difference
    for multiple consecutive values. The result is saved until the node is activated again.

    Attributes:
        KNOWN_GRIP_TYPES: Grip types with a dedicated detection branch.
        armed: Flag if incoming values are evaluated.
        median_complete: Flag if the baseline has been computed.
        drop_reported: Latched detection result for the current grasp.
        allowed_force_jitter: Deviation from the baseline counting as a drop.
        allowed_reports: Consecutive drop conditions before reporting.
        values_to_calculate_median: Baseline window size, must be uneven.
        report_counter: Current run length of consecutive drop conditions.
        grip_type: Grip currently monitored, selects the sensors evaluated.
        nforce_lists: Collected baseline values per sensor.
        nforce_medians: Baseline force per sensor.
    """

    # tridigital and spherical are custom grip types that are not defined yet
    KNOWN_GRIP_TYPES = ("cylindrical", "pinch", "lateral")

    def __init__(self):
        """ROS2 node for detection state, services, and subscription.
 
        Side Effects:
            - Subscribes to /mia_hand/data_streams/fingers/forces/data for forces.
            - Registers /force_monitor/activate to arm and disarm the monitor.
            - Registers /object_dropped to provide the dropped status.
        """

        super().__init__("force_monitor")
        
        self.armed = False
        self.median_complete = False
        self.drop_reported = False

        self.allowed_force_jitter = 50
        self.allowed_reports = 50

        # This value should be uneven
        self.values_to_calculate_median = 21

        self.report_counter = 0

        # Default is cylindrical
        self.grip_type = "cylindrical"

        self.nforce_lists = {"thumb": [], "index": [], "mrl": []}
        self.nforce_medians = {"thumb": 0, "index": 0, "mrl": 0}

        self.create_subscription(ForceData, '/mia_hand/data_streams/fingers/forces/data', self.get_data, 10)
        self.activation_status = self.create_service(ActivateForceMonitor, '/force_monitor/activate', self.activation_callback)
        self.dropped_service = self.create_service(GetObjectDropped, '/object_dropped', self.object_dropped_response)

    def activation_callback(self, request, response):
        """Arm and disarm requests from the job-runner when picking and placing objects.

        Arming resets the dropped state, warns on an unknown grip type and sets the default type.
        Sets self.armed last, so no value is processed against a half-reset
        state. Disarming only stops the evaluation and keeps the dropped status, 
        so that it can be checked later by the orchestrator.

        Side Effects:
            - Updates self.armed and, when arming, self.grip_type
            - When arming, resets all detection state
            - Logs a warning for an unknown grip type

        Args:
            request: `activate` selects arm or disarm, `grip_type` is only evaluated when arming.
            response: Response object to be filled with status and message.

        Returns:
            ActivateForceMonitor response, `success` is always True.
        """

        is_activated = request.activate

        if is_activated:
            self.grip_type = request.grip_type

            if self.grip_type not in self.KNOWN_GRIP_TYPES:
                self.get_logger().warn(
                    f"Unknown grip type or typing error with type: '{self.grip_type}'. Falling back to default grip type (cylindrical)."
                )
                self.grip_type = "cylindrical"

            for sensor in self.nforce_lists:
                self.nforce_lists[sensor].clear()
                self.nforce_medians[sensor] = 0

            self.report_counter = 0
            self.median_complete = False
            self.drop_reported = False
            self.armed = True
            response.success = True
            response.message = "Force monitor activated"
            return response
        else:
            self.armed = False
            response.success = True
            response.message = "Force monitor deactivated"
            return response

    def get_data(self, msg):
        """Callback to receive and check one value from the force stream.

        Sequence:
            1. Discard the value if not armed or a drop is already detected.
            2. Collect the value while the baseline is not calculated yet and
               trigger the median baseline calculation once it is.
            3. Otherwise compare with the saved baseline. Which sensors are relevant
               depends on self.grip_type.
            4. Count consecutive drop conditions and report at the required
               run length.

        Side Effects:
            - Appends to self.nforce_lists during baseline collection
            - Updates self.report_counter
            - Sets self.drop_reported and logs a warning on drop detection

        Args:
            msg: ForceData value with thumb_nfor, index_nfor and mrl_nfor (thumb, index, mrl).
        """

        if not self.armed or self.drop_reported:
            return

        forces = {"thumb": msg.thumb_nfor, "index": msg.index_nfor, "mrl": msg.mrl_nfor}

        # Create the baseline by collecting values and calculating the median
        if not self.median_complete:
            for sensor, value in forces.items():
                self.nforce_lists[sensor].append(value)
            if len(self.nforce_lists["thumb"]) == self.values_to_calculate_median:
                self.calculate_median()
            return

        # tridigital and spherical are custom grip types that are not defined yet
        # after implementing one or both, the relevant force sensors for the grips should be addressed here
        if self.grip_type == "pinch":
            dropped = (
                (self.nforce_medians["thumb"] - forces["thumb"]) > self.allowed_force_jitter
                and (forces["index"] - self.nforce_medians["index"]) > self.allowed_force_jitter
            )
        
        elif self.grip_type == "lateral":
            dropped = (
                (forces["thumb"] - self.nforce_medians["thumb"]) > self.allowed_force_jitter
            )      

        else:
            dropped = (
                (self.nforce_medians["thumb"] - forces["thumb"]) > self.allowed_force_jitter
                and (forces["index"] - self.nforce_medians["index"]) > self.allowed_force_jitter
                and (forces["mrl"] - self.nforce_medians["mrl"]) > self.allowed_force_jitter
            )     
        
        if dropped:
            self.report_counter += 1
        else:
            self.report_counter = 0

        if self.report_counter >= self.allowed_reports and not self.drop_reported:
            self.get_logger().warn("Force drop detected. Object lost")
            self.drop_reported = True
                
    def calculate_median(self):
        """Calculate the baseline force per sensor from the collected values.
 
        A median is used so that single outliers in the stream do not shift the
        baseline.
 
        Side Effects:
            - Sorts the lists in self.nforce_lists in place
            - Updates self.nforce_medians
            - Sets self.median_complete, switching get_data to evaluation
        """

        for sensor, values in self.nforce_lists.items():
            values.sort()
            self.nforce_medians[sensor] = values[self.values_to_calculate_median // 2]
        self.median_complete = True

    def object_dropped_response(self, request, response):
        """Provides the saved drop status.
 
        The value refers to the grasp from when the node was last armed and stays valid
        after disarming.
 
        Args:
            request: GetObjectDropped request, takes no arguments.
            response: Response object to be filled with the drop status.
 
        Returns:
            GetObjectDropped response containing object_dropped.
        """

        response.object_dropped = self.drop_reported
        return response

def main(args=None):
    """Start the force monitor node."""

    rclpy.init(args=args)
    node = force_monitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()