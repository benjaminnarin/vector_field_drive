#!/usr/bin/env python
"""
Module field_movement.py
Creates a class which calculates a rough field around a moving robot, and then
attempts to steer the robot away from any obstacles by calculating any colliding
walls, based on the laser data, and then subtracting a surface normal projection
away from the inputted command velocity. This should keep the robot from driving
into the objects, as it removes the velocity component that pushes the robot
into the obstacle.
"""
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import rospy

class VectorFieldError(Exception):
    pass

class VectorFieldMover(object):
    """
    This class exists to pair up the continuous callback of the laser scan
    with the sparse callbacks for the velocity movements.
    """
    def __init__(self):
        """
        distance threshold says how far away do I look before something is dangerously close
        num_divisions is the number of divisions in the laser scan to create
        """
        self._laser_scan = None
        self._vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.distance_threshold = 1.25 #meters
        self.num_divisions = 19# best guess? seems to work better with 16 than 10; Switched to 19 to fit scan_multi outputs

    def laser_scan_cb(self, msg):
        """ Records the recieved laser scan into the class.  """
        self._laser_scan = msg

    def wall_normal(self, distance, start_index, div_width):
        """
        Helper function to calculate the normal vector of the imaginary
        wall created by the closest laser scan within a single section.

        Args:
            distance: distance to the "wall"
            start_index: starting index within the laser scan for this division
            div_width: width of divisions for the laser scan

            start_index & div_width are used to calculate the normal angle for a wall orthogonal to
            the robot, if the robot was directly facing the wall in this division.

        Returns: surface normal vector of the hypothetical wall
        """
        angle = start_index*div_width + div_width/2
        angle *= self._laser_scan.angle_increment
        angle -= self._laser_scan.angle_min
        vector = np.array([-1*np.cos(angle), -1*np.sin(angle)])
        return vector

    def velocity_cb(self, msg):
        """
        Converts the current laser scan into a rough vector field near
        the robot. These vectors are collected, and then applied on the current
        velocity value (via subtraction). The resulting vector is then republished
        on a seperate velocity topic.
        """
        # Sanity check for the first update
        if self._laser_scan is None:
            print "No laser scan in object"
            return

        # Keep track of a planar version of the twist message, so we can modify it during later steps
        planar_vel = np.array([msg.linear.x, msg.linear.y])

        # Divide the laser scan into a series of divisions, and calculate a vector
        # for each division. (Bill's suggestion, the easiest implementation)

        # Check the divisions for the proper size
        if len(self._laser_scan.ranges) % self.num_divisions > 0:
            raise VectorFieldError("The number of divisions ({}) does not cleanly divide the scan size ({})".format(self.num_divisions, len(self._laser_scan.ranges)))

        # Calculate out the vector components to subtract, based on the divisions
        div_width = len(self._laser_scan.ranges)/self.num_divisions
        # Loop across each division in the laser scan
        for start_index in range(0, len(self._laser_scan.ranges), div_width):
            # Slice this division out of the laser scan and find the minimum
            scan_beam = self._laser_scan.ranges[start_index:start_index+div_width]
            closest_distance = min(scan_beam)
            print "Start index: {}    Min distance: {}".format(start_index, closest_distance)
            # create normal vector based on closest distance
            if closest_distance < self.distance_threshold:
                # Consider this closest distance to me as a normal from a wall at that distance, normal to the robot.
                wall_normal = self.wall_normal(closest_distance, start_index, div_width)

                # Actually heading toward the wall?
                if np.dot(wall_normal, planar_vel) > 0:
                    # Calculate the vector projection of the requested velocity into the wall normal, and subtract that away
                    projection = (np.dot(planar_vel, wall_normal) / np.linalg.norm(wall_normal)**2)*wall_normal
                    print "Subtracting projection: ", projection
                    planar_vel -= projection

        # Compare the fully compute planar velocity to travel along, vs the original input velocity
        print "Planar vel: ", planar_vel
        print "Refer  vel: ", np.array([msg.linear.x, msg.linear.y])
        # Assign and publish
        msg.linear.x = planar_vel[0]
        msg.linear.y = planar_vel[1]
        self._vel_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("vel_field")
    VFM = VectorFieldMover()
    # Register callbacks for laserscan and incoming velocity
    rospy.Subscriber("/scan", LaserScan, callback=VFM.laser_scan_cb, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, callback=VFM.velocity_cb, queue_size=10)
    rospy.spin()
