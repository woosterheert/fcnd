import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection, altitude, size_of_box):
        super().__init__(connection)
        self.in_mission = True
        self.check_state = {}
        self.altitude = altitude
        self.waypoints = self.calculate_box(size_of_box)

        # initial state
        self.flight_state = States.MANUAL
        self.way_points_visited = 0

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        triggered on any incoming LOCAL_POSITION messages
        If a message comes in during take off, the function checks if the final attitude has been reached
        and if so, tells the drone to switch to waypoint finding
        If a message comes in during waypoint finding, the function checks if the drone has reached the
        waypoint and tells it to switch to the next task
        """

        if self.flight_state == States.TAKEOFF:
            altitude = -1 * self.local_position[2]
            if altitude > 0.95 * self.altitude:
                self.waypoint_transition()

        if self.flight_state == States.WAYPOINT:
            current_position = self.local_position
            current_position[2] *= -1
            print('flying to: ', self.waypoints[self.way_points_visited - 1])
            print('current position: ', current_position)
            distance_to_destination = np.linalg.norm(np.array(self.waypoints[self.way_points_visited - 1]) - current_position)
            if distance_to_destination < 0.5:
                self.waypoint_transition()

    def velocity_callback(self):
        """
        triggered on any incoming LOCAL_VELOCITY messages
        If a message comes in during landing, the function checks if the drone is near enough to the ground
        to switch to disarming
        """

        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        triggered on any incoming STATE messages
        If a message comes in during the manual fase, the function tells the drone to switch to arming
        If a message comes in during arming, the function tells the drone to switch to take-off
        If a message comes in during disarming, the function tells the drone to switch to manual mode
        """

        if not self.in_mission:
            # drone has landed and mission is finished
            return
        if self.flight_state == States.MANUAL:
            # from manual we arm the drone to start the motors
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            # after arming we tell the drone to start flying
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            # after disarming we tell the drone to return to manual mode and finish the mission
            self.manual_transition()

    def calculate_box(self, ds):
        """
        based on the desired altitude and the size of the box, calculates the waypoints to visit
        """

        return [[0, ds, self.altitude], [ds, ds, self.altitude],
                [ds, 0, self.altitude], [0, 0, self.altitude]]

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.takeoff(self.altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        if self.way_points_visited == 4:
            self.landing_transition()
        else:
            next_way_point = self.waypoints[self.way_points_visited]
            self.way_points_visited += 1
            self.cmd_position(*next_way_point, 0)
            if self.flight_state != States.WAYPOINT:
                self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn, 5, 10)
    time.sleep(2)
    drone.start()
