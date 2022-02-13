from math import sin, cos
from sat_controller import SatControllerInterface, sat_msgs

# Team code is written as an implementation of various methods
# within the the generic SatControllerInterface class.
# If you would like to see how this class works, look in sat_control/sat_controller

# Specifically, init, run, and reset

class TeamController(SatControllerInterface):
    """ Team control code """

    def team_init(self):
        """ Runs any team based initialization """
        # Run any initialization you need

        # Persistant data
        self.counter = 0
        self.sequence = 0
        self.buffer_radius = 0.6
        self.docking_distance = 0.2
        #self.approach_speed = 0.3
        #self.docking_speed = 0.
        
        # Example of logging
        self.logger.info("Initialized :)")
        self.logger.warning("Warning...")
        self.logger.error("Error!")

        # Update team info
        team_info = sat_msgs.TeamInfo()
        team_info.teamName = "Capeture"
        team_info.teamID = 4

        # Return team info
        return team_info

    def team_run(self, system_state: sat_msgs.SystemState, satellite_state: sat_msgs.SatelliteState, dead_sat_state: sat_msgs.SatelliteState) -> sat_msgs.ControlMessage:
        """ Takes in a system state, satellite state """
        
        print(dead_sat_state)
        print(satellite_state)

        # Get timedelta from elapsed time
        elapsed_time = system_state.elapsedTime.ToTimedelta()
        self.logger.info(f'Elapsed time: {elapsed_time}')

        # Example of persistant data
        self.counter += 1

        # Example of logging
        self.logger.info(f'Counter value: {self.counter}')

        # Create a thrust command message
        control_message = sat_msgs.ControlMessage()

        # Move from point A to point B along a straight line (PID)
        if(self.sequence == 0):
            control_message.thrust.f_x = -2.0 * (satellite_state.pose.x - (dead_sat_state.pose.x + sin(dead_sat_state.pose.theta)*self.buffer_radius)) - 3.0 * satellite_state.twist.v_x
            control_message.thrust.f_y = -2.0 * (satellite_state.pose.y - (dead_sat_state.pose.y - cos(dead_sat_state.pose.theta)*self.buffer_radius)) - 3.0 * satellite_state.twist.v_y
            if (abs(satellite_state.pose.x - (dead_sat_state.pose.x + sin(dead_sat_state.pose.theta)*self.buffer_radius)) < 0.01 and abs(satellite_state.pose.y - (dead_sat_state.pose.y - cos(dead_sat_state.pose.theta)*self.buffer_radius)) < 0.01):
                self.sequence = 1
            return control_message

        # Rotate in point B to face dead satellite
        elif(self.sequence == 1):
            control_message.thrust.tau = - 0.3 * (satellite_state.pose.theta - dead_sat_state.pose.theta - 3.1415) - satellite_state.twist.omega
            if (abs(satellite_state.pose.theta - dead_sat_state.pose.theta) < 3.1415/180):
                self.sequence = 2
            return control_message
        
        # Dock to dead satellite at point C
        elif(self.sequence == 2):
            control_message.thrust.f_x = -2.0 * (satellite_state.pose.x - (dead_sat_state.pose.x + sin(dead_sat_state.pose.theta)*self.docking_distance)) - 3.0 * satellite_state.twist.v_x
            control_message.thrust.f_y = -2.0 * (satellite_state.pose.y - (dead_sat_state.pose.y - cos(dead_sat_state.pose.theta)*self.docking_distance)) - 3.0 * satellite_state.twist.v_y
            if (abs(satellite_state.pose.x - (dead_sat_state.pose.x + sin(dead_sat_state.pose.theta)*self.docking_distance)) < 0.01 and abs(satellite_state.pose.y - (dead_sat_state.pose.y - cos(dead_sat_state.pose.theta)*self.docking_distance)) < 0.01):
                self.sequence = 3
            return control_message
        
        # point B coordinates (after second move, R = exclusion zone radius + buffer)
        # x = dead_sat_state.pose.x + sin(dead_sat_state.pose.theta) * R
        # y = dead_sat_state.pose.y - cos(dead_sat_state.pose.theta) * R
        
    def team_reset(self) -> None:
        # Run any reset code
        pass