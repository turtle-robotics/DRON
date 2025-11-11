/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
the intended behavior is for the drone to takeoff, fly a square with a 1 meter side length then land
**/

/**
 * @brief Offboard control square 2
 * @file offboard_control_square_2.cpp
 * @addtogroup examples
 * @author Chris Ambroziak 
 */

//px4 topics
#include <cstdint>
#include <px4_msgs/msg/detail/vehicle_local_position__struct.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/home_position.hpp>

//ros includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <stdint.h>

//these 3 includes are needed for the keyboard reading
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//general includes
#include <chrono>
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <algorithm>

//include for frame transformations
#include <GeographicLib/Geodesic.hpp>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::string, std::cin, std::cout, std::endl, std::vector;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control_square")
	{
        //get input for the number of drones to control
        cout << "enter the number of drones to control (1 or 2): " << endl;
        cin >> num_drones;
        if(num_drones < 1){
            RCLCPP_ERROR(this->get_logger(), "invalid number of drones, exiting program");
            rclcpp::shutdown();
        }

        const size_t drone_count = static_cast<size_t>(num_drones);

        current_positions.resize(drone_count);
        current_vehicle_statuses.resize(drone_count);
        land_detecteds.resize(drone_count);
        current_global_positions.resize(drone_count);
        home_positions.resize(drone_count);
        current_states.assign(drone_count, states_alias::TAKEOFF);
        states_reached.assign(drone_count, false);
        local_origins.resize(num_drones);
        current_attitudes.resize(drone_count);

    

        
		
        //create publisher and subscribers for the first drone, the first drone has no additional px4_# namespace in the topic name
        offboard_control_mode_publishers.push_back(this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10));
        trajectory_setpoint_publishers.push_back(this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10));
        vehicle_command_publishers.push_back(this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10));


        vehicle_attitude_subs.push_back(this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos_best_effort,
            [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
                current_attitudes[0] = *msg;
            }
        ));

        home_position_subs.push_back( this->create_subscription<HomePosition>(
            "/fmu/out/home_position_v1", qos_best_effort,
            [this](const HomePosition::UniquePtr msg){
                home_positions[0] = *msg;
            }
        ));
        

        vehicle_local_position_subs.push_back( this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos_best_effort,
            [this](const VehicleLocalPosition::UniquePtr msg) {
                //check if position data is valid before updating
                //this is important because if we try to use invalid data the drone will freak out
                if (msg->xy_valid) {
                    current_positions[0] = *msg;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Position data invalid! Not updating.");
                }
            }
        ));
        vehicle_status_subs.push_back( this->create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos_best_effort,
            [this](const VehicleStatus::UniquePtr msg) {
                current_vehicle_statuses[0] = *msg;
            }
        ));

        vehicle_land_detected_subs.push_back( this->create_subscription<VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", qos_best_effort,
            [this](const VehicleLandDetected::UniquePtr msg) {
                land_detecteds[0] = *msg;
            }
        ));

        vehicle_global_position_subs.push_back( this->create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos_best_effort,
            [this](const VehicleGlobalPosition::UniquePtr msg){
                current_global_positions[0] = *msg;
            }
        ));

        //create publishers for other drones
        //with a vector, i have to create the publisher then push it into the vector
        for(int i =1; i <num_drones; i++) {
            //create publishers for each drone
            offboard_control_mode_publishers.push_back(this->create_publisher<OffboardControlMode>("/px4_" + std::to_string(i) + "/fmu/in/offboard_control_mode", 10));
            trajectory_setpoint_publishers.push_back(this->create_publisher<TrajectorySetpoint>("/px4_" + std::to_string(i) + "/fmu/in/trajectory_setpoint", 10));
            vehicle_command_publishers.push_back(this->create_publisher<VehicleCommand>("/px4_" + std::to_string(i) + "/fmu/in/vehicle_command", 10));

            //create subscribers for each drone
            const size_t drone_index = static_cast<size_t>(i);
            vehicle_attitude_subs.push_back(this->create_subscription<px4_msgs::msg::VehicleAttitude>(
                "/px4_" + std::to_string(i) + "/fmu/out/vehicle_attitude", qos_best_effort,
                [this, drone_index](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
                    current_attitudes[drone_index] = *msg;
                }
            ));
            
            
            home_position_subs.push_back( this->create_subscription<HomePosition>(
                "/px4_" + std::to_string(i) + "/fmu/out/home_position_v1", qos_best_effort,
                [this, drone_index](const HomePosition::UniquePtr msg){
                    home_positions[drone_index] = *msg;
                }
            ));
            

            vehicle_local_position_subs.push_back( this->create_subscription<VehicleLocalPosition>(
                "/px4_" + std::to_string(i) + "/fmu/out/vehicle_local_position_v1", qos_best_effort,
                [this, drone_index](const VehicleLocalPosition::UniquePtr msg) {
                    //check if position data is valid before updating
                    //this is important because if we try to use invalid data the drone will freak out
                    if (msg->xy_valid) {
                        current_positions[drone_index] = *msg;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Position data invalid! Not updating.");
                    }
            }
            ));
            vehicle_status_subs.push_back( this->create_subscription<VehicleStatus>(
                "/px4_" + std::to_string(i) + "/fmu/out/vehicle_status_v1", qos_best_effort,
                [this, drone_index](const VehicleStatus::UniquePtr msg) {
                    current_vehicle_statuses[drone_index] = *msg;
            }
            ));

            
            vehicle_land_detected_subs.push_back( this->create_subscription<VehicleLandDetected>(
                "/px4_" + std::to_string(i) + "/fmu/out/vehicle_land_detected", qos_best_effort,
                [this, drone_index](const VehicleLandDetected::UniquePtr msg) {
                    land_detecteds[drone_index] = *msg;
            }
            ));

            vehicle_global_position_subs.push_back( this->create_subscription<VehicleGlobalPosition>(
                "/px4_" + std::to_string(i) + "/fmu/out/vehicle_global_position", qos_best_effort,
                [this, drone_index](const VehicleGlobalPosition::UniquePtr msg){
                    current_global_positions[drone_index] = *msg;
            }
            ));

        }

 

        //gets the origin of every drone. because we havent moved yet, this is our starting position
		


        
        
		//todo: implement a keyboard listeneer, if h or H is pressed, make all drones hover and pause the FSM.
		
		//todo: best practice is to stream setpoint commands for a few seconds before switching to offboard mode and arming, implement later as needed
        RCLCPP_INFO(this->get_logger(), "POI set to: [%.1f, %.1f, %.1f]", poi_x, poi_y, poi_z);
		
		// Set to offboard control mode, arm, and prepare for takeoff
		offboard_all_drones();
		publish_offboard_control_mode_all();
		
        //todo: publish arming_check_request and subscribe to arming_check_response to verify armed status before flying
		this->arm_all();
		
		
		

		auto timer_callback = [this]() -> void { // triggers every 200ms, think of this as our main "while" loop
            int offset = num_waypoints / num_drones;
			publish_offboard_control_mode_all(); //needs to be called regularly to maintain offboard mode

            if(!origins_initialized) {
                // In your constructor or initialization function, after you have current_positions populated:
                if (num_drones > 1) {
                    // Drone 0's origin is always (0,0,0) in its own frame
                    local_origins[0] = {0.0f, 0.0f, 0.0f};
                    
                    // Calculate other drones' origins relative to drone 0
                    for (int i = 1; i < num_drones; i++) {
                        local_origins[i] = calculate_origin_difference(current_positions[0], current_positions[i]);
                        
                        RCLCPP_INFO(this->get_logger(), 
                                "Drone %d origin relative to drone 0: [%.3f, %.3f, %.3f] meters", 
                                i, local_origins[i].x, local_origins[i].y, local_origins[i].z);
                    }
                    origins_initialized = true;
                } else {
                    // Single drone case
                    local_origins[0] = {0.0f, 0.0f, 0.0f};
                }
            }

       
            
			//if keyboard was pressed we get the input and check if it was q
			if(kb_press()){
				char c  = getchar();
				if( c == 'q' || c == 'Q'){
                    //if q was pressed we land all drones
					RCLCPP_INFO(this->get_logger(), "q was pressed, beginning land + shutoff sequence");
					
                    RTL_all(); //initiates RTl sequence
                    RTL_in_progress = true;
				    
			    }
            }

           
            /**TODO: this is scuffed because im speedrunning this for saturday, this will probably need to be implemented as a switch statement with a mini state
            machine divided into abstract mission progress states like TAKEOFF, MISSION, RTL, etc*/


            if(RTL_in_progress){
                
                //check if every vehicle status is disarmed, if they are do RCLCPP shutdown
                //if not disarmed check if landed, if landed disarm
                //if not landed continue to publish RTL_all()
                //std::all_of() returns true if every the predicate returns for all elements 
                all_disarmed = std::all_of(current_vehicle_statuses.begin(), current_vehicle_statuses.end(),
                    [](const VehicleStatus& status) {
                        return status.arming_state == status.ARMING_STATE_DISARMED;
                    });

                all_landed = std::all_of(land_detecteds.begin(), land_detecteds.end(),
                    [](const VehicleLandDetected& land) {
                        return land.landed;
                    });
                if(all_disarmed) {
                    RCLCPP_INFO(this->get_logger(), "attempting shutdown");
                    rclcpp::shutdown();
                } else if(all_landed) {
                    this->disarm_all();
                } else {
                    RTL_all(); //continue RTL
                }

                
            } else{
                
                if(takeoff_complete) { //starts as false;
                    RCLCPP_INFO(this->get_logger(), "TAKEOFF COMPLETED");
                    publish_trajectory_setpoint_local();
                    if(do_we_shift_states()){
                        for(int i =0; i < num_drones; i++) { //increases every state by 1, loops back to 1 after 4
                            current_states[i] = (states_alias)((int)current_states[i] % num_waypoints + 1);
                        }
                    }
                } else {
                    takeoff_all();
                    if(do_we_shift_states()) {//once takeoff is complete
                        takeoff_complete = true;
                        current_states[0] = states_alias::WAYPOINT_1;
                        for(int i =1; i < num_drones; i++){
                            //todo: this formula will fail if num_drones > num_waypoints, we will need to validate this during user input
                            current_states[i] = (states_alias)((int)current_states[0] + offset * i);
                        }					
                    }
                }
            }
				
				

		};
		timer_ = this->create_wall_timer(200ms, timer_callback); //think of this as the trigger for our main "for/while"
        //this timer MUST have atleast a 2Hz frequency or offboard mode will be exited
	}
	void arm_all();
	void disarm_all();
        
private:

	rclcpp::TimerBase::SharedPtr timer_;


    //publisher arrays to populate 
    std::vector<rclcpp::Publisher<OffboardControlMode>::SharedPtr> offboard_control_mode_publishers;
    std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> trajectory_setpoint_publishers;
    std::vector<rclcpp::Publisher<VehicleCommand>::SharedPtr> vehicle_command_publishers;
    

    //subscriber arrays to populate
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr> vehicle_attitude_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> vehicle_local_position_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> vehicle_status_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr> vehicle_land_detected_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr> vehicle_global_position_subs;
    std::vector<rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr> home_position_subs;
    
    std::vector<px4_msgs::msg::VehicleAttitude> current_attitudes;
    std::vector<VehicleLocalPosition> current_positions;
    std::vector<VehicleStatus> current_vehicle_statuses;
    std::vector<VehicleLandDetected> land_detecteds; 
    std::vector<VehicleGlobalPosition> current_global_positions;
    std::vector<HomePosition> home_positions;


	/**gives an alias to certain numbers, these aliases are used to "index" our state machine and determine
	where we are in the state machine */
	enum class states_alias {
		TAKEOFF = 0,
		WAYPOINT_1 = 1,
		WAYPOINT_2 = 2,
		WAYPOINT_3 = 3,
		WAYPOINT_4 = 4,
	};
    //todo: make num_waypoints change based on num of inputted coordinates (low priority, this is for the future once we take coordinates from input)
    int num_waypoints = 4;
    bool takeoff_complete = false;
	
    std::vector<states_alias> current_states;
    std::vector<bool> states_reached;

    //todo make the drone_id parameters not have a default value
    void publish_offboard_control_mode(int drone_id = 0);
    void publish_offboard_control_mode_all();
    void publish_trajectory_setpoint_local();
    void publish_vehicle_command(uint16_t command, int drone_id, float param1 = 0.0, float param2 = 0.0);
    void offboard_all_drones();
    void takeoff_all();
    bool waypoint_reached(int drone_id, bool takeoff_complete);
    bool do_we_shift_states();
    int kb_press();

    void RTL_all(); //TODO IMPLEMENT WHEN Q  IS PRESSED
    bool RTL_in_progress = false; //keep track of if we are in RTL
    bool all_disarmed = false;
    bool all_landed = false;

    //if the length is less than 2 meters the drones will fly really close to eachother, and may collide in real life
    float length = 2.0f; //length of the square sides in meters
    const float waypoint_radius = 0.15f; //waypoint radius in meters. this is used to determine when we consider a waypoint to be reached

    int num_drones = 1;//number of drones to control, default is 1

    struct Origins {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        
    };


    vector<Origins> local_origins; //vector to track NED origins of every drone
    bool origins_initialized = false;
    Origins calculate_origin_difference(const VehicleLocalPosition& ref1, const VehicleLocalPosition& ref2);
	
	

	rclcpp::QoS qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //some subscriptions must use best effort

    
    std::array<std::array<float,3>,6> state_machine = {{
		{0, 0, -5},          // TAKEOFF		0
		{length, 0, -5},     // WAYPOINT_1	1
		{2*length, 0, -5},     // WAYPOINT_2	2
		{2*length, length, -5},    // WAYPOINT_3	3
		{length, length, -5},    // WAYPOINT_4	4
	}};

	//type ros2 topic list to view all topics ros can see, you still have to subscribe to actually use them

    float poi_x = 1.5f * length;  // Center of the square in X
    float poi_y = 0.5f * length;  // Center of the square in Y  
    float poi_z = -5.0f;          // Same altitude as flight

    /**
     * @brief Calculate yaw angle to point toward the Point of Interest
     * @param drone_id ID of the drone
     * @return Yaw angle in radians (-π to π)
     */
    float calculate_yaw_to_poi(int drone_id) {
        // Get current position in shared frame
        float current_x = current_positions[drone_id].x + local_origins[drone_id].x;
        float current_y = current_positions[drone_id].y + local_origins[drone_id].y;
        
        // Calculate vector FROM DRONE TO POI
        float dx = poi_x - current_x;
        float dy = poi_y - current_y;
        
        // Calculate yaw angle - this is the direction the drone should face
        // In NED frame: 0 = North, π/2 = East, π = South, -π/2 = West
        float yaw = atan2(dy, dx);  // This gives the correct angle from North
        
        RCLCPP_DEBUG(this->get_logger(), "Drone %d at [%.1f, %.1f] to POI [%.1f, %.1f], yaw: %.3f rad (%.1f°)", 
                    drone_id, current_x, current_y, poi_x, poi_y, yaw, yaw * 180.0 / M_PI);
        return yaw;
    }

};

void OffboardControl::offboard_all_drones() {
    //go through each drone, switch it to offboard mode
    for(int i =0; i < num_drones; i++) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,i,  1, 6);
    }
}

/**
 * @brief Send a command to Arm all vehicles
 */
void OffboardControl::arm_all()
{
    for(int i =0; i < num_drones; i++) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,i,  1.0, 0 );
    }
	RCLCPP_INFO(this->get_logger(), "Arm commands send");
}

/**
 * @brief Send a command to Disarm all vehicles
 */
void OffboardControl::disarm_all()
{
    for(int i =0; i < num_drones; i++) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,i,  0.0, 0);
    }
	RCLCPP_INFO(this->get_logger(), "Disarm commands send");
}

void OffboardControl::RTL_all() {
    for(int i =0; i < num_drones; i++) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,i);
    }
    RCLCPP_INFO(this->get_logger(), "RTL commands sent");
}

/**
 * @brief Calculate the difference in meters between local frame origins
 * @param ref1 Reference coordinates of first drone (drone 0)
 * @param ref2 Reference coordinates of other drone
 * @return Origins structure containing x, y, z differences in meters (NED frame)
 */
OffboardControl::Origins OffboardControl::calculate_origin_difference(const VehicleLocalPosition& ref1, const VehicleLocalPosition& ref2)
{
    Origins diff;
    
    try {
        // Use GeographicLib to calculate geodesic between the two reference points
        GeographicLib::Geodesic geodesic(GeographicLib::Constants::WGS84_a(), 
                                       GeographicLib::Constants::WGS84_f());
        
        double s12;  // distance in meters
        double azi1, azi2;  // azimuths (bearings) at points 1 and 2
        
        // Calculate the forward geodesic from ref1 to ref2
        geodesic.Inverse(ref1.ref_lat, ref1.ref_lon, 
                        ref2.ref_lat, ref2.ref_lon,
                        s12, azi1, azi2);
        
        // Convert from geographic (azi1 = bearing from north) to NED frame:
        // - North (x) = s12 * cos(azi1)  
        // - East (y) = s12 * sin(azi1)
        // Note: azi1 is in degrees, measured clockwise from north
        double azi1_rad = azi1 * M_PI / 180.0;
        
        diff.x = static_cast<float>(s12 * std::cos(azi1_rad));  // North
        diff.y = static_cast<float>(s12 * std::sin(azi1_rad));  // East
        
        // Calculate altitude difference (NED frame: down is positive)
        // ref_alt is typically in meters AMSL (Above Mean Sea Level)
        diff.z = static_cast<float>(ref2.ref_alt - ref1.ref_alt);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Origin difference calculated: dx=%.3f, dy=%.3f, dz=%.3f meters", 
                    diff.x, diff.y, diff.z);
                    
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Error calculating origin difference: %s", e.what());
        // Return zero difference on error
        diff.x = 0.0f;
        diff.y = 0.0f;
        diff.z = 0.0f;
    }
    
    return diff; //returns the difference in NED frame as a Origins struct
}

/**
 * @brief determine if we need to shift to next state in the waypoint fsm
 * by calculated the distance to the target waypoint. returns true if we are within the waypoint radius
 */
bool OffboardControl::waypoint_reached(int drone_id, bool takeoff_complete) {
    // Calculate 3D distance to target waypoint
    float delta_x, delta_y, delta_z, target_x, target_y, target_z;

	int state_key = static_cast<int>(current_states[drone_id]);
    
    //for drone 1 it is flying to {2,-1,0}
    //gets target coordinates from the state machine
    //the state machine is default in the shared frame
    target_x = state_machine[state_key][0];
    target_y = state_machine[state_key][1];
    target_z = state_machine[state_key][2];  

	/** the missile knows where it is at all
	times. it knows this because it knows
	where it isn't, by subtracting where it
	is from where it isn't or where it isn't
	from where it is, whichever is greater, it
	obtains a difference or
	deviation. the guidance subsystem uses
	deviations to generate corrective
	commands to drive the missile from a
	position where it is to a position where
	it isn't and arriving at a position
	where it wasn't.  */

    //gets the current position in the shared frame
    float shared_x, shared_y, shared_z;
    if(takeoff_complete){ //if takeoff is complete we use the shared frame to check if weve reached the waypoint
        shared_x = current_positions[drone_id].x + local_origins[drone_id].x;
        shared_y = current_positions[drone_id].y + local_origins[drone_id].y;
        shared_z = current_positions[drone_id].z + local_origins[drone_id].z;
    } else{ //if we are taking off we dont care about shared frame, just go up 5 meters
        shared_x = current_positions[drone_id].x;
        shared_y = current_positions[drone_id].y;
        shared_z = current_positions[drone_id].z;
    }

    delta_x = target_x - shared_x;
    delta_y = target_y - shared_y;
    delta_z = target_z - shared_z;  
    
    
    float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    RCLCPP_INFO(this->get_logger(), "Drone %d data:", drone_id);
    RCLCPP_INFO(this->get_logger(), "Distance to waypoint %d: %.3f m", state_key, distance);
    RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f]", 
                shared_x, shared_y,shared_z);
    RCLCPP_INFO(this->get_logger(), "Target position: [%.2f, %.2f, %.2f]", 
                target_x, target_y, target_z);
    
    return (distance < waypoint_radius); // true if we need to shift states
}

/**
 * @brief check every drone with waypoint_reached() to see if every drone has reached its waypoint
    returns true if all drones have reached their waypoints
 */
bool OffboardControl::do_we_shift_states() {
    //when takeoff is not complete we dont check shared frame_only local one
    
    for(int i =0; i < num_drones; i++) {
        states_reached[i] = waypoint_reached(i, takeoff_complete);
    }

    for(int i = 0; i < num_drones; i++){
        if(!states_reached[i]){//if any state is not reached, we return false. it is not time to shift states
            return false;
        }
    }
    //if we get here, all drones have reached their waypoints and we can shift states
    return true;
    
}

void OffboardControl::publish_offboard_control_mode_all() {
    //go through each drone, publish offboard control mode
    for(int i =0; i < num_drones; i++) {
        this->publish_offboard_control_mode(i);
    }
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode(int drone_id)
{
	//because we are only doing position control with trajectory setpoint we only need to set the position member to true
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publishers[drone_id]->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint in local frame
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint_local()
{
    for(int i =0; i < num_drones; i++) {
        TrajectorySetpoint msg{};

        //this will be 0, 1, 2, or 3 depending on what point of the square we need to travel to
        int state_key = static_cast<int>(current_states[i]); //gets state key of each drone

        
        //position relative to the shared frame
        msg.position = {state_machine[state_key][0] - local_origins[i].x,
                        state_machine[state_key][1] - local_origins[i].y,
                        state_machine[state_key][2] - local_origins[i].z}; 
    
        if (current_states[i] != states_alias::TAKEOFF) {
            msg.yaw = calculate_yaw_to_poi(i);
            RCLCPP_INFO(this->get_logger(), "Drone %d facing POI at waypoint %d, yaw = %.3f rad", 
                       i, state_key, msg.yaw);
        }


        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publishers[i]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Drone %d data:", i);
        RCLCPP_INFO(this->get_logger(), "trajectory setpoint %d sent: position = [%f, %f, %f]",state_key,
                    msg.position[0], msg.position[1], msg.position[2]);
    }
}

/**
 * @brief takeoff to an altitude of 5 meters, must be armed and in offboard mode
 */
void OffboardControl::takeoff_all()
{
    //this is not transformed into shared frame because we want each to ascend 5 meters, not to the same 5 meter altitude position
    //essentially by not transforming we are preventing crashes
	for(int i = 0; i < num_drones; i++) {
        TrajectorySetpoint msg{};
        
        msg.position = {0.0, 0.0, -5.0}; // Takeoff to 5 meters altitude
        //msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publishers[i]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "takeoff command sent to drone %d", i);
    }
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, int drone_id, float param1, float param2)
{
	//todo
	//maybe give this a third parameter for which drone you want to send the command to?
	//consider subscribing to VehicleCommandAck to confirm the command was received
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	// msg.target_system = 1; //this targets px4 instance 0
    // msg.target_system = 2; //this targets px4 instance 1
    //i think we can also do msg.target_system = 0; because this is the broadcast signal
    msg.target_system  = drone_id + 1; //this will target the id of the drone we want, drone 0 needs target_system = 1
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publishers[drone_id]->publish(msg);
	// vehicle_command_publisher_1->publish(msg);
}
// there exists a frame_transform.h file in the px4_ros_com package that has a function to do this, look into using that

/**
 * @brief checks if a key was pressed
 */
int OffboardControl::kb_press() {
	//declares two terminal setting structs, oldt holds the current terminal settings so we can restore them
	//newt is a modified copy we will keep temporarily
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt); //fetch current terminal attributes for stdin into oldt
    newt = oldt; //copy into newt
    newt.c_lflag &= ~(ICANON | ECHO); //makes characters available immediately and echo prevents them from being printed to the terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); //apply modified attributes immediately to stdin

	
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);//query current file status flags on stdin file descriptor 
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); //set new file flags to the old flags plus nonblocking, and then restore old ones later

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); //restore old terminal attributes
    fcntl(STDIN_FILENO, F_SETFL, oldf); //restore old file status flags

    if (ch != EOF) {
		//puts the character back into the stdin stream so it can be read again
		//this will be used to determine which key was pressed so we can act accordingly
        ungetc(ch, stdin); 
        return 1;
    }
    return 0;
}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
