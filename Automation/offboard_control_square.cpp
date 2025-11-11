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
 * @brief Offboard control square
 * @file offboard_control_square.cpp
 * @addtogroup examples
 * @author Chris Ambroziak 
 */

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

//these 3 includes are needed for the keyboard reading
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


#include <chrono>
#include <iostream>
#include <string>



using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::string;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control_square")
	{
		//create publishers to we can actually send our messages to the drone
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		
		
		// below we create multiple subscriptions to get data about the drone
		vehicle_local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position_v1", qos_best_effort,
			[this](const VehicleLocalPosition::UniquePtr msg) {
				//check if position data is valid before updating
				//this is important because if we try to use invalid data the drone will freak out
				if (msg->xy_valid) {
					current_position_ = *msg;
				} else {
					RCLCPP_WARN(this->get_logger(), "Position data invalid! Not updating.");
				}
		});

		
		//used to determine if we are armed
		vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status_v1", qos_best_effort,
			[this](const VehicleStatus::UniquePtr msg) {
				current_vehicle_status = *msg;
		});

		//used to determine if we have landed 
		vehicle_land_detected_sub_ = this->create_subscription<VehicleLandDetected>(
			"/fmu/out/vehicle_land_detected", qos_best_effort,
			[this](const VehicleLandDetected::UniquePtr msg) {
				land_detected = *msg;
				
		});

		//lets create a subscription to the vehicle global position
		vehicle_global_position_sub_ = this->create_subscription<VehicleGlobalPosition>(
			"/fmu/out/vehicle_global_position", qos_best_effort,
			[this](const VehicleGlobalPosition::UniquePtr msg){
				current_global_position = *msg;
		});
		//gives EKF lat, long and alt. different from raw GPS lat, lon, alt

		//ignore this block of commented-out code, this was for testing purposes
		// vehicle_global_position_sub_ = this->create_subscription<VehicleGlobalPosition>(
		// 	"px4_1/fmu/out/vehicle_global_position", qos_best_effort,
		// 	[this](const VehicleGlobalPosition::UniquePtr msg){
		// 		current_global_position_1 = *msg;
		// });
		
			
		
		//todo
		//best practice is to stream setpoint commands for a few seconds before switching to offboard mode and arming, implement later as needed

		
		// Set to offboard control mode, arm, and prepare for takeoff
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		publish_offboard_control_mode();
		
		this->arm();
		
		// i ran this code and it says our starting gps position is 0,0,0 for the blank world simulation
		// RCLCPP_INFO(this->get_logger(), "global position for normal is lat: %f, lon: %f, alt: %f", current_global_position.lat, current_global_position.lon, current_global_position.alt);
		// RCLCPP_INFO(this->get_logger(), "global position for _1 is lat: %f, lon: %f, alt: %f", current_global_position_1.lat, current_global_position_1.lon, current_global_position_1.alt);
		
		
		

		auto timer_callback = [this]() -> void { // triggers every 200ms, this is our main "for/while" loop
			publish_offboard_control_mode(); //needs to be called regularly to maintain offboard mode

			//if keyboard was pressed we get the input and check if it was q
			if(kb_press()){
				char c  = getchar();
				if( c == 'q' || c == 'Q'){
					RCLCPP_INFO(this->get_logger(), "q was pressed, beginning land + shutoff sequence");
					current_state = states_alias::LAND;
				}
			}



			switch(current_state) { //starts at 0 for takeoff
				case states_alias::TAKEOFF:
					takeoff();
					if(do_we_shift_states()) {
						current_state = states_alias::WAYPOINT_1;						
					}
					break;
				case states_alias::WAYPOINT_1:
					publish_trajectory_setpoint_local(); //go to next waypoint
					if(do_we_shift_states()) {//checks if the drone is where it wants to be
						current_state = states_alias::WAYPOINT_2;						
					}
					break;
				case states_alias::WAYPOINT_2:
					publish_trajectory_setpoint_local();
					if(do_we_shift_states()) {
						current_state = states_alias::WAYPOINT_3;						
					}
					break;
				case states_alias::WAYPOINT_3:
					publish_trajectory_setpoint_local();
					if(do_we_shift_states()) {
						current_state = states_alias::WAYPOINT_4;						
					}
					break;
				case states_alias::WAYPOINT_4:
					publish_trajectory_setpoint_local();
						if(do_we_shift_states()) {
							current_state = states_alias::WAYPOINT_1;						
						}
						break;
				case states_alias::LAND:
					if(current_vehicle_status.arming_state == current_vehicle_status.ARMING_STATE_DISARMED) {
						RCLCPP_INFO(this->get_logger(), "attempting shutdown");
						rclcpp::shutdown();
					
					} else if(land_detected.landed) {
						this->disarm();	
						break;					
					} else{
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0);
						RCLCPP_INFO(this->get_logger(), "returning to launch");
					}

					
					break;
				default:
					publish_trajectory_setpoint_local();
					RCLCPP_WARN(this->get_logger(), "Lost track of states, fix your FSM");
					break;


			}

		};
		timer_ = this->create_wall_timer(200ms, timer_callback); //think of this as the trigger for our main "for/while"
	}
	void arm();
	void disarm();
        
private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
	
	std::atomic<uint64_t> timestamp_;   //idk why this line is here, it was in the example provided in the documentation
	VehicleLocalPosition current_position_;
	VehicleStatus current_vehicle_status;
	VehicleLandDetected land_detected; 
	VehicleGlobalPosition current_global_position, current_global_position_1;  

	/**gives an alias to certain numbers, these aliases are used to "index" our state machine and determine
	where we are in the state machine */
	enum class states_alias {
		TAKEOFF = 0,
		WAYPOINT_1 = 1,
		WAYPOINT_2 = 2,
		WAYPOINT_3 = 3,
		WAYPOINT_4 = 4,
		LAND = 5
	};
	states_alias current_state = states_alias::TAKEOFF; //starts off in the takeoff state

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint_local();
	void publish_trajectory_setpoint_body();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void takeoff();
	bool do_we_shift_states();
	int kb_press();

	
	float length = 1.0f; //length of the square sides in meters
	
	
	

	const float waypoint_radius = 0.15f; //waypoint radius in meters. this is used to determine when we consider a waypoint to be reached
	
	

	rclcpp::QoS qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //some subscriptions must use best effort


	//state machines to hold our waypoint coordinates
	std::array<std::array<float,3>,6> state_machine = {{
		{0, 0, -5},          // TAKEOFF		0
		{length, 0, -5},     // WAYPOINT_1	1
		{0, length, -5},     // WAYPOINT_2	2
		{-length, 0, -5},    // WAYPOINT_3	3
		{0, -length, -5},    // WAYPOINT_4	4
		{0, 0, 0}            // LAND	5
	}};
	//1, 1, 1 

	//type ros2 topic list to view all topics ros can see, you still have to subscribe to actually use them


};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	//param 1 = 1 means we arm the drone

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief determine if we need to shift to next state in the waypoint fsm
 * by calculated the distance to the target waypoint. returns true if we are within the waypoint radius
 */
bool OffboardControl::do_we_shift_states() {
    // Calculate 3D distance to target waypoint
    float delta_x, delta_y, delta_z, target_x, target_y, target_z;

	int state_key = static_cast<int>(current_state);
    
    //gets target coordinates from the state machine
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
    delta_x = target_x - current_position_.x;
    delta_y = target_y - current_position_.y;
    delta_z = target_z - current_position_.z;  
    
    
    float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    
    RCLCPP_INFO(this->get_logger(), "Distance to waypoint %d: %.3f m", state_key, distance);
    RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f]", 
                current_position_.x, current_position_.y, current_position_.z);
    RCLCPP_INFO(this->get_logger(), "Target position: [%.2f, %.2f, %.2f]", 
                target_x, target_y, target_z);
    
    return (distance < waypoint_radius); // true if we need to shift states
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	//because we are only doing position conrtol with trajectory setpoint we only need to set the position member to true
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint in local frame
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint_local()
{
    
	TrajectorySetpoint msg{};

	int state_key = static_cast<int>(current_state);
	
	//this will be 0, 1, 2, or 3 depending on what point of the square we need to travel to
	 

    //the position object needs floats so you cast integers to floats
	msg.position = {state_machine[state_key][0],
					state_machine[state_key][1],
					state_machine[state_key][2]};
	
   
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "trajectory setpoint %d sent: position = [%f, %f, %f]",state_key,
				 msg.position[0], msg.position[1], msg.position[2]);
}

/**
 * @brief takeoff to an altitude of 5 meters, must be armed and in offboard mode
 */
void OffboardControl::takeoff()
{
	
    TrajectorySetpoint msg{};
	
    msg.position = {0.0, 0.0, -5.0}; // Takeoff to 5 meters altitude
    //msg.yaw = -3.14;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "takeoff command sent");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	//todo
	//maybe give this a third parameter for which drone you want to send the command to?
	//consider subscribing to VehicleCommandAck to confirm the command was received
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1; //this targets px4 instance 0
	// msg.target_system = 2; //this targets px4 instance 1
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
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
