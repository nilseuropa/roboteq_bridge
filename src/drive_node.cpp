#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Time         encoder_read_time, encoder_read_prev_time;
std_msgs::Float32 measuredLeftSpeed;
std_msgs::Float32 measuredRightSpeed;
int32_t           odom_encoder_left;
int32_t           odom_encoder_right;
serial::Serial    controller;
std::string       port;
char              odom_buf[24];
int               baud;
int               encoder_ppr;
int               watchdog_timeout;
int               motor_amp_limit;
int               motor_max_speed;
int               motor_max_acceleration;
int               motor_max_deceleration;
int               odom_idx=0;
bool              open_loop;
bool              report_angular_velocities;
bool              shouldPublishWheelVel = false;
bool              invert_left_motor = false;
bool              invert_right_motor = false;
double            gear_ratio;
double            wheel_radius;
double            wheel_separation;

#define PI 3.1415926535897932384626433832795

// All RPM values are revolution-per-minute

void configure_module(){

    // stop motors
    controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();

    // disable echo
    controller.write("^ECHOF 1\r");
    controller.flush();

    // set watchdog timeout
    std::stringstream wdt_cmd;
    wdt_cmd << "^RWD " << watchdog_timeout << "\r";
    controller.write(wdt_cmd.str());

    // set motor operating mode to closed-loop speed
    controller.write("^MMOD 1 1\r");
    controller.write("^MMOD 2 1\r");

    // set motor amps limit (A * 10)
    std::stringstream alim_cmd1;
    alim_cmd1 << "^ALIM 1 " << motor_amp_limit*10 << "\r";
    controller.write(alim_cmd1.str());
    std::stringstream alim_cmd2;
    alim_cmd2 << "^ALIM 2 " << motor_amp_limit*10 << "\r";
    controller.write(alim_cmd2.str());

    // set max motor speed (rpm) for relative speed commands
    std::stringstream mxrpm_cmd1;
    mxrpm_cmd1 << "^MXRPM 1 " << motor_max_speed << "\r";
    controller.write(mxrpm_cmd1.str());
    std::stringstream mxrpm_cmd2;
    mxrpm_cmd2 << "^MXRPM 2 " << motor_max_speed << "\r";
    controller.write(mxrpm_cmd2.str());

    // set max motor acceleration rate (rpm/s * 10)
    std::stringstream mac_cmd1;
    mac_cmd1 << "^MAC 1 " << motor_max_acceleration*10 << "\r";
    controller.write(mac_cmd1.str());
    std::stringstream mac_cmd2;
    mac_cmd2 << "^MAC 2 " << motor_max_acceleration*10 << "\r";
    controller.write(mac_cmd2.str());

    // set max motor deceleration rate (rpm/s * 10)
    std::stringstream mdec_cmd1;
    mdec_cmd1 << "^MDEC 1 " << motor_max_deceleration*10 << "\r";
    controller.write(mdec_cmd1.str());
    std::stringstream mdec_cmd2;
    mdec_cmd2 << "^MDEC 2 " << motor_max_deceleration*10 << "\r";
    controller.write(mdec_cmd2.str());

    // set PID parameters (gain * 10)
    controller.write("^KP 1 20\r");
    controller.write("^KP 2 20\r");
    controller.write("^KI 1 20\r");
    controller.write("^KI 2 20\r");
    controller.write("^KD 1 0\r");
    controller.write("^KD 2 0\r");

    // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
    controller.write("^EMOD 1 18\r");
    controller.write("^EMOD 2 34\r");

    // set encoder counts (ppr)
    std::stringstream right_enccmd;
    right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
    std::stringstream left_enccmd;
    left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
    controller.write(right_enccmd.str());
    controller.write(left_enccmd.str());

    // start encoder output (50 hz)
    controller.write("# C_?CR_# 20\r");
    controller.flush();
}

int32_t get_motor_rpm(float lin_speed) {
  return ((60.0 * lin_speed) / (wheel_radius * 2 * PI)) * gear_ratio; // wheel_rpm * gear_ratio
}

void send_motor_rpm(const int32_t motor_rpm, bool left) {
  std::stringstream cmd;
  if (left) {
	cmd << "!S 2 ";
  } else {
	cmd << "!S 1 ";
  }
  cmd << motor_rpm << "\r";
  controller.write(cmd.str());
}

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  float leftLinearVelocity  = (2.0f*twist.linear.x - wheel_separation*twist.angular.z)/2.0f;
  if (invert_left_motor) leftLinearVelocity*=-1.0f;
  float rightLinearVelocity = (2.0f*twist.linear.x + wheel_separation*twist.angular.z)/2.0f;
  if (invert_right_motor) rightLinearVelocity*=-1.0f;
  send_motor_rpm(get_motor_rpm(leftLinearVelocity) , true);
  send_motor_rpm(get_motor_rpm(rightLinearVelocity), false);
  controller.flush();
}

void left_wheel_cmd_cb(const std_msgs::Float32& left_speed){
  send_motor_rpm(get_motor_rpm(left_speed.data), true);
  controller.flush();
}

void right_wheel_cmd_cb(const std_msgs::Float32& right_speed){
  send_motor_rpm(get_motor_rpm(right_speed.data), false);
  controller.flush();
}

float calculate_wheel_vel(bool left) {
	encoder_read_time = ros::Time::now();
	int32_t encoder_steps_delta = 0;
	if (left) { encoder_steps_delta = odom_encoder_left; }
	else {      encoder_steps_delta = odom_encoder_right; }
	float dt = (encoder_read_time-encoder_read_prev_time).toSec();
	float encoder_cpr = encoder_ppr * 4;
	return ((float)encoder_steps_delta/encoder_cpr) / gear_ratio * (wheel_radius * 2 * PI) / dt;
}

void read_encoder_report() {
	if (controller.available()) {
		char ch = 0;
		if (controller.read((uint8_t*)&ch, 1) == 0) return; // Return if reading failed
		if (ch == '\r') { // End of message, interpret it
			odom_buf[odom_idx] = 0;
			// CR=... is an encoder count message
			if (odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=') {
				for (int p = 3; p < odom_idx; p++ ) { // Cycle through the buffer
					if (odom_buf[p] == ':') {
						odom_buf[p] = 0;
						odom_encoder_right = (int32_t)strtol(odom_buf+3, NULL, 10);
						odom_encoder_left =  (int32_t)strtol(odom_buf+p+1, NULL, 10);
						measuredLeftSpeed.data = calculate_wheel_vel(true);
						measuredRightSpeed.data = calculate_wheel_vel(false);
						if (report_angular_velocities) {
							// Report angular velocities for wheels, divide by wheel_radius;
							measuredLeftSpeed.data  /= wheel_radius;
							measuredRightSpeed.data /= wheel_radius;
						}
						shouldPublishWheelVel = true; // Indicate, that wheel vel has changed and should be published
						encoder_read_prev_time = encoder_read_time;
						break; // Quit for cycle
					}
				}
			}
			odom_idx = 0;
		} else if (odom_idx < (sizeof(odom_buf)-1) ) {
			// Accumulate characters in the buffer
			odom_buf[odom_idx++] = ch;
		}
	}
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "drive_node");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  ROS_INFO_STREAM("port: " << port);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("watchdog_timeout", watchdog_timeout, 100);
  ROS_INFO_STREAM("watchdog_timeout: " << watchdog_timeout);
  nhLocal.param("encoder_ppr", encoder_ppr, 2048); // CUI inc AMT 102-V, PPR pre-quadrature
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("gear_ratio", gear_ratio, 18.7); // (37/9)*(50/11) = 18.6868;
  ROS_INFO_STREAM("gear_ratio: " << gear_ratio);
  nhLocal.param("wheel_radius", wheel_radius, 0.165);
  ROS_INFO_STREAM("wheel_radius: " << wheel_radius);
  nhLocal.param("wheel_separation", wheel_separation, 0.56);
  ROS_INFO_STREAM("wheel_separation: " << wheel_separation);
  nhLocal.param("report_angular_velocities", report_angular_velocities, true); // r2 stack compatibility
  ROS_INFO_STREAM("report_angular_velocities: " << report_angular_velocities);
  nhLocal.param("invert_left_motor", invert_left_motor, false);
  ROS_INFO_STREAM("invert_left_motor: " << invert_left_motor);
  nhLocal.param("invert_right_motor", invert_right_motor, false);
  ROS_INFO_STREAM("invert_right_motor: " << invert_right_motor);
  nhLocal.param("motor_amp_limit", motor_amp_limit, 50); // Amps
  ROS_INFO_STREAM("motor_amp_limit: " << motor_amp_limit);
  nhLocal.param("motor_max_speed", motor_max_speed, 100); // rpm
  ROS_INFO_STREAM("motor_max_speed: " << motor_max_speed);
  nhLocal.param("motor_max_acceleration", motor_max_acceleration, 200); // rpm/s
  ROS_INFO_STREAM("motor_max_acceleration: " << motor_max_acceleration);
  nhLocal.param("motor_max_deceleration", motor_max_deceleration, 200); // rpm/s
  ROS_INFO_STREAM("motor_max_deceleration: " << motor_max_deceleration);

  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	controller.setPort(port);
	controller.setBaudrate(baud);
	controller.setTimeout(timeout);

  while ( ros::ok() ) {
		ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
		try {
			controller.open();
			if ( controller.isOpen() )
			{
				ROS_INFO("Successfully opened serial port");
				break;
			}
		}
		catch (serial::IOException e) {
			ROS_WARN_STREAM("serial::IOException: " << e.what());
		}
		ROS_WARN("Failed to open serial port");
		sleep( 5 );
	}
  configure_module();

  ros::Publisher  rightWheelVelocityPublisher = n.advertise<std_msgs::Float32>("/base/wheel_vel/right", 100);
  ros::Publisher  leftWheelVelocityPublisher  = n.advertise<std_msgs::Float32>("/base/wheel_vel/left", 100);
  ros::Subscriber rightWheelCmd = n.subscribe("/base/wheel_cmd/right", 100, right_wheel_cmd_cb);
  ros::Subscriber leftWheelCmd  = n.subscribe("/base/wheel_cmd/left", 100, left_wheel_cmd_cb);
  ros::Subscriber cmdVelCmd     = n.subscribe("/cmd_vel", 100, cmd_vel_cb);

  ROS_INFO("RobotEQ bridge started.");

  while (ros::ok()) {
    ros::spinOnce();
  	read_encoder_report();
  	if (shouldPublishWheelVel) {
  		rightWheelVelocityPublisher.publish(measuredRightSpeed);
  		leftWheelVelocityPublisher.publish( measuredLeftSpeed);
  		shouldPublishWheelVel = false;
  	}
  }

  if (controller.isOpen()) controller.close();
  ROS_INFO("RobotEQ bridge exiting.");
  return 0;
}
