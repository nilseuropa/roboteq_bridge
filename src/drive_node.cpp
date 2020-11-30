#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <ctime>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Time         encoder_read_time, encoder_read_prev_time;
ros::Time         controller_response_time;
std_msgs::Float32 measured_left_wheel_speed;
std_msgs::Float32 measured_right_wheel_speed;
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
bool              odom_initialized = false;
bool              report_angular_velocities;
bool              should_publish_velocities = false;
bool              controller_is_reporting = false;
double            gear_ratio;
double            wheel_radius;
double            wheel_separation;

// All RPM values are revolution-per-minute

void configure_module(bool closed_loop){

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

    if (closed_loop){
      // TODO:
      // * add PID params
      // * set feedback
      // PID parameters (gain * 10)
      // controller.write("^KP 1 20\r");
      // controller.write("^KI 1 20\r");
      // controller.write("^KD 1 0\r");
      // controller.write("^KP 2 20\r");
      // controller.write("^KI 2 20\r");
      // controller.write("^KD 2 0\r");

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
    }

    controller.flush();
}

int32_t get_motor_rpm(float lin_speed) {
  return int32_t( ((60.0 * lin_speed) / (wheel_radius * 2 * M_PI)) * gear_ratio );
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
  controller.flush();
}

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  float left_track_linear_velocity  = (2.0f*twist.linear.x - wheel_separation*twist.angular.z)/2.0f;
  float right_track_linear_velocity = (2.0f*twist.linear.x + wheel_separation*twist.angular.z)/2.0f;
  send_motor_rpm(get_motor_rpm(left_track_linear_velocity) , true);
  send_motor_rpm(get_motor_rpm(right_track_linear_velocity), false);
}

void left_wheel_cmd_cb(const std_msgs::Float32& left_speed){
  send_motor_rpm(get_motor_rpm(left_speed.data), true);
}

void right_wheel_cmd_cb(const std_msgs::Float32& right_speed){
  send_motor_rpm(get_motor_rpm(right_speed.data), false);
}

float calculate_wheel_vel(bool left) {
	encoder_read_time = ros::Time::now();
	int32_t encoder_steps_delta = 0;
	if (left) { encoder_steps_delta = odom_encoder_left; }
	else {      encoder_steps_delta = odom_encoder_right; }
	float dt = (encoder_read_time-encoder_read_prev_time).toSec();
	float encoder_cpr = encoder_ppr * 4;
  // Indicate, that wheel vel has changed and should be published
  odom_initialized = bool(dt*1000 < 40); // encoder output (50 hz ~20ms) double taken
  if (odom_initialized) should_publish_velocities = true;
	return ((float)encoder_steps_delta/encoder_cpr) / gear_ratio * (wheel_radius * 2 * M_PI) / dt;
}

void read_encoder_report() {
	if (controller.available()) {
		char ch = 0;
		if (controller.read((uint8_t*)&ch, 1) == 0) {
      ROS_WARN("Controller read failed.");
      return; // Return if reading failed
    }
		if (ch == '\r') { // End of message, interpret it
			odom_buf[odom_idx] = 0;
			// CR=... is an encoder count message
			if (odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=') {

        char right_buf[10];
        int  left_idx = 0;
        bool msg_valid = false;

        for (int f = 3; f < odom_idx; f++ ) {
          if (odom_buf[f] != ':') {
            right_buf[f-3]=odom_buf[f];
          }
          else {
            msg_valid = true;
            left_idx = f+1;
            break;
          }
        }

        if (msg_valid){

          odom_encoder_right = (int32_t)strtol(right_buf, NULL, 10);
          measured_right_wheel_speed.data = calculate_wheel_vel(false);

          char left_buf[10];
          for (int p = left_idx; p < odom_idx; p++ ) {
            left_buf[p-left_idx] = odom_buf[p];
          }
          odom_encoder_left =  (int32_t)strtol(left_buf, NULL, 10);
          measured_left_wheel_speed.data  = calculate_wheel_vel(true);

          if (report_angular_velocities) {
            // Report angular velocities for wheels, divide by wheel_radius;
            measured_left_wheel_speed.data  /= wheel_radius;
            measured_right_wheel_speed.data /= wheel_radius;
          }

          encoder_read_prev_time = encoder_read_time;
        }
			}
			odom_idx = 0;
		}
    else if ( odom_idx < (sizeof(odom_buf)-1) ) {
			// Accumulate characters in the buffer
			odom_buf[odom_idx++] = ch;
		}
    controller_is_reporting = true;
    controller_response_time = ros::Time::now();
	}
  else if ( (ros::Time::now()-controller_response_time).toSec()*1000 > watchdog_timeout )
  {
    ROS_WARN("Encoder stream unavailable.");
    controller_is_reporting = false;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "drive_node");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  bool enable_twist_input, closed_loop;

  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  nhLocal.param("baud", baud, 115200);
  nhLocal.param("watchdog_timeout", watchdog_timeout, 100);
  nhLocal.param("encoder_ppr", encoder_ppr, 2048); // CUI inc AMT 102-V, PPR pre-quadrature
  nhLocal.param("gear_ratio", gear_ratio, 18.7); // (37/9)*(50/11) = 18.6868;
  nhLocal.param("wheel_radius", wheel_radius, 0.165);
  nhLocal.param("wheel_separation", wheel_separation, 0.56);
  nhLocal.param("report_angular_velocities", report_angular_velocities, true); // r2 stack compatibility
  nhLocal.param("motor_amp_limit", motor_amp_limit, 50); // Amps
  nhLocal.param("motor_max_speed", motor_max_speed, 100); // rpm
  nhLocal.param("motor_max_acceleration", motor_max_acceleration, 200); // rpm/s
  nhLocal.param("motor_max_deceleration", motor_max_deceleration, 200); // rpm/s
  nhLocal.param("enable_twist_input",enable_twist_input,true);
  nhLocal.param("closed_loop",closed_loop,true);

  serial::Timeout timeout = serial::Timeout::simpleTimeout(watchdog_timeout);
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
			ROS_ERROR_STREAM("serial::IOException: " << e.what());
		}
		ROS_ERROR("Failed to open serial port");
		sleep( 5 );
	}

  configure_module(closed_loop);
  while (ros::ok()&&!odom_initialized) {
    read_encoder_report();
  }
  ROS_INFO("Motor controller configured.");

  ros::Publisher  right_wheel_vel_pub;
  ros::Publisher  left_wheel_vel_pub;

  right_wheel_vel_pub = n.advertise<std_msgs::Float32>("/right_motor/velocity", 100);
  left_wheel_vel_pub  = n.advertise<std_msgs::Float32>("/left_motor/velocity", 100);

  ros::Subscriber right_wheel_vel_cmd = n.subscribe("/right_motor/command", 100, right_wheel_cmd_cb);
  ros::Subscriber leftt_wheel_vel_cmd = n.subscribe("/left_motor/command", 100, left_wheel_cmd_cb);

  ros::Subscriber twist_cmd_sub;
  if (enable_twist_input)
  {
    twist_cmd_sub = n.subscribe("/cmd_vel", 100, cmd_vel_cb);
    ROS_INFO("Subscribed to /cmd_vel");
  }

  ROS_INFO("Drive node started.");
  while (ros::ok()) {
    ros::spinOnce();

    // TODO: fault handling on "!controller_is_reporting"
    if (closed_loop) {
      read_encoder_report();
    	if (should_publish_velocities)
      {
    		right_wheel_vel_pub.publish(measured_right_wheel_speed);
    		left_wheel_vel_pub.publish( measured_left_wheel_speed);
    		should_publish_velocities = false;
    	}
    }
  }

  if (controller.isOpen()) controller.close();
  ROS_INFO("Drive node exiting...");
  return 0;
}
