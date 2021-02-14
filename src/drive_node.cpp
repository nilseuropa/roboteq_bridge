#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <ctime>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>

/*
 * TODO:
 * * closed loop / open loop switch
 * * setting PID params ( dyanmic reconf )
 * * helper function for stream command settings
 * */

 // PID parameters (gain * 10)
 // controller.write("^KP 1 20\r");
 // controller.write("^KI 1 20\r");
 // controller.write("^KD 1 0\r");
 // controller.write("^KP 2 20\r");
 // controller.write("^KI 2 20\r");
 // controller.write("^KD 2 0\r");

ros::Publisher    right_wheel_vel_pub;
ros::Publisher    left_wheel_vel_pub;
ros::Publisher    right_motor_current_pub;
ros::Publisher    left_motor_current_pub;
ros::Publisher    battery_state_pub;

ros::Time         controller_response_time;
ros::Time         now;
ros::Time         left_stream_last_time;
ros::Time         right_stream_last_time;

ros::Timer        twist_command_watchdog;
ros::Timer        left_motor_cmd_watchdog;
ros::Timer        right_motor_cmd_watchdog;
ros::Timer        brake_timer;

std_msgs::Float32 measured_left_wheel_speed;
std_msgs::Float32 measured_right_wheel_speed;
std_msgs::Float32 current_amp_right;
std_msgs::Float32 current_amp_left;
sensor_msgs::BatteryState battery_state;

int32_t           odom_encoder_left;
int32_t           odom_encoder_right;
int32_t           current_decaamp_left;
int32_t           current_decaamp_right;
int32_t           battery_decavolts;
int32_t           controller_temp;

serial::Serial    controller;
std::string       port;
int               baud;
int               encoder_ppr;
int               watchdog_timeout;
int               motor_amp_limit;
int               motor_max_speed;
int               motor_max_acceleration;
int               motor_max_deceleration;
int               brake_io_channel;
int               brake_rpm_threshold;
int               brake_power_threshold;
bool              stream_initialized = false;
double            gear_ratio;
double            wheel_radius;
double            wheel_separation;
double            current_lpf_tau;
bool              channel_order_RL = true;
bool              closed_loop = true; // currently true is the only option ( we are setting rpms )
bool              publish_velocities = true;
bool              publish_currents = true;
bool              publish_battery_state = true;
bool              aux_brake_control = true;
bool              brake_state = true;
bool              invert_brake_mode = false;

void configure_module()
{
    // reset controller
    controller.write("^RESET\r");

    // stop motors
    controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();
    ROS_INFO("Controller reset. Full stop.");

    // disable echo
    controller.write("^ECHOF 1\r");
    controller.flush();
    ROS_INFO("Command echo off.");

    // set watchdog timeout
    std::stringstream wdt_cmd;
    wdt_cmd << "^RWD " << watchdog_timeout << "\r";
    controller.write(wdt_cmd.str());
    ROS_INFO("Watchdog configured.");

    // set motor amps limit (A * 10)
    std::stringstream alim_cmd1;
    alim_cmd1 << "^ALIM 1 " << motor_amp_limit*10 << "\r";
    controller.write(alim_cmd1.str());
    std::stringstream alim_cmd2;
    alim_cmd2 << "^ALIM 2 " << motor_amp_limit*10 << "\r";
    controller.write(alim_cmd2.str());
    ROS_INFO("Maximum motor current set: %f Amps", motor_amp_limit);

    // set max motor speed (rpm) for relative speed commands
    std::stringstream mxrpm_cmd1;
    mxrpm_cmd1 << "^MXRPM 1 " << motor_max_speed << "\r";
    controller.write(mxrpm_cmd1.str());
    std::stringstream mxrpm_cmd2;
    mxrpm_cmd2 << "^MXRPM 2 " << motor_max_speed << "\r";
    controller.write(mxrpm_cmd2.str());
    ROS_INFO("Maximum motor speed set: %d RPM", motor_max_speed);

    // set max motor acceleration rate (rpm/s * 10)
    std::stringstream mac_cmd1;
    mac_cmd1 << "^MAC 1 " << motor_max_acceleration*10 << "\r";
    controller.write(mac_cmd1.str());
    std::stringstream mac_cmd2;
    mac_cmd2 << "^MAC 2 " << motor_max_acceleration*10 << "\r";
    controller.write(mac_cmd2.str());
    ROS_INFO("Maximum motor acceleration set: %d RPM/sec", int32_t(motor_max_acceleration));

    // set max motor deceleration rate (rpm/s * 10)
    std::stringstream mdec_cmd1;
    mdec_cmd1 << "^MDEC 1 " << motor_max_deceleration*10 << "\r";
    controller.write(mdec_cmd1.str());
    std::stringstream mdec_cmd2;
    mdec_cmd2 << "^MDEC 2 " << motor_max_deceleration*10 << "\r";
    controller.write(mdec_cmd2.str());
    ROS_INFO("Maximum motor deceleration set: %d RPM/sec", int32_t(motor_max_deceleration));

    if (aux_brake_control)
    {
      // disable automatic digital output actions
      std::stringstream set_doa_cmd;
      set_doa_cmd << "^DOA " << brake_io_channel << " 0\r";
      controller.write(mdec_cmd2.str());
      ROS_INFO("Auxiliary brake control active.");
    }

    /*
    0: Open-loop
    1: Closed-loop speed
    2: Closed-loop position relative
    3: Closed-loop count position
    4: Closed-loop position tracking
    5: Closed-loop torque
    6: Closed-loop speed position
    */

    if (closed_loop)
    {
      // set motor operating mode to closed-loop speed
      controller.write("^MMOD 1 1\r");
      controller.write("^MMOD 2 1\r");
      // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
      controller.write("^EMOD 1 18\r");
      controller.write("^EMOD 2 34\r");
      ROS_INFO("Closed loop speed control.");
    }
    else
    {
      controller.write("^MMOD 1 0\r");
      controller.write("^MMOD 2 0\r");
    }

    // set encoder counts (ppr)
    std::stringstream right_enccmd;
    right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
    std::stringstream left_enccmd;
    left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
    controller.write(right_enccmd.str());
    controller.write(left_enccmd.str());
    ROS_INFO("Encoder resolution set.");

    // clear buffer history
    std::stringstream stream_command;
    stream_command << "# C\r";
    controller.write(stream_command.str());

    if (publish_velocities || publish_currents)
    {
        if (channel_order_RL)
        {
            stream_command.str(std::string());
            stream_command << R"(/"RM="," "?a 1_?s 1_# 20)" << "\r";
            controller.write(stream_command.str());
            stream_command.str(std::string());
            stream_command << R"(/"LM="," "?a 2_?s 2_# 20)" << "\r";
            controller.write(stream_command.str());
        }
        else
        {
            stream_command.str(std::string());
            stream_command << R"(/"RM="," "?a 2_?s 2_# 20)" << "\r";
            controller.write(stream_command.str());
            stream_command.str(std::string());
            stream_command << R"(/"LM="," "?a 1_?s 1_# 20)" << "\r";
            controller.write(stream_command.str());
        }
        ROS_INFO("Motor channel streams queried.");
    }

    if (publish_battery_state)
    {
        stream_command.str(std::string());
        stream_command << R"(/"BA="," "?v 2_?t 1_# 100)" << "\r";
        controller.write(stream_command.str());
        ROS_INFO("Battery state stream queried.");
    }
    controller.flush();
}

#define TWO_PI (2*M_PI)
#define TO_RAD_PER_SEC ((2*M_PI)/60)

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

int32_t linear_velocity_to_rpm(float lin_vel)
{
    return int32_t( (60.0f * lin_vel) / (wheel_radius * TWO_PI * gear_ratio) );
}

int32_t angular_velocity_to_rpm(float ang_vel)
{
    return int32_t((ang_vel / TWO_PI) * 60.0 * gear_ratio );
}

int32_t norm_to_power_cmd(float norm)
{
  return int32_t(map(norm, -1.0, 1.0, -2000, 2000));
}

void set_digital_output(const int32_t number, bool state)
{
    std::stringstream cmd;
    if (state)
    {
        cmd << "!D1 " << number << "\r";
    }
    else
    {
        cmd << "!D0 " << number << "\r";
    }
    controller.write(cmd.str());
    controller.flush();
}

void release_brake()
{
    set_digital_output(brake_io_channel, !invert_brake_mode);
}

void brake()
{
    set_digital_output(brake_io_channel, invert_brake_mode);
}

void update_brake_state(const ros::TimerEvent&)
{
    if (brake_state) brake(); else release_brake();
}

void brake_cb(const std_msgs::Bool state)
{
    brake_state = state.data;
}

int32_t right_motor_rpm, left_motor_rpm;
int32_t left_motor_power, right_motor_power;

void release_brake_on_rpm_threshold()
{
    if (!aux_brake_control)
    {
        // if any desired motor rpm is greater than a threshold, release the brakes
        if (abs(left_motor_rpm) > brake_rpm_threshold || abs(right_motor_rpm) > brake_rpm_threshold) // TODO: validate conditions
        {
            release_brake();
        }
        // otherwise start a timed brake lock callback
        else
        {
            brake_timer.stop();
            brake_timer.start();
        }
    }
}

void release_brake_on_power_threshold()
{
    if (!aux_brake_control)
    {
        // if any desired motor rpm is greater than a threshold, release the brakes
        if (abs(left_motor_power) > brake_power_threshold || abs(right_motor_power) > brake_power_threshold) // TODO: validate conditions
        {
            release_brake();
        }
        // otherwise start a timed brake lock callback
        else
        {
            brake_timer.stop();
            brake_timer.start();
        }
    }
}

void send_motor_power_right(const int32_t motor_power)
{
    std::stringstream cmd;
    if (channel_order_RL) {
        cmd << "!G 1 " << motor_power << "\r"; // Ch1: R, Ch2: L
    } else {
        cmd << "!G 2 " << motor_power << "\r";
    }
    controller.write(cmd.str());
    controller.flush();
    right_motor_power = motor_power;
    release_brake_on_power_threshold();
}

void send_motor_power_left(const int32_t motor_power)
{
    std::stringstream cmd;
    if (channel_order_RL) {
        cmd << "!G 1 " << motor_power << "\r"; // Ch1: R, Ch2: L
    } else {
        cmd << "!G 2 " << motor_power << "\r";
    }
    controller.write(cmd.str());
    controller.flush();
    left_motor_power = motor_power;
    release_brake_on_power_threshold();
}

void send_motor_rpm_right(const int32_t motor_rpm)
{
    std::stringstream cmd;
    if (channel_order_RL) {
        cmd << "!S 1 " << motor_rpm << "\r"; // Ch1: R, Ch2: L
    } else {
        cmd << "!S 2 " << motor_rpm << "\r";
    }
    controller.write(cmd.str());
    controller.flush();
    right_motor_rpm = motor_rpm;
    release_brake_on_rpm_threshold();
}

void send_motor_rpm_left(const int32_t motor_rpm)
{
    std::stringstream cmd;
    if (channel_order_RL) {
        cmd << "!S 2 " << motor_rpm << "\r"; // Ch1: R, Ch2: L
    } else {
        cmd << "!S 1 " << motor_rpm << "\r";
    }
    controller.write(cmd.str());
    controller.flush();
    left_motor_rpm = motor_rpm;
    release_brake_on_rpm_threshold();
}

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
    twist_command_watchdog.stop();
    float left_track_linear_velocity  = (2.0f * twist.linear.x - wheel_separation * twist.angular.z)/2.0f;
    float right_track_linear_velocity = (2.0f * twist.linear.x + wheel_separation * twist.angular.z)/2.0f;
    send_motor_rpm_left( linear_velocity_to_rpm(left_track_linear_velocity));
    send_motor_rpm_right(linear_velocity_to_rpm(right_track_linear_velocity));
    twist_command_watchdog.start();
}

void left_wheel_power_cmd_cb(const std_msgs::Float32& left_power)
{
    left_motor_cmd_watchdog.stop();
    send_motor_power_left(left_power.data);
    left_motor_cmd_watchdog.start();
}

void right_wheel_power_cmd_cb(const std_msgs::Float32& right_power)
{
    right_motor_cmd_watchdog.stop();
    send_motor_power_right(right_power.data);
    right_motor_cmd_watchdog.start();
}

void left_wheel_speed_cmd_cb(const std_msgs::Float32& left_speed_angular)
{
    left_motor_cmd_watchdog.stop();
    send_motor_rpm_left( angular_velocity_to_rpm(left_speed_angular.data));
    left_motor_cmd_watchdog.start();
}

void right_wheel_speed_cmd_cb(const std_msgs::Float32& right_speed_angular)
{
    right_motor_cmd_watchdog.stop();
    send_motor_rpm_right(angular_velocity_to_rpm(right_speed_angular.data));
    right_motor_cmd_watchdog.start();
}

float calculate_wheel_angular_velocity(int32_t motor_rpm)
{
    return ( ( (float)motor_rpm * TO_RAD_PER_SEC ) / gear_ratio );
}

// TODO: open loop stop
void twist_watchdog_cb(const ros::TimerEvent&)
{
    twist_command_watchdog.stop();
    // controller.write("!G 1 0\r");
    // controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();
}

void left_motor_watchdog_cb(const ros::TimerEvent&)
{
    left_motor_cmd_watchdog.stop();
    send_motor_rpm_left(0);
}

void right_motor_watchdog_cb(const ros::TimerEvent&)
{
    right_motor_cmd_watchdog.stop();
    send_motor_rpm_right(0);
}

void brake_watchdog_cb(const ros::TimerEvent&)
{
    brake_timer.stop();
    brake();
}

void read_roboteq_stream()
{
    std::string buffer = "";
    std::string lkey = "LM=";
    std::string rkey = "RM=";
    std::string bkey = "BA=";

    if (controller.available()>0)
    {
        controller.readline(buffer, 65536, "\r");
        if (
                (buffer.length() > 4) &&
                ((buffer.substr(0,3) == lkey) ||
                 (buffer.substr(0,3) == rkey) ||
                 (buffer.substr(0,3) == bkey))
                )
        {
            // Valid header keyword
            now = ros::Time::now();

            std::string data = buffer.substr(3);
            stream_initialized = true;
            std::stringstream ss(data);
            if (buffer.substr(0,3) == rkey) {
                ss >> current_decaamp_right >> odom_encoder_right;               // Extract current and encoder cnt.
                if (ss.fail()) {
                    // Invalid message
                    ROS_WARN("Right motor channel: unexpected data");
                    return;
                }
                double rc_lpf = (now-right_stream_last_time).toSec() / current_lpf_tau;
                current_amp_right.data = current_amp_right.data + rc_lpf * ((current_decaamp_right / 10.0) - current_amp_right.data);
                measured_right_wheel_speed.data = calculate_wheel_angular_velocity(odom_encoder_right);
                //if (measured_right_wheel_speed.data == 0) current_amp_right.data = 0;
                if (publish_velocities) right_wheel_vel_pub.publish(measured_right_wheel_speed);
                if (publish_currents) right_motor_current_pub.publish(current_amp_right);
                right_stream_last_time = now;
            }
            else if (buffer.substr(0,3) == lkey) {
                ss >> current_decaamp_left >> odom_encoder_left;
                if (ss.fail()) {
                    // Invalid message
                    ROS_WARN("Left motor channel: unexpected data");
                    return;
                }
                double lc_lpf = (now-left_stream_last_time).toSec() / current_lpf_tau;
                current_amp_left.data = current_amp_left.data + lc_lpf * ((current_decaamp_left / 10.0) - current_amp_left.data);
                measured_left_wheel_speed.data = calculate_wheel_angular_velocity(odom_encoder_left);
                //if (measured_left_wheel_speed.data == 0) current_amp_left.data = 0;
                if (publish_velocities) left_wheel_vel_pub.publish(measured_left_wheel_speed);
                if (publish_currents) left_motor_current_pub.publish(current_amp_left);
                left_stream_last_time = now;
            }
            else if (buffer.substr(0,3) == bkey) {
                ss >> battery_decavolts >> controller_temp;
                if (ss.fail()) {
                    // Invalid message
                    ROS_WARN("Battery channel: unexpected data");
                    return;
                }
                battery_state.header.stamp = now;
                battery_state.temperature = controller_temp;
                battery_state.current = -(fabs(current_amp_left.data) + fabs(current_amp_right.data));
                battery_state.voltage = battery_decavolts/10.0;
                if (publish_battery_state) battery_state_pub.publish(battery_state);
            }
        } else {
            return; // Return if reading failed
        }
        controller_response_time = now;
    }
    else if ( (ros::Time::now()-controller_response_time).toSec()*1000 > watchdog_timeout )
    {
        if (stream_initialized) ROS_WARN("Controller stream unavailable.");
        stream_initialized = false;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "drive_node");
    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");

    bool enable_twist_input;
    std::string battery_frame;
    int brake_timeout;
    double refresh_rate;
    nhLocal.param("refresh_rate", refresh_rate, 1000.0);
    nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
    nhLocal.param("baud", baud, 115200);
    nhLocal.param("watchdog_timeout", watchdog_timeout, 100);
    nhLocal.param("brake_io_channel", brake_io_channel, 1); // ?
    nhLocal.param("encoder_ppr", encoder_ppr, 2048); // CUI inc AMT 102-V, PPR pre-quadrature
    nhLocal.param("gear_ratio", gear_ratio, 18.7); // (37/9)*(50/11) = 18.6868;
    nhLocal.param("wheel_radius", wheel_radius, 0.165);
    nhLocal.param("wheel_separation", wheel_separation, 0.56);
    nhLocal.param("brake_rpm_threshold", brake_rpm_threshold, 0); // default is zero
    nhLocal.param("brake_power_threshold", brake_power_threshold, 0); // default is zero
    nhLocal.param("brake_timeout", brake_timeout, 500);
    nhLocal.param("current_lpf_tau", current_lpf_tau, 0.5);
    nhLocal.param("motor_amp_limit", motor_amp_limit, 50); // Amps
    nhLocal.param("motor_max_speed", motor_max_speed, 100); // rpm
    nhLocal.param("motor_max_acceleration", motor_max_acceleration, 200); // rpm/s
    nhLocal.param("motor_max_deceleration", motor_max_deceleration, 200); // rpm/s
    nhLocal.param("enable_twist_input",enable_twist_input, false);
    nhLocal.param("publish_velocities", publish_velocities, true);
    nhLocal.param("aux_brake_control", aux_brake_control, false);
    nhLocal.param("invert_brake_mode", invert_brake_mode, false);
    nhLocal.param("publish_currents", publish_currents, true);
    nhLocal.param("publish_battery_state", publish_battery_state, true);
    nhLocal.param("closed_loop",closed_loop,true);
    nhLocal.param("motor_channel_order_right_left", channel_order_RL, true); // True: RL, False: LR
    nhLocal.param<std::string>("battery_frame", battery_frame, "battery_link");

    // set up watchdogs
    serial::Timeout timeout = serial::Timeout::simpleTimeout(watchdog_timeout);
    twist_command_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), twist_watchdog_cb, true);
    left_motor_cmd_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), left_motor_watchdog_cb, true);
    right_motor_cmd_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), right_motor_watchdog_cb, true);
    brake_timer = n.createTimer(ros::Duration(brake_timeout/1000.0), brake_watchdog_cb, true);

    if (publish_battery_state)
    {
        battery_state_pub = n.advertise<sensor_msgs::BatteryState>("/battery", 10);
        battery_state.header.frame_id = battery_frame;
        battery_state.header.stamp = ros::Time::now();
        ROS_INFO("Advertising battery state.");
    }

    if (publish_velocities)
    {
        right_wheel_vel_pub = n.advertise<std_msgs::Float32>("/right_wheel/velocity", 10);
        left_wheel_vel_pub  = n.advertise<std_msgs::Float32>("/left_wheel/velocity",  10);
        ROS_INFO("Advertising wheel velocities.");
    }

    if (publish_currents)
    {
        right_motor_current_pub = n.advertise<std_msgs::Float32>("/right_motor/current", 10);
        left_motor_current_pub  = n.advertise<std_msgs::Float32>("/left_motor/current",  10);
        ROS_INFO("Advertising motor currents.");
    }

    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);

    while ( ros::ok() )
    {
        ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
        try
        {
            controller.open();
            if ( controller.isOpen() )
            {
                ROS_INFO("Successfully opened serial port.");
                break;
            }
        } catch (serial::IOException e)
        {
            ROS_ERROR_STREAM("serial::IOException: " << e.what());
        }
        ROS_ERROR("Failed to open serial port");
        ROS_WARN("Retry in 5 seconds.");
        sleep( 5 );
    }

    configure_module();
    ROS_INFO("Motor controller configured.");

    if (publish_battery_state || publish_velocities || publish_currents)
    {
      ROS_INFO("Waiting for stream...");
      while (ros::ok() && !stream_initialized)
      {
        read_roboteq_stream();
      }
      ROS_INFO("Stream registered.");
    }

    ros::Subscriber right_wheel_cmd;
    ros::Subscriber left_wheel_cmd;

    if (closed_loop)
    {
      right_wheel_cmd = n.subscribe("/right_wheel/command", 10, right_wheel_speed_cmd_cb);
      left_wheel_cmd  = n.subscribe("/left_wheel/command", 10, left_wheel_speed_cmd_cb);
    }
    else {
      right_wheel_cmd = n.subscribe("/right_motor/voltage_norm", 10, right_wheel_power_cmd_cb);
      left_wheel_cmd  = n.subscribe("/left_motor/voltage_norm", 10, left_wheel_power_cmd_cb);
    }

    ros::Subscriber twist_cmd_sub;
    if (enable_twist_input && closed_loop)
    {
        twist_cmd_sub = n.subscribe("/cmd_vel", 10, cmd_vel_cb);
        ROS_INFO("Twist input enabled.");
    }

    ros::Subscriber brake_cmd_sub;
    ros::Timer brake_refresh_timer;
    if (aux_brake_control)
    {
        brake_cmd_sub = n.subscribe("/brake", 10, brake_cb);
        brake_refresh_timer = n.createTimer(ros::Duration(0.01), update_brake_state);
        ROS_INFO("Auxiliary brake control enabled.");
    }
    brake_state = invert_brake_mode;
    brake();

    ros::Rate rate(refresh_rate);
    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    while (ros::ok())
    {
        ros::spinOnce();
        read_roboteq_stream();
        rate.sleep();
    }

    // Shutdown
    if (controller.isOpen()) controller.close();
    ROS_INFO("Drive node exiting...");
    return 0;
}
