# roboteq_bridge #
#### ROS serial bridge for robotEQ power modules ####

![](doc/modules.png)

### Protocol documentation ###
[Official robotEQ manual](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file)

### Dependencies ###
http://wiki.ros.org/serial

### Configuration ###
#### Node parameters: ####
```yaml
port: /dev/roboteq
baud: 115200
watchdog_timeout: 100 # topic and serial communication timeout [millisec]
encoder_ppr: 256 # pulses per revolutions on the motor axis encoder
gear_ratio: 55.0 # ratio by the motor axis rotations are slowed down to wheel axis
wheel_radius: 0.095 # driven wheel radius for differential drive [meter]
wheel_separation: 0.425 # distance of driven wheels for differential drive [meter]
motor_amp_limit: 10 # maximum current issued to motors [Amps]
motor_max_speed: 1000 # maximum speed issued to motors [RPM]
motor_max_acceleration: 2000 # [RPM / sec]
motor_max_deceleration: 2000 # [RPM / sec]
closed_loop: true # roboteq internal speed controller (PID)
enable_twist_input: false # differential drive on /cmd_vel
publish_velocities: true # wheel velocities [rad/sec]
publish_currents: true # motor currents Float32 topics [Amps]
current_lpf_tau: 0.5 # motor current low pass filter time constant
publish_battery_state: true # batteryState topic
battery_frame: base_link # frame_id for batteryState topic
aux_brake_control: true # whether to use automatic braking or enable external control topic on /brake
invert_brake_mode: true # digital output open drain config
brake_rpm_threshold: 0 # automatic brake release above desired command threshold [RPM]
brake_timeout: 500 # automatic brake lock after all motors received stop command
```

#### Parameters parsed to the module: ####
* watchdog_timeout - _milliseconds, stops drive after communication timeout_
* encoder_ppr - _number of pulses per encoder rotation_
* motor_amp_limit - _Amperes, internal current limit_
* motor_max_speed - _maximum allowed RPM on the encoder_
* motor_max_acceleration - _rpm/sec on encoder_
* motor_max_deceleration - _rpm/sec on encoder_

### Subscribers ###
* /cmd_vel - _twist command input_
* /brake - _brake control input (if not automatic)_
* /base/wheel_cmd/right - _independent direct speed input for the right wheel (report_angular_velocities)_
* /base/wheel_cmd/left - _independent direct speed input for the left wheel (report_angular_velocities)_

### Publishers ###
* /base/wheel_vel/right - _right wheel velocity (report_angular_velocities)_
* /base/wheel_vel/left - _left wheel velocity (report_angular_velocities)_
