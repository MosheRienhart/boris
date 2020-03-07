#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
#import roslib
import tf.transformations
import tf_conversions
import tf2_ros

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from sensor_msgs.msg import JointState
import std_srvs.srv

#roslib.load_manifest('diagnostic_updater')
import diagnostic_updater, diagnostic_msgs.msg

import time
import math
import traceback
import Queue

from odrive_interface import ODriveInterfaceAPI, ODriveFailure
from odrive_interface import ChannelBrokenException, ChannelDamagedException
from odrive_simulator import ODriveInterfaceSimulator

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    
    # use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

class ODriveNode(object):
    #To adjust encoder to horizontal position for zero radians
    encoder_index_offset = -50 
    last_pos = 0.0
    driver = None
    prerolling = False
    yaw_angle_skip_queue_val = 0
    tilt_angle_skip_queue_val = 0
    new_pos_yaw = 0
    new_pos_tilt = 0
    # Robot params for radians -> cpr conversion
    #AS5048 -> 16384 Steps/rev (SPI/I2C interface)
    #AS5047P -> 4000 Steps/rev = 1000PPR (ABI Interface)
    #AS5047D -> 2000 Steps/rev = 500PPR (ABI Interface)
    axis_for_tilt = 1
    encoder_cpr = 2000
    
    # Startup parameters
    connect_on_startup = True
    calibrate_on_startup = False
    engage_on_startup = True
    
    publish_joint_angles = True
    # Simulation mode

    sim_mode = False
    
    def __init__(self):
        self.sim_mode             = get_param('simulation_mode', False)
        self.publish_joint_angles = get_param('publish_joint_angles', True) # if self.sim_mode else False
        self.publish_temperatures = get_param('publish_temperatures', True)
        
        self.axis_for_tilt = float(get_param('~axis_for_tilt', 0)) # if tilt calibrates first, this should be 0, else 1

        self.connect_on_startup   = get_param('~connect_on_startup', False)
        #self.calibrate_on_startup = get_param('~calibrate_on_startup', False)
        #self.engage_on_startup    = get_param('~engage_on_startup', False)
        
        self.has_preroll     = get_param('~use_preroll', False)
                
        self.publish_current = get_param('~publish_current', True)
        self.publish_raw_odom =get_param('~publish_raw_odom', True)
        
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)

        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',         std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',            std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',           std_srvs.srv.Trigger, self.release_motor)
                
        self.status_pub = rospy.Publisher('status', std_msgs.msg.String, latch=True, queue_size=2)
        self.status = "disconnected"
        self.status_pub.publish(self.status)
        
        self.command_queue = Queue.Queue(maxsize=1)

        # subscribed to simple radian angle message (x axis is tilt, z axis is yaw)
        self.gimbal_angle_subscribe = rospy.Subscriber("/cmd_gimbal_angle", Vector3, self.cmd_gimble_angle_callback, queue_size=2)

        self.publish_diagnostics = True
        if self.publish_diagnostics:
            self.diagnostic_updater = diagnostic_updater.Updater()
            self.diagnostic_updater.setHardwareID("Not connected, unknown")
            self.diagnostic_updater.add("ODrive Diagnostics", self.pub_diagnostics)
        
        if self.publish_temperatures:
            self.temperature_publisher_yaw  = rospy.Publisher('yaw/temperature', Float64, queue_size=2)
            self.temperature_publisher_tilt = rospy.Publisher('tilt/temperature', Float64, queue_size=2)
        
        self.i2t_error_latch = False
        if self.publish_current:
            #self.current_yawoop_count = 0
            #self.yaw_current_accumulator  = 0.0
            #self.tilt_current_accumulator = 0.0
            self.current_publisher_yaw  = rospy.Publisher('yaw/current', Float64, queue_size=2)
            self.current_publisher_tilt = rospy.Publisher('tilt/current', Float64, queue_size=2)
            self.i2t_publisher_yaw  = rospy.Publisher('yaw/i2t', Float64, queue_size=2)
            self.i2t_publisher_tilt = rospy.Publisher('tilt/i2t', Float64, queue_size=2)
            
            rospy.logdebug("ODrive will publish motor currents.")
            
            self.i2t_resume_threshold  = get_param('~i2t_resume_threshold',  222)            
            self.i2t_warning_threshold = get_param('~i2t_warning_threshold', 333)
            self.i2t_error_threshold   = get_param('~i2t_error_threshold',   666)
        
        self.last_cmd_gimble_angle_time = rospy.Time.now()
        
        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_yaw  = rospy.Publisher('yaw/raw_odom/encoder',   Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_encoder_tilt = rospy.Publisher('tilt/raw_odom/encoder',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_yaw      = rospy.Publisher('yaw/raw_odom/velocity',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_tilt     = rospy.Publisher('tilt/raw_odom/velocity', Int32, queue_size=2) if self.publish_raw_odom else None
            
        if self.publish_joint_angles:
            self.joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=2)
            
            jsm = JointState()
            self.joint_state_msg = jsm
            #jsm.name.resize(2)
            #jsm.position.resize(2)
            jsm.name = ['camera_yaw_joint','camera_tilt_joint']
            jsm.position = [0.0, 0.0]            

        
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        main_rate = rospy.Rate(1) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)
        
        self.fast_timer_comms_active = False
        
        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break;
            
            # fast timer running, so do nothing and wait for any errors
            if self.fast_timer_comms_active:
                continue
            
            # check for errors
            if self.driver:
                try:
                    # driver connected, but fast_comms not active -> must be an error?
                    # TODO: try resetting errors and recalibrating, not just a full disconnection
                    error_string = self.driver.get_errors(clear=True)
                    if error_string:
                        rospy.logerr("Had errors, disconnecting and retrying connection.")
                        rospy.logerr(error_string)
                        self.driver.disconnect()
                        self.status = "disconnected"
                        self.status_pub.publish(self.status)
                        self.driver = None
                    else:
                        # must have called connect service from another node
                        self.fast_timer_comms_active = True
                except (ChannelBrokenException, ChannelDamagedException, AttributeError):
                    rospy.logerr("ODrive USB connection failure in main_loop.")
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
                except:
                    rospy.logerr("Unknown errors accessing ODrive:" + traceback.format_exc())
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
            
            if not self.driver:
                if not self.connect_on_startup:
                    #rospy.loginfo("ODrive node started, but not connected.")
                    continue
                
                if not self.connect_driver(None)[0]:
                    rospy.logerr("Failed to connect.") # TODO: can we check for timeout here?
                    continue
                    
                if self.publish_diagnostics:
                    self.diagnostic_updater.setHardwareID(self.driver.get_version_string())
            
            else:
                pass # loop around and try again
        
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        # in case of failure, assume some values are zero
        self.vel_yaw = 0
        self.vel_tilt = 0
        self.current_tilt = 0
	self.current_yaw = 0
	self.temp_v_yaw = 0
	self.temp_v_tilt = 0
	self.motor_state_yaw = "not connected" # undefined
	self.motor_state_tilt = "not connected"
	self.bus_voltage = 0
        
        # Handle reading from Odrive and sending odometry
        if self.fast_timer_comms_active:
            try:
                # check errors
                error_string = self.driver.get_errors()
                if error_string:
                    self.fast_timer_comms_active = False
                else:
                    # reset watchdog
                    self.driver.feed_watchdog()
                    
                    # read all required values from ODrive for odometry
                    self.motor_state_yaw = self.driver.yaw_state()
                    self.motor_state_tilt = self.driver.tilt_state()
                    
                    self.encoder_cpr = self.driver.encoder_cpr
                
                    self.driver.update_time(time_now.to_sec())
                    self.vel_yaw = self.driver.yaw_vel_estimate()   # units: encoder counts/s
                    self.vel_tilt = -self.driver.tilt_vel_estimate() # neg is forward for tilt
                    self.new_pos_yaw = self.driver.yaw_pos()        # units: encoder counts
                    self.new_pos_tilt = -self.driver.tilt_pos()      # sign!
                    
                    # for temperatures
                    self.temp_v_yaw = self.driver.yaw_temperature()
                    self.temp_v_tilt = self.driver.tilt_temperature()
                    # for current
                    self.current_yaw = self.driver.yaw_current()
                    self.current_tilt = self.driver.tilt_current()
                    # voltage
                    self.bus_voltage = self.driver.bus_voltage()
                    
            except (ChannelBrokenException, ChannelDamagedException):
                rospy.logerr("ODrive USB connection failure in fast_timer." + traceback.format_exc(1))
                self.fast_timer_comms_active = False
                self.status = "disconnected"
                self.status_pub.publish(self.status)
                self.driver = None
            except:
                rospy.logerr("Fast timer ODrive failure:" + traceback.format_exc())
                self.fast_timer_comms_active = False
                
        # as required by SLAM
        if self.publish_temperatures:
            self.pub_temperatures()
        if self.publish_current:
            self.pub_current()
        if self.publish_joint_angles:
            self.pub_joint_angles(time_now)
        if self.publish_diagnostics:
            self.diagnostic_updater.update()
	if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_yaw.publish(self.new_pos_yaw)
            self.raw_odom_publisher_encoder_tilt.publish(self.new_pos_tilt)
            self.raw_odom_publisher_vel_yaw.publish(self.vel_yaw)
            self.raw_odom_publisher_vel_tilt.publish(self.vel_tilt)

	"""
        try:
            # check and stop motor if no pos command has been received in > 1s
            #if self.fast_timer_comms_active:
            if self.driver:
                if (time_now - self.last_cmd_gimble_angle_time).to_sec() > 0.5 and self.last_pos > 0:
                    self.driver.drive(0,0)
                    self.last_pos = 0
                    self.last_cmd_gimble_angle_time = time_now
                # release motor after 10s stopped
                if (time_now - self.last_cmd_gimble_angle_time).to_sec() > 10.0 and self.driver.engaged():
                    self.driver.release() # and release            
        except (ChannelBrokenException, ChannelDamagedException):
            rospy.logerr("ODrive USB connection failure in cmd_gimble_angle timeout." + traceback.format_exc(1))
            self.fast_timer_comms_active = False
            self.driver = None
        except:
            rospy.logerr("cmd_gimble_angle timeout unknown failure:" + traceback.format_exc())
            self.fast_timer_comms_active = False
	"""
        
        # handle sending drive commands.
        # from here, any errors return to get out
        if self.fast_timer_comms_active and not self.command_queue.empty():
            # check to see if we're initialised and engaged motor
            try:
                if not self.driver.has_prerolled(): #ensure_prerolled():
                    rospy.logwarn_throttle(5.0, "ODrive has not been prerolled, ignoring drive command.")
                    motor_command = self.command_queue.get_nowait()
                    return
            except:
                rospy.logerr("Fast timer exception on preroll." + traceback.format_exc())
                self.fast_timer_comms_active = False                
            try:
                motor_command = self.command_queue.get_nowait()
            except Queue.Empty:
                rospy.logerr("Queue was empty??" + traceback.format_exc())
                return
            
            if motor_command[0] == 'drive':
                try:
                    if self.publish_current and self.i2t_error_latch:
                        # have exceeded i2t bounds
                        return
                    
                    if not self.driver.engaged():
                        self.driver.engage()
                        self.status = "engaged"
                        
                    #yaw_angle_val, tilt_angle_val = motor_command[1]
                    self.driver.drive(self.yaw_angle_skip_queue_val - self.encoder_index_offset,self.tilt_angle_skip_queue_val - self.encoder_index_offset)
                    #self.last_pos = max(abs(yaw_angle_val), abs(tilt_angle_val))
                    self.last_cmd_gimble_angle_time = time_now
                except (ChannelBrokenException, ChannelDamagedException):
                    rospy.logerr("ODrive USB connection failure in drive_cmd." + traceback.format_exc(1))
                    self.fast_timer_comms_active = False
                    self.driver = None
                except:
                    rospy.logerr("motor drive unknown failure:" + traceback.format_exc())
                    self.fast_timer_comms_active = False

            elif motor_command[0] == 'release':
                pass
            # ?
            else:
                pass
    
    
    def terminate(self):
        self.fast_timer.shutdown()
        if self.driver:
            self.driver.release()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            return (False, "Already connected.")
        
        ODriveClass = ODriveInterfaceAPI if not self.sim_mode else ODriveInterfaceSimulator
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(tilt_axis=self.axis_for_tilt):
            self.driver = None
            #rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        #rospy.loginfo("ODrive connected.")
        
        # okay, connected,    
        self.fast_timer_comms_active = True
        
        self.status = "connected"
        self.status_pub.publish(self.status)
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        try:
            if not self.driver.disconnect():
                return (False, "Failed disconnection, but try reconnecting.")
        except:
            rospy.logerr('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.status = "disconnected"
            self.status_pub.publish(self.status_pub)
            self.driver = None
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            if not self.driver.preroll(wait=True):
                self.status = "preroll_fail"
                self.status_pub.publish(self.status)
                return (False, "Failed preroll.")
            
            self.status_pub.publish("ready")
            rospy.sleep(1)
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.has_prerolled():
            return (False, "Not prerolled.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
        
    
    # Helpers and callbacks
    
    def convert(self, yaw_radians, tilt_radians):

        # convert message radian values into CPR values applicable to Odrive

        yaw_angle_val  = int((yaw_radians/(2*math.pi)) * self.encoder_cpr)
        tilt_angle_val = int((tilt_radians/(2*math.pi)) * self.encoder_cpr)
    
        return yaw_angle_val, tilt_angle_val

    def cmd_gimble_angle_callback(self, msg):
        #rospy.loginfo("Received a /cmd_gimbal_angle message!")
        #rospy.loginfo("Yaw Axis: [%f]"%(msg.z))
        #rospy.loginfo("Tilt Axis: [%f]"%(msg.x))

        yaw_angle_val, tilt_angle_val = self.convert(msg.z, msg.x)
        #rospy.loginfo("Sending encoder set point!")
        #rospy.loginfo("Yaw Axis: [%f]"%(yaw_angle_val))
        #rospy.loginfo("Tilt Axis: [%f]"%(tilt_angle_val))
        
        #Insure motors never exceed half rotation
        if (yaw_angle_val > 500):
            yaw_angle_val = 500
            rospy.logwarn("Yaw Axis received cmd > 500 of [%f]"%(msg.z))
        elif (yaw_angle_val < -500):
            yaw_angle_val = -500
            rospy.logwarn("Yaw Axis received cmd < -500 of [%f]"%(msg.z))
        if (tilt_angle_val > 300):
            tilt_angle_val = 300
            rospy.logwarn("Tilt Axis received cmd > 300 of [%f]"%(msg.x))
        elif(tilt_angle_val < -300):
            tilt_angle_val = -300
            rospy.logwarn("Tilt Axis received cmd < -300 of [%f]"%(msg.x))
        try:
	    self.yaw_angle_skip_queue_val = yaw_angle_val
	    self.tilt_angle_skip_queue_val = tilt_angle_val
            drive_command = ('drive', (yaw_angle_val, tilt_angle_val))
            self.command_queue.put_nowait(drive_command)
        except Queue.Full:
            pass
            
        self.last_cmd_gimble_angle_time = rospy.Time.now()
        
    def pub_diagnostics(self, stat):
        stat.add("Status", self.status)
        stat.add("Motor state YAW", self.motor_state_yaw) 
        stat.add("Motor state TILT", self.motor_state_tilt)
        stat.add("FET temp YAW (C)", round(self.temp_v_yaw,1))
        stat.add("FET temp TILT (C)", round(self.temp_v_tilt,1))
        stat.add("Motor temp YAW (C)", "unimplemented")
        stat.add("Motor temp TILT (C)", "unimplemented")
        stat.add("Motor current YAW (A)", round(self.current_yaw,1))
        stat.add("Motor current TILT (A)", round(self.current_tilt,1))
        stat.add("Voltage (V)", round(self.bus_voltage,2))
        stat.add("Motor i2t YAW", round(self.yaw_energy_acc,1))
        stat.add("Motor i2t TILT", round(self.tilt_energy_acc,1))
        
        # https://github.com/ros/common_msgs/blob/jade-devel/diagnostic_msgs/msg/DiagnosticStatus.msg
        if self.status == "disconnected":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not connected")
        else:
            if self.i2t_error_latch:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "i2t overheated, drive ignored until cool")
            elif self.yaw_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "yaw motor over i2t warning threshold")
            elif self.yaw_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "yaw motor over i2t error threshold")
            elif self.tilt_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "tilt motor over i2t warning threshold")
            elif self.tilt_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "tilt motor over i2t error threshold")
            # Everything is okay:
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Running")
        
        
    def pub_temperatures(self):
        # https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972
        # https://discourse.odriverobotics.com/t/thermistors-on-the-odrive/813/7
        # https://www.digikey.com/product-detail/en/murata-electronics-north-america/NCP15XH103F03RC/490-4801-1-ND/1644682
        #p3 =  363.0
        #p2 = -459.2
        #p1 =  308.3
        #p0 =  -28.1
        #
        #vl = self.temp_v_yaw
        #vr = self.temp_v_tilt

        #temperature_l = p3*vl**3 + p2*vl**2 + p1*vl + p0
        #temperature_r = p3*vr**3 + p2*vr**2 + p1*vr + p0
        
        #print(temperature_l, temperature_r)
        
        self.temperature_publisher_yaw.publish(self.temp_v_yaw)
        self.temperature_publisher_tilt.publish(self.temp_v_tilt)
        
    # Current publishing and i2t calculation
    i2t_current_nominal = 2.0
    i2t_update_rate = 0.01
    
    def pub_current(self):
        self.current_publisher_yaw.publish(float(self.current_yaw))
        self.current_publisher_tilt.publish(float(self.current_tilt))
        
        now = time.time()
        
        if not hasattr(self, 'last_pub_current_time'):
            self.last_pub_current_time = now
            self.yaw_energy_acc = 0
            self.tilt_energy_acc = 0
            return
            
        # calculate and publish i2t
        dt = now - self.last_pub_current_time
        
        power = max(0, self.current_yaw**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.yaw_energy_acc *= 1 - self.i2t_update_rate * dt
        self.yaw_energy_acc += energy
        
        power = max(0, self.current_tilt**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.tilt_energy_acc *= 1 - self.i2t_update_rate * dt
        self.tilt_energy_acc += energy
        
        self.last_pub_current_time = now
        
        self.i2t_publisher_yaw.publish(float(self.yaw_energy_acc))
        self.i2t_publisher_tilt.publish(float(self.tilt_energy_acc))
        
        # stop odrive if overheated
        if self.yaw_energy_acc > self.i2t_error_threshold or self.tilt_energy_acc > self.i2t_error_threshold:
            if not self.i2t_error_latch:
                self.driver.release()
                self.status = "overheated"
                self.i2t_error_latch = True
                rospy.logerr("ODrive has exceeded i2t error threshold, ignoring drive commands. Waiting to cool down.")
        elif self.i2t_error_latch:
            if self.yaw_energy_acc < self.i2t_resume_threshold and self.tilt_energy_acc < self.i2t_resume_threshold:
                # have cooled enough now
                self.status = "ready"
                self.i2t_error_latch = False
                rospy.logerr("ODrive has cooled below i2t resume threshold, ignoring drive commands. Waiting to cool down.")
        
        
    #     current_quantizer = 5
    #
    #     self.yaw_current_accumulator += self.current_yaw
    #     self.tilt_current_accumulator += self.current_tilt
    #
    #     self.current_yawoop_count += 1
    #     if self.current_yawoop_count >= current_quantizer:
    #         self.current_publisher_yaw.publish(float(self.yaw_current_accumulator) / current_quantizer)
    #         self.current_publisher_tilt.publish(float(self.tilt_current_accumulator) / current_quantizer)
    #
    #         self.current_yawoop_count = 0
    #         self.yaw_current_accumulator = 0.0
    #         self.tilt_current_accumulator = 0.0       
    
    def pub_joint_angles(self, time_now):
        jsm = self.joint_state_msg
        jsm.header.stamp = time_now
        if self.driver:
            jsm.position[0] = (float(self.new_pos_yaw + self.encoder_index_offset)/self.encoder_cpr) * 2 * math.pi
            jsm.position[1] = (float(self.new_pos_tilt + self.encoder_index_offset)/self.encoder_cpr) * 2 * math.pi
            
        self.joint_state_publisher.publish(jsm)

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.main_loop()
    #rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass

