from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from retailbot_interfaces.msg import RoboteqInfo



def imu_to_csv(imu_msg: Imu):
    csv_rows = 2 
    csv_line_list: list[list[any]] = [ [] for i in range(csv_rows) ]
    twist_message_items = {
        "seconds" : str(imu_msg.header.stamp.sec),
        "nanoseconds" : str(imu_msg.header.stamp.nanosec),
        
        "orientation x (quaternions)" : str(imu_msg.orientation.x),
        "orientation y (quaternions)" : str(imu_msg.orientation.y),
        "orientation z (quaternions)" : str(imu_msg.orientation.z),
        "orientation w (quaternions)" : str(imu_msg.orientation.w),

        "angular velocity x (rad/sec)" : str(imu_msg.angular_velocity.x),
        "angular velocity y (rad/sec)" : str(imu_msg.angular_velocity.y),
        "angular velocity z (rad/sec)" : str(imu_msg.angular_velocity.z),

        "linear acceleration x (m/s^2)" : str(imu_msg.linear_acceleration.x),
        "linear acceleration y (m/s^2)" : str(imu_msg.linear_acceleration.y),
        "linear acceleration z (m/s^2)" : str(imu_msg.linear_acceleration.z),

        "frame_id" : str(imu_msg.header.frame_id),
    }
    for name, val in twist_message_items.items():
        csv_line_list[0].append(name)
        csv_line_list[1].append(val)
    return csv_line_list  



def twist_to_csv(twist_msg: Twist):
    csv_rows = 2 
    csv_line_list: list[list[any]] = [ [] for i in range(csv_rows) ]
    twist_message_items = {
        "x velocity linear (m/s)" : str(twist_msg.linear.x),
        "y velocity linear (m/s)" : str(twist_msg.linear.y),
        "z velocity linear (m/s)" : str(twist_msg.linear.z),

        "x velocity angular (rad/sec)" : str(twist_msg.angular.x),
        "y velocity angular (rad/sec)" : str(twist_msg.angular.y),
        "z velocity angular (rad/sec)" : str(twist_msg.angular.z),
    }
    for name, val in twist_message_items.items():
        csv_line_list[0].append(name)
        csv_line_list[1].append(val)
    return csv_line_list
    


def odometry_to_csv(odometry_msg: Odometry):
    csv_rows = 2 
    csv_line_list: list[list[any]] = [ [] for i in range(csv_rows) ]
    twist_message_items = {
        "seconds" : str(odometry_msg.header.stamp.sec),
        "nanoseconds" : str(odometry_msg.header.stamp.nanosec),

        "x position (meters)" : str(odometry_msg.pose.pose.position.x),
        "y position (meters)" : str(odometry_msg.pose.pose.position.y),
        "z position (meters)" : str(odometry_msg.pose.pose.position.z),

        "x rotation (quaternions)" : str(odometry_msg.pose.pose.orientation.x),
        "y rotation (quaternions)" : str(odometry_msg.pose.pose.orientation.y),
        "z rotation (quaternions)" : str(odometry_msg.pose.pose.orientation.z),
        "w rotation (quaternions)" : str(odometry_msg.pose.pose.orientation.w),

        "x velocity linear (m/s)" : str(odometry_msg.twist.twist.linear.x),
        "y velocity linear (m/s)" : str(odometry_msg.twist.twist.linear.y),
        "z velocity linear (m/s)" : str(odometry_msg.twist.twist.linear.z),

        "x velocity angular (ras/sec)" : str(odometry_msg.twist.twist.angular.x),
        "y velocity angular (ras/sec)" : str(odometry_msg.twist.twist.angular.y),
        "z velocity angular (ras/sec)" : str(odometry_msg.twist.twist.angular.z),

        "child_frame_id" : str(odometry_msg.child_frame_id),
        "frame_id" : str(odometry_msg.header.frame_id),
    }
    for name, val in twist_message_items.items():
        csv_line_list[0].append(name)
        csv_line_list[1].append(val)
    return csv_line_list



def roboteq_info_to_csv(roboteq_info_msg: RoboteqInfo):
    sec = [roboteq_info_msg.header.stamp.sec]
    nano = [roboteq_info_msg.header.stamp.nanosec]
    csv_line_list = [
        ["seconds"] + ["nanoseconds"] + roboteq_info_msg.data_description,
        sec + nano + roboteq_info_msg.front_left_motor_data,
        sec + nano + roboteq_info_msg.back_left_motor_data,
        sec + nano + roboteq_info_msg.front_right_motor_data,
        sec + nano + roboteq_info_msg.back_right_motor_data,
    ]   
    return csv_line_list 



# # # # # # # # # # # # # # # 
# Unused, still testing these:
# # # # # # # # # # # # # # # 



def _is_message_component(obj, attr):
    final_condition = True
    start_withs_to_avoid = ['SLOT_TYPES','__','_', 'imag','real']
    for srt in start_withs_to_avoid:
        final_condition = final_condition and not attr.startswith(srt)
    final_condition = final_condition and not callable(getattr(obj,attr))
    return final_condition



def _unpack_object_attrs(obj: object, attr_trace: str, attr_list: list[tuple], base_types_to_keep: list[type]):
    i = 0 
    for attr_str_name in dir(obj):
        input(f'searching through {attr_str_name},{i}')
        i += 1 
        # attr_trace = ''
        if _is_message_component(obj, attr_str_name):
            attr_obj = getattr(obj, attr_str_name)
            if any([isinstance(attr_obj,type) for type in base_types_to_keep]):
                print(f'old attr_list: {attr_list}')
                attr_list.append((attr_trace + '.' + attr_str_name, attr_obj))
                print(f'new attr_list: {attr_list}')
            else:
                attr_trace += '.' + attr_str_name
                _unpack_object_attrs(attr_obj, attr_trace, attr_list,base_types_to_keep)
    return attr_list



def unpack_object_attrs(obj: object, base_types_to_keep: list[type]):
    return _unpack_object_attrs(obj,'', [], base_types_to_keep)