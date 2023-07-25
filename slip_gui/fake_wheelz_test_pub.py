import rclpy, numpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from retailbot_interfaces.msg import RoboteqInfo


class Fake_Wheelz(Node):


    def __init__(self):

        self.spin_resolution = 32 

        super().__init__('slip_gui_node')

        self.declare_parameter('roboteq_odom_topic', 'roboteq_odom')        
        self.declare_parameter('roboteq_info_topic', 'roboteq_info')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('pub_rate_hz', 10.0)

        self.fake_cmd_vel_pub = self.create_publisher(
            msg_type= Twist,
            topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
        self.fake_roboteq_odom_pub = self.create_publisher(
            msg_type= Odometry,
            topic= self.get_parameter('roboteq_odom_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
        self.fake_roboteq_info_pub = self.create_publisher(
            msg_type= RoboteqInfo,
            topic= self.get_parameter('roboteq_info_topic').get_parameter_value().string_value,
            qos_profile= 10
            )
        
        self.roboteq_odom_timer = self.create_timer(
            timer_period_sec = (1.0 / self.get_parameter('pub_rate_hz').get_parameter_value().double_value),
            callback = self.publish_fake_date
            )
        
        self.radian_list = numpy.linspace(
            start= 0.0,
            stop= math.radians(360.0),
            num= self.spin_resolution).tolist()
        
        self.radian_index = 0 
        
        
    def publish_fake_date(self):

        twist_msg = Twist()
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.radian_list[self.radian_index]

        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        self.fake_cmd_vel_pub.publish(twist_msg)

        odom_message = Odometry()

        odom_message.header.frame_id = 'fake'
        odom_message.child_frame_id = 'fake'
        self.fake_roboteq_odom_pub.publish(odom_message)

        info_msg = RoboteqInfo()
        query_cmds = [
            ("Motor Amps",""),
            ("Relative Encoder Count",""),
            ("Absolute Encoder Count",""),
            ("Closed Loop Error",""),
            ("Encoder RPM",""),
            ("Sensor Errors",""),
            ("Temperature",""),
            ("Firmware ID",""),
            ]

        for cmd_str, cmd in query_cmds:
            info_msg.data_description.append(cmd_str)
            info_msg.front_left_motor_data.append('fake')
            info_msg.back_left_motor_data.append('fake')
            info_msg.front_right_motor_data.append('fake')
            info_msg.back_right_motor_data.append('fake')
        
        self.fake_roboteq_info_pub.publish(info_msg)


        if self.radian_index < len(self.radian_list) - 1:
            self.radian_index += 1 
        else:
            self.radian_index = 0 




def main(args=None):

    rclpy.init(args=args)
    wheelz_not_real = Fake_Wheelz()
    rclpy.spin(wheelz_not_real)
    wheelz_not_real.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()