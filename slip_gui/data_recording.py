from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from retailbot_interfaces.msg import RoboteqInfo

import csv, time, rclpy, threading, queue
from rclpy.node import Node

from object_attr_unpacker import twist_to_csv, odometry_to_csv, roboteq_info_to_csv, imu_to_csv

from slip_gui import Slip_Gui_TK


# 5 GB per minute  


class TopicRecorder(Node):

    def __init__(self, topic: str, msg_type: type, recording_file_path: str ):

        self.cmd_vel_sub = self.create_subscription(msg_type,topic,self.record_msg,10)
        self.need_file_header = True
        self.recording_file_path = recording_file_path
        self.make_header = True
        self.msg_queue = queue.Queue()
        self.lock = threading.Lock()
        self.event = threading.Event()

    def record_msg(self, msg):

        with self.lock:
            self.msg_queue.put(msg)

        msg_types = {
            Twist: twist_to_csv,
            Odometry : odometry_to_csv,
            Imu : imu_to_csv,
            RoboteqInfo : roboteq_info_to_csv
                }
        
        for type, func in msg_types.items():
            if isinstance(msg,type):
                to_be_written: list = func(msg)
                break
            else: 
                raise TypeError

        if self.need_file_header:
            to_be_written = to_be_written
            self.make_file_header = False
        else:
            to_be_written = to_be_written[1:]

        with open(self.recording_file_path, "a+") as file:
            write = csv.writer(file)
            write.writerows(to_be_written)


class DataRecordingNode(Node):


    def __init__(self):

        super().__init__('slip_gui_node')

        self.declare_parameter('data_target_directory', 'src/slip_detection/slip_gui/slip_gui/fake_test_data/') # Directory csv files are saved to 
        self.declare_parameter('autofill_file_names', True) # False if want to add note to saved file name 
        self.declare_parameter('data_write_depth', 100) # Number of data points kept in buffer to file writer 
        self.declare_parameter('data_list_depth', 10) # Number of data points kept natively in GUI 

        self.declare_parameter('roboteq_odom_topic', 'roboteq_odom') # Odometry topic put out by robotqes  
        self.declare_parameter('roboteq_info_topic', 'roboteq_info') # Motor info topic put out by robotqes
        self.declare_parameter('cmd_vel_topic', 'cmd_vel') # Twist topic roboteqs listen to

        self.declare_parameter('precent_messages_written', 0.10) # Increments of 10 
        self.declare_parameter('debugging', True)

        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            callback= self.cmd_vel_recorder,
            qos_profile= 10
            )
        
        self.roboteq_odom_sub = self.create_subscription(
            msg_type= Odometry,
            topic= self.get_parameter('roboteq_odom_topic').get_parameter_value().string_value,
            callback= self.roboteq_odom_recorder,
            qos_profile= 10
            )
        
        self.roboteq_info_pub = self.create_subscription(
            msg_type= RoboteqInfo,
            topic= self.get_parameter('roboteq_info_topic').get_parameter_value().string_value,
            callback= self.roboteq_info_recorder,
            qos_profile= 10
            )
        
        self.imu_sub = self.create_subscription(
            msg_type= Imu,
            topic= self.get_parameter('/zed2i/zed_node/imu/data').get_parameter_value().string_value,
            callback= self.roboteq_info_recorder,
            qos_profile= 10
            )
        
        self.time_string = time.strftime("%Y_%m_%d_%H_%M_%S")
        self.ext: str = '.csv'

        self.custom_file_str: str = ''

        if (not self.get_parameter('autofill_file_names').get_parameter_value().bool_value):
            self.custom_file_str = input("Enter custom file name: ") + "_"

        self.make_info_header = True
        self.make_odom_header = True
        self.make_cmd_vel_header = True 


        # Create thread-safe queues to store data
        self.cmd_vel_queue = queue.Queue()
        self.roboteq_odom_queue = queue.Queue()
        self.roboteq_info_queue = queue.Queue()

        # Create thread-safe locks for the data lists
        self.cmd_vel_lock = threading.Lock()
        self.odom_lock = threading.Lock()
        self.info_lock = threading.Lock()

        # Create thread-safe events to notify the GUI about new data
        self.cmd_vel_event = threading.Event()
        self.odom_event = threading.Event()
        self.info_event = threading.Event()


    def update_list(self, new_val, list: list):
        if len(list) >= self.get_parameter('data_list_depth').get_parameter_value().integer_value:
            list.pop(0)
        list.append(new_val)

  
    def cmd_vel_recorder(self, cmd_vel_msg):
            # Enqueue the data using the queue
            self.cmd_vel_queue.put(cmd_vel_msg)
            self.record_info(cmd_vel_msg, 
                             self.cmd_vel_queue, 
                             self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
                             self.cmd_vel_list_lock)


    def roboteq_odom_recorder(self, roboteq_odom_msg):
        # Enqueue the data using the queue
        self.roboteq_odom_queue.put(roboteq_odom_msg)
        self.record_info(roboteq_odom_msg, 
                         self.roboteq_odom_queue, 
                         self.get_parameter('roboteq_odom_topic').get_parameter_value().string_value,
                         self.odom_list_lock
                         )


    def roboteq_info_recorder(self, roboteq_info_msg):
        # Enqueue the data using the queue
        self.roboteq_info_queue.put(roboteq_info_msg)
        self.record_info(roboteq_info_msg, 
                         self.roboteq_info_queue, 
                         self.get_parameter('roboteq_info_topic').get_parameter_value().string_value,
                         self.info_list_lock)
        

    def record_info(self, message: any, message_queue: queue.Queue, topic_name: str, queue_lock: threading.Lock ):
            
            with queue_lock:
                message_list = list(message_queue.queue)

            if self.get_parameter('debugging').get_parameter_value().bool_value:
                self.get_logger().info("message list length: " + str(len(message_list)))

            percent_msgs_written = self.get_parameter('precent_messages_written').get_parameter_value().double_value
            assert percent_msgs_written < 1.0, "requested percentage of messages kept is more than 100%"

            dir: str = self.get_parameter('data_target_directory').get_parameter_value().string_value
            file_path = dir + self.time_string + '_' + self.custom_file_str + topic_name + self.ext

            if len(message_list) >= self.get_parameter('data_write_depth').get_parameter_value().integer_value:

                if self.get_parameter('debugging').get_parameter_value().bool_value:
                    self.get_logger().info("Writing to file. message list: " + str(message_list))
                written_list = [] 

                if isinstance(message, Twist):
                    for msg in message_list:
                        to_be_written = twist_to_csv(msg)
                        if self.make_cmd_vel_header:
                            written_list.extend(to_be_written)
                            self.make_cmd_vel_header = False 
                        else:
                            written_list.extend(to_be_written[1:])

                elif isinstance(message, Odometry):
                    for msg in message_list:
                        to_be_written = odometry_to_csv(msg)
                        if self.make_odom_header:
                            written_list.extend(to_be_written)
                            self.make_odom_header = False 
                        else:
                            written_list.extend(to_be_written[1:])

                elif isinstance(message, RoboteqInfo):
                    for msg in message_list:
                        to_be_written = roboteq_info_to_csv(msg)
                        if self.make_info_header:
                            written_list.extend(to_be_written)
                            self.make_info_header = False 
                        else:
                            written_list.extend(to_be_written[1:])
                else:
                    raise TypeError
                

                print(f'{written_list}')
                with open(file_path, "a+") as file:
                    write = csv.writer(file)
                    write.writerows(written_list)
                    written_list = []
                    message_list = []
            else:
                message_list.append(message)


# def main(args=None):
#     rclpy.init(args=args)
#     recording = DataRecordingNode()

#     slip_gui = Slip_Gui_TK(recording)
#     gui_thread = threading.Thread(target=slip_gui.fetch_data)
#     gui_thread.start()

#     while rclpy.ok():
#         rclpy.spin_once(recording)

#         if recording.get_parameter('debugging').get_parameter_value().bool_value:
#             logged_string: str = f''
#             for queue in [recording.roboteq_info_queue,recording.roboteq_odom_queue,recording.cmd_vel_queue]:
#                 logged_string += f'\n{queue.queue}'
#             recording.get_logger().info(logged_string)
#             # recording.get_logger().info(f'\n{recording.cmd_vel_queue}\n{recording.roboteq_odom_queue}\n{recording.roboteq_info_queue}')
#             # recording.get_logger().info(f'\n{len(recording.cmd_vel_queue)}\n{len(recording.roboteq_odom_queue)}\n{len(recording.roboteq_info_queue)}')

#     recording.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    recording = TopicRecorder()

    
    while rclpy.ok():

        rclpy.spin_once(recording)

       
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


















































































"""
Keeping this to come back to. Not best
practice, but easiest option. Will address 
sometime soon. k bye. 
"""
















# from PyQt5 import QtWidgets
# import pyqtgraph as pg 
# from pyqtgraph import PlotWidget, plot
# import sys  # We need sys so that we can pass argv to QApplication
# import os

# import csv, time, rclpy, threading, math
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from retailbot_interfaces.msg import RoboteqInfo
# from .object_attr_unpacker import unpack_object_attrs


# from PyQt5 import QtWidgets, QtCore
# import pyqtgraph as pg
# from pyqtgraph import PlotWidget
# import sys

# import tkinter as tk
# from tkinter import ttk
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# import numpy as np

# import queue


# # All of these should return a list of tuples for corresponding message content names and values 
# def twist_to_csv(twist_msg: Twist):
#     csv_rows = 2 

#     csv_line_list: list[list[any]] = [ [] for i in range(csv_rows) ]
#     twist_message_items = {
#         "x velocity linear" : str(twist_msg.linear.x),
#         "y velocity linear" : str(twist_msg.linear.y),
#         "z velocity linear" : str(twist_msg.linear.z),
#         "x velocity angular" : str(twist_msg.angular.x),
#         "y velocity angular" : str(twist_msg.angular.y),
#         "z velocity angular" : str(twist_msg.angular.z),
#     }

#     for name, val in twist_message_items.items():
#         csv_line_list[0].append(name)
#         csv_line_list[1].append(val)

#     return csv_line_list
    

# def odometry_to_csv(odometry_msg: Odometry):
#     csv_rows = 2 

#     csv_line_list: list[list[any]] = [ [] for i in range(csv_rows) ]
#     twist_message_items = {
#         "child_frame_id" : str(odometry_msg.child_frame_id),
#         "frame_id" : str(odometry_msg.header.frame_id),
#         "seconds" : str(odometry_msg.header.stamp.sec),
#         "nanoseconds" : str(odometry_msg.header.stamp.nanosec),

#         "x position" : str(odometry_msg.pose.pose.position.x),
#         "y position" : str(odometry_msg.pose.pose.position.y),
#         "z position" : str(odometry_msg.pose.pose.position.z),

#         "x quat rot" : str(odometry_msg.pose.pose.orientation.x),
#         "y quat rot" : str(odometry_msg.pose.pose.orientation.y),
#         "z quat rot" : str(odometry_msg.pose.pose.orientation.z),
#         "w quat rot" : str(odometry_msg.pose.pose.orientation.w),

#         "x velocity linear" : str(odometry_msg.twist.twist.linear.x),
#         "y velocity linear" : str(odometry_msg.twist.twist.linear.y),
#         "z velocity linear" : str(odometry_msg.twist.twist.linear.z),

#         "x velocity angular" : str(odometry_msg.twist.twist.angular.x),
#         "y velocity angular" : str(odometry_msg.twist.twist.angular.y),
#         "z velocity angular" : str(odometry_msg.twist.twist.angular.z),
#     }
#     for name, val in twist_message_items.items():
#         csv_line_list[0].append(name)
#         csv_line_list[1].append(val)

#     return csv_line_list


# def roboteq_info_to_csv(roboteq_info_msg: RoboteqInfo):

#     sec = [roboteq_info_msg.header.stamp.sec]
#     nano = [roboteq_info_msg.header.stamp.nanosec]

#     csv_line_list = [
#         ["seconds"] + ["nanoseconds"] + roboteq_info_msg.data_description,
#         sec + nano + roboteq_info_msg.front_left_motor_data,
#         sec + nano + roboteq_info_msg.back_left_motor_data,
#         sec + nano + roboteq_info_msg.front_right_motor_data,
#         sec + nano + roboteq_info_msg.back_right_motor_data,
#     ]   
#     return csv_line_list 


# class DataRecordingNode(Node):


#     def __init__(self):

#         super().__init__('slip_gui_node')

#         self.declare_parameter('data_target_directory', 'src/slip_detection/slip_gui/slip_gui/fake_test_data/') # Directory csv files are saved to 
#         self.declare_parameter('autofill_file_names', True) # False if want to add note to saved file name 
#         self.declare_parameter('data_write_depth', 100) # Number of data points kept in buffer to file writer 
#         self.declare_parameter('data_list_depth', 10) # Number of data points kept natively in GUI 


#         self.declare_parameter('roboteq_odom_topic', 'roboteq_odom') # Odometry topic put out by robotqes  
#         self.declare_parameter('roboteq_info_topic', 'roboteq_info') # Motor info topic put out by robotqes
#         self.declare_parameter('cmd_vel_topic', 'cmd_vel') # Twist topic roboteqs listen to

#         self.declare_parameter('precent_messages_written', 0.10) # Increments of 10 
#         self.declare_parameter('debugging', False)

#         self.cmd_vel_sub = self.create_subscription(
#             msg_type= Twist,
#             topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
#             callback= self.cmd_vel_recorder,
#             qos_profile= 10
#             )
        
#         self.roboteq_odom_sub = self.create_subscription(
#             msg_type= Odometry,
#             topic= self.get_parameter('roboteq_odom_topic').get_parameter_value().string_value,
#             callback= self.roboteq_odom_recorder,
#             qos_profile= 10
#             )
        
#         self.roboteq_info_pub = self.create_subscription(
#             msg_type= RoboteqInfo,
#             topic= self.get_parameter('roboteq_info_topic').get_parameter_value().string_value,
#             callback= self.roboteq_info_recorder,
#             qos_profile= 10
#             )
        
#         self.curr_cmd_vel_list: list[list[Twist]] = [] 
#         self.curr_roboteq_odom_list: list[list[Odometry]] = [] 
#         self.curr_roboteq_info_list: list[list[RoboteqInfo]] = []

#         self.time_string = time.strftime("%Y_%m_%d_%H_%M_%S")
#         self.ext: str = '.csv'

#         self.custom_file_str: str = ''

#         if (not self.get_parameter('autofill_file_names').get_parameter_value().bool_value):
#             self.custom_file_str = input("Enter custom file name: ") + "_"

#         self.make_info_header = True
#         self.make_odom_header = True
#         self.make_cmd_vel_header = True 

#         self.cmd_vel_list = [] 
#         self.odom_list = [] 
#         self.info_list = [] 

    
#     def update_list(self, new_val, list: list):
#         if len(list) >= self.get_parameter('data_list_depth').get_parameter_value().integer_value:
#             list.pop(0)
#         list.append(new_val)

  

#     def record_info(self, message: any, message_list: list[any], topic_name: str):

#         if self.get_parameter('debugging').get_parameter_value().bool_value:
#             self.get_logger().info("message list length: " + str(len(message_list)))

#         percent_msgs_written = self.get_parameter('precent_messages_written').get_parameter_value().double_value
#         assert percent_msgs_written < 1.0, "requested percentage of messages kept is more than 100%"

#         dir: str = self.get_parameter('data_target_directory').get_parameter_value().string_value
#         file_path = dir + self.time_string + '_' + self.custom_file_str + topic_name + self.ext

#         if len(message_list) >= self.get_parameter('data_write_depth').get_parameter_value().integer_value:

#             if self.get_parameter('debugging').get_parameter_value().bool_value:
#                 self.get_logger().info("Writing to file. message list: " + str(message_list))
#             written_list = [] 

#             if isinstance(message, Twist):
#                 for msg in message_list:
#                     to_be_written = twist_to_csv(msg)
#                     if self.make_cmd_vel_header:
#                         written_list.extend(to_be_written)
#                         self.make_cmd_vel_header = False 
#                     else:
#                         written_list.extend(to_be_written[1:])

#             elif isinstance(message, Odometry):
#                 for msg in message_list:
#                     to_be_written = odometry_to_csv(msg)
#                     if self.make_odom_header:
#                         written_list.extend(to_be_written)
#                         self.make_odom_header = False 
#                     else:
#                         written_list.extend(to_be_written[1:])

#             elif isinstance(message, RoboteqInfo):
#                 for msg in message_list:
#                     to_be_written = roboteq_info_to_csv(msg)
#                     if self.make_info_header:
#                         written_list.extend(to_be_written)
#                         self.make_info_header = False 
#                     else:
#                         written_list.extend(to_be_written[1:])
#             else:
#                 raise TypeError
            

#             print(f'{written_list}')
#             with open(file_path, "a+") as file:
#                 write = csv.writer(file)
#                 write.writerows(written_list)
#                 written_list = []
#                 message_list = []
#         else:
#             message_list.append(message)

#     def cmd_vel_recorder(self, cmd_vel_msg):
#         self.update_list(twist_to_csv(cmd_vel_msg)[1:],self.cmd_vel_list)
#         self.record_info(cmd_vel_msg, 
#                            self.curr_cmd_vel_list, 
#                            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value)

#     def roboteq_odom_recorder(self, roboteq_odom_msg):
#         self.update_list(odometry_to_csv(roboteq_odom_msg)[1:],self.odom_list)
#         self.record_info(roboteq_odom_msg, 
#                            self.curr_roboteq_odom_list, 
#                            self.get_parameter('roboteq_odom_topic').get_parameter_value().string_value)

#     def roboteq_info_recorder(self, roboteq_info_msg):
#         self.update_list(roboteq_info_to_csv(roboteq_info_msg)[1:],self.info_list)
#         self.record_info(roboteq_info_msg, 
#                            self.curr_roboteq_info_list, 
#                            self.get_parameter('roboteq_info_topic').get_parameter_value().string_value)


# class Slip_Gui_TK(tk.Tk):
#     def __init__(self, recorder: DataRecordingNode):
#         super().__init__()

#         self.recorder = recorder

#         self.cmd_vel_list = [1,2,3,4]
#         self.odom_list = [5,2,7,8]
#         self.info_list = [2,5,3,8]

#         self.title("Retail Bot")
#         self.geometry("1200x900")

#         self.create_widgets()

#         # Create a Timer to update the plot periodically
#         self.update_plot()

#         # Other setup and initialization code...

#     def create_widgets(self):
#         self.figure = plt.figure(figsize=(5, 4))
#         self.ax = self.figure.add_subplot(111)
#         self.canvas = FigureCanvasTkAgg(self.figure, master=self)
#         self.canvas.draw()
#         self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
#         checklist_frame = tk.Frame(self, width=300)
#         checklist_frame.grid(row=0, column=1, sticky="ns",)

#         checklist_frame.grid_propagate(0)

#         tk.Label(checklist_frame, text="Checklist:").pack(anchor=tk.W)

#         Checklist = ["Option 1","Option 2","Option 3"]

#         for item in range(20):
#             tk.Checkbutton(checklist_frame, text="item").pack(anchor=tk.W)

#         self.columnconfigure(0, weight=1)
#         self.rowconfigure(0, weight=1)

#     def update_plot(self):

#         cmd_vel_x = range(len(self.cmd_vel_list))
#         odom_x = range(len(self.odom_list))
#         info_x = range(len(self.info_list))

#         self.ax.clear()

#         self.ax.plot(cmd_vel_x, self.cmd_vel_list, label='Cmd Vel', color='r')
#         self.ax.plot(odom_x, self.odom_list, label='Odometry', color='g')
#         self.ax.plot(info_x, self.info_list, label='Info', color='b')
#         self.ax.set_autoscale_on(True)

#         self.ax.set_xlabel('Time (s)', fontsize='xx-large')
#         self.ax.set_ylabel('Values', fontsize='xx-large')
#         self.ax.legend(prop={'size': 'xx-large'})
#         self.ax.legend()

#         self.canvas.draw()

#         self.after(500, self.update_plot)




# def main(args=None):
#     rclpy.init(args=args)
#     recording = DataRecordingNode()

#     # slip_gui = Slip_Gui_TK()

#     # gui_thread = threading.Thread(target=slip_gui.mainloop)
#     # gui_thread.start()

#     while rclpy.ok():
#         rclpy.spin_once(recording)

#         if recording.get_parameter('debugging').get_parameter_value().bool_value:
#             recording.get_logger().info(f'\n{recording.cmd_vel_list}\n{recording.odom_list}\n{recording.info_list}')
#             recording.get_logger().info(f'\n{len(recording.cmd_vel_list)}\n{len(recording.odom_list)}\n{len(recording.info_list)}')

#        # slip_gui = Slip_Gui_TK()
#        # slip_gui.mainloop()

#     recording.destroy_node()
#     rclpy.shutdown()
    
# if __name__ == '__main__':
#     main()