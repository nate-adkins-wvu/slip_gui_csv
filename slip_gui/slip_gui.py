import matplotlib, tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from data_recording import DataRecordingNode



class Slip_Gui_TK(tkinter.Tk):
    def __init__(self, recording_node: DataRecordingNode):
        super().__init__()

        self.recording_node = recording_node
        self.cmd_vel_list = [1,2,3,4]
        self.odom_list = [5,2,7,8]
        self.info_list = [2,5,3,8]

        self.title("Retail Bot")
        self.geometry("1200x900")

        self.create_widgets()

        self.update_plot()


    def create_widgets(self):
        self.figure = matplotlib.pyplot.figure(figsize=(5, 4))
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        checklist_frame = tkinter.Frame(self, width=300)
        checklist_frame.grid(row=0, column=1, sticky="ns",)

        checklist_frame.grid_propagate(0)

        tkinter.Label(checklist_frame, text="Checklist:").pack(anchor=tkinter.W)

        Checklist = ["Option 1","Option 2","Option 3"]

        for item in range(20):
            tkinter.Checkbutton(checklist_frame, text="item").pack(anchor=tkinter.W)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)


    def fetch_data(self):
        while True:
            self.recording_node.cmd_vel_event.wait()
            self.recording_node.roboteq_odom_event.wait()
            self.recording_node.roboteq_info_event.wait()

            with self.recording_node.cmd_vel_list_lock:
                self.cmd_vel_list = list(self.recording_node.cmd_vel_queue.queue)

            with self.recording_node.odom_list_lock:
                self.odom_list = list(self.recording_node.roboteq_odom_queue.queue)

            with self.recording_node.info_list_lock:
                self.info_list = list(self.recording_node.roboteq_info_queue.queue)

            with self.recording_node.cmd_vel_list_lock:
                self.recording_node.cmd_vel_queue.queue.clear()
                self.recording_node.cmd_vel_event.clear()

            with self.recording_node.odom_list_lock:
                self.recording_node.roboteq_odom_queue.queue.clear()
                self.recording_node.roboteq_odom_event.clear()

            with self.recording_node.info_list_lock:
                self.recording_node.roboteq_info_queue.queue.clear()
                self.recording_node.roboteq_info_event.clear()
            self.update_plot()


    def update_plot(self):

        cmd_vel_x = range(len(self.cmd_vel_list))
        odom_x = range(len(self.odom_list))
        info_x = range(len(self.info_list))

        self.ax.clear()

        self.ax.plot(cmd_vel_x, self.cmd_vel_list, label='Cmd Vel', color='r')
        self.ax.plot(odom_x, self.odom_list, label='Odometry', color='g')
        self.ax.plot(info_x, self.info_list, label='Info', color='b')
        self.ax.set_autoscale_on(True)

        self.ax.set_xlabel('Time (s)', fontsize='xx-large')
        self.ax.set_ylabel('Values', fontsize='xx-large')
        self.ax.legend(prop={'size': 'xx-large'})
        self.ax.legend()

        self.canvas.draw()

        self.after(500, self.update_plot)