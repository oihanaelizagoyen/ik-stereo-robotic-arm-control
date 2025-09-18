import sys
import socket
import json
import select
import numpy as np
from ikpy.chain import Chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import queue
import threading
import time

sys.path.append('../Classes')
from arduino_communication import *
from csv_logger import *

class RobotArmApp:
    def __init__(self):
        # Socket communication
        self.host = '127.0.0.1'
        self.port = 12345
        self.server = None
        self.client_conn = None
        self.client_addr = None
        self.receive_buffer = ""
        self.message_queue = queue.Queue()
        self.message_processing_thread = threading.Thread(target=self.process_messages)

        # Arduino communication
        self.arduino_communication = ArduinoCommunication()
        self.arduino_response_processing_thread = threading.Thread(target=self.process_arduino_responses)

        # Kinematics
        self.kinematics_chain = self.obtain_kinematics_chain()
        self.servo_angles = None
        self.orientation_enabled = False

        # Graph
        self.plot_enabled = True # ACTIVATE/DEACTIVATE ROBOT ARM SIMULATOR
        self.x_limit = [-1, 1]
        self.y_limit = [-1, 1]
        self.z_limit = [0, 1]
        self.legend_lines = [
            Line2D([0], [0], color='g', lw=2),
            Line2D([0], [0], color='#00DDFF', lw=2),
            Line2D([0], [0], color='#FFC000', lw=2)
        ]
        graph = self.initialize_graph()
        if graph != None:
            self.fig, self.ax = graph

        # Log
        self.time_control_log_enabled = True # ACTIVATE/DEACTIVATE TIME CONTROL (LOG)
        self.log_data = {}
        self.csv_logger = CSVLogger(["StateID", "Unity_EqueueTime", "Unity_SendTime","Python_ReceiveTime", "Python_SendTime", "Arduino_ReceiveTime"])

        # App state
        self.running = True

     # SOCKET
    def prepare_socket_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4 y SOCK_STREAM = TCP
        self.server.bind((self.host, self.port))
        self.server.listen(1)

    def handle_new_connection(self):
        self.client_conn, self.client_addr = self.server.accept()
        self.client_conn.setblocking(False)
        print(f"Client connected: {self.client_addr}")

    def handle_client_data(self):
        try:
            data = self.client_conn.recv(1024)

            if not data:  # Remote closing event
                print("Client disconnected.")
                self.client_conn.close()
                self.client_conn = None
                return None
            
            self.receive_buffer += data.decode()
        except Exception as e:
            print(f"Error in handle_client_data: {e}")

    def enqueue_messages(self):
        while '\n' in self.receive_buffer:
            line, self.receive_buffer = self.receive_buffer.split('\n', 1)
            if line.strip():
                self.message_queue.put(line.strip())

    def process_messages(self):
        while self.running:
            try:
                message = self.message_queue.get(timeout=0.1)
                response = self.extract_info_from_message(message)
                if response is None:
                        continue
                
                inverse_kinematics_active = response[0]
                if inverse_kinematics_active:
                    if self.time_control_log_enabled:
                        _, position, orientation, clamp, state_id, unity_equeue_time, unity_send_time, python_receive_time = response
                    else:
                        _, position, orientation, clamp = response
                else:
                    if self.time_control_log_enabled:
                        _, angles, clamp, state_id, unity_equeue_time, unity_send_time, python_receive_time = response
                    else:
                        _, angles, clamp = response

                if inverse_kinematics_active:
                    if position is None or orientation is None or clamp is None:
                        continue
                    self.calculate_inverse_kinematics(position, orientation, "all")
                    if self.servo_angles is None:
                        continue
                    servo_angles_arduino = (np.round(np.degrees(self.servo_angles[1:-2]))).astype(int).tolist()
                else:
                    if angles is None or clamp is None:
                        continue
                    self.servo_angles = [0] + np.radians(angles).tolist() + [0, 0]
                    servo_angles_arduino = angles.astype(int).tolist()

                servo_angles_arduino += [clamp]
                if self.time_control_log_enabled:
                    servo_angles_arduino = [state_id] + servo_angles_arduino

                self.update_graph(position if inverse_kinematics_active else None)

                if self.running:
                    self.arduino_communication.sendCommand(servo_angles_arduino)
                    if self.time_control_log_enabled:
                        python_send_time = int(time.time() * 1000)
                        data_dictionary = {
                            "StateID": state_id,
                            "Unity_EqueueTime": unity_equeue_time,
                            "Unity_SendTime": unity_send_time,
                            "Python_ReceiveTime": python_receive_time,
                            "Python_SendTime": python_send_time,
                            "Arduino_ReceiveTime": 0  # Placeholder, will be updated upon Arduino response
                        }
                        self.log_data[state_id] = data_dictionary
            except queue.Empty:
                continue

    def extract_info_from_message(self, message):
        try:
            print_message = ""
            info = ()

            message_json = json.loads(message)

            inverse_kinematics_active = bool(message_json['InverseKinematicsActive'])
            clamp = 10 if int(message_json['ClampState']) == 1 else 60

            if inverse_kinematics_active:
                position = np.array(message_json['Position'])
                orientation = self.quaternion_to_rotation_matrix(np.array(message_json['Orientation']))
                info = (inverse_kinematics_active, position, orientation, clamp)
                print_message = f"InverseKinematicsActive={message_json['InverseKinematicsActive']}, Position={message_json['Position']}, Orientation={message_json['Orientation']}, ClampState={message_json['ClampState']}"
            else:
                angles = np.array(message_json['Angles'])
                info = (inverse_kinematics_active, angles, clamp)
                print_message = f"InverseKinematicsActive={message_json['InverseKinematicsActive']}, Angles={message_json['Angles']}, ClampState={message_json['ClampState']}"

            if self.time_control_log_enabled:
                state_id = int(message_json['StateID'])
                unity_equeue_time = int(message_json['EqueueTime'])
                unity_send_time = int(message_json['SendTime'])
                python_receive_time = int(time.time() * 1000)
                info += (state_id, unity_equeue_time, unity_send_time, python_receive_time)
                print_message += f", StateID={message_json['StateID']}, EqueueTime={message_json['EqueueTime']}, SendTime={message_json['SendTime']}"

            print(f"Received information: {print_message}")
            return info

        except json.JSONDecodeError:
            print("Error decoding JSON")
            return None
        except Exception as e:
            print(f"Error in extract_info_from_message: {e}")
            return None

    # ARDUINO
    def process_arduino_responses(self):
        while self.running:
            try:
                response = self.arduino_communication.read()
                if response.size > 0:
                    print(f"Arduino response: {response}")
                    state_id = int(response[0])
                    self.log_data[state_id]["Arduino_ReceiveTime"] = int(time.time() * 1000)
                    self.csv_logger.log(self.log_data[state_id])
                    del self.log_data[state_id]
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"Error in process_arduino_responses: {e}")
                time.sleep(0.001)

    # KINEMATICS
    def obtain_kinematics_chain(self):
        return Chain.from_urdf_file(
            "RobotArm.urdf",     # URDF file path
            base_elements=["base"], # List of links that start the chain
            last_link_vector=np.array([0, 0, 0.102]), # Tip translation vector
            name="brazo_robot", # Chain name
            base_element_type="link", # Base element type
            active_links_mask=[False, True, True, True, True, True, False, False], # List of union statuses (active or inactive)
            symbolic=True
        )

    def calculate_inverse_kinematics(self, position, orientation, orientation_mode):
        # Validate orientation dimensions
        if orientation_mode == "all" and orientation.shape != (3, 3):
                print ("When the orientation mode is ‘all’, the orientation must be a matrix of dimension (3,3).")
                self.servo_angles = None
        if orientation_mode in ["X", "Y", "Z"] and orientation.shape != (3,):
                print ("When the orientation mode is ‘X’, ‘Y’, or ‘Z’, the orientation must be a vector of dimension (3,).")
                self.servo_angles = None
        
        # Reach the desired position.
        new_servo_angles = self.kinematics_chain.inverse_kinematics(
            target_position=position,
            initial_position=self.servo_angles
        )

        if (self.orientation_enabled):
            # From the desired position, achieve the desired orientation.
            self.servo_angles = self.kinematics_chain.inverse_kinematics(
                target_position=position,
                target_orientation=orientation,
                orientation_mode=orientation_mode,
                initial_position=new_servo_angles
            )
        else:
            self.servo_angles = new_servo_angles

    # GRAPH
    def on_key_press(self, event):
        if event.key == "q" or event.key == "Q":
            print("The 'q' or 'Q' button was pressed.")
            self.running = False

    def configure_axis(self, ax):
        ax.set_xlim(self.x_limit[0], self.x_limit[1])
        ax.set_ylim(self.y_limit[0], self.y_limit[1])
        ax.set_zlim(self.z_limit[0], self.z_limit[1])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend(self.legend_lines, ['X', 'Y', 'Z'])
        ax.text((self.x_limit[0]+self.x_limit[1])/2, self.y_limit[0], self.z_limit[0], "FRONT", color="red", fontsize=20)
        return ax

    def initialize_graph(self):
        if not self.plot_enabled:
            return None
        fig, ax = plot_utils.init_3d_figure()
        fig.set_figheight(9)
        fig.set_figwidth(13)

        initial_angles = [1.5708]*len(self.kinematics_chain)
        initial_angles[2] = 1.308996939
        self.kinematics_chain.plot(initial_angles, ax)
        ax = self.configure_axis(ax)

        plt.ion()
        fig.canvas.mpl_connect("key_press_event", self.on_key_press)
        fig.text(0.5, 0.95, "Press 'q' or 'Q' for exit", ha="center", va="center", fontsize=12, color="red")
        
        return fig, ax

    def update_graph(self, target_position=None):
        if not self.plot_enabled:
            return
        if not plt.fignum_exists(self.fig.number):
            self.running = False
            return
        
        self.fig.clf()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.kinematics_chain.plot(self.servo_angles, self.ax, target=target_position)
        self.ax = self.configure_axis(self.ax)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    # UTILITIES
    def quaternion_to_rotation_matrix(self, quaternion):
        x,y,z,w = quaternion
        return np.array([
            [1 - 2 * (y*y + z*z),   2 * (x*y - z*w),        2 * (x*z + y*w)],
            [2 * (x*y + z*w),       1 - 2 * (x*x + z*z),    2 * (y*z - x*w)],
            [2 * (x*z - y*w),       2 * (y*z + x*w),        1 - 2 * (x*x + y*y)]
        ])

    # APP
    def close(self):
        print("Closing app...")
        self.running = False
        
        if self.message_processing_thread.is_alive():
            self.message_processing_thread.join()

        if self.arduino_response_processing_thread.is_alive():
            self.arduino_response_processing_thread.join()

        if self.client_conn:
            self.client_conn.close()
        
        if self.server:
            self.server.close()

        plt.close('all')

        if self.arduino_communication:
            self.arduino_communication.closeCommunication()

    def run(self):
        print("Initializing app...")
        self.message_processing_thread.start()
        if self.time_control_log_enabled:
            self.arduino_response_processing_thread.start()
        self.prepare_socket_server()
        if self.plot_enabled: 
            plt.show(block=False)

        try:
            while self.running:
                if self.plot_enabled and not plt.fignum_exists(self.fig.number):
                    print("Graph window was closed.")
                    self.running = False
                    continue
                
                sockets_to_check = [self.client_conn] if self.client_conn else [self.server]
                readable, _, _ = select.select(sockets_to_check, [], [], 0)
                for socket in readable:
                    if not self.running:
                        break
                    if socket is self.server:
                        self.handle_new_connection()
                    elif socket is self.client_conn:
                        self.handle_client_data()
                        self.enqueue_messages()
                if self.plot_enabled:
                    plt.pause(0.01) # Pause for Matplotlib to update the window.
        except Exception as e:
            print(f"An error has occurred during the app cycle: {e}")
        finally:
            self.close()

if __name__ == "__main__":
    app = RobotArmApp()
    app.run()