#!/usr/bin/env python3
"""
GUI application to control and monitor the Robotiq gripper.

This application provides:
- Real-time gripper status display
- Activation control
- Position control with slider
- Effort (force) control
- Quick action buttons (open, close, half)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time


class RobotiqGripperGUI:
    """GUI application for Robotiq gripper control."""

    def __init__(self, root):
        self.root = root
        self.root.title("Robotiq Gripper Controller")
        self.root.geometry("600x850")
        self.root.resizable(False, False)
        
        # ROS2 node and clients (initialized later in separate thread)
        self.node = None
        self.action_client = None
        self.activation_client = None
        self.joint_state_sub = None
        
        # State variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.target_position = tk.DoubleVar(value=0.0)
        self.target_effort = tk.DoubleVar(value=50.0)
        self.is_activated = tk.BooleanVar(value=False)
        self.is_moving = tk.BooleanVar(value=False)
        self.last_command_success = tk.StringVar(value="N/A")
        
        # Create GUI
        self.create_widgets()
        
        # Start ROS2 in separate thread
        self.ros_thread = threading.Thread(target=self.init_ros, daemon=True)
        self.ros_thread.start()
        
        # Start GUI update loop
        self.update_gui()

    def init_ros(self):
        """Initialize ROS2 node and clients in separate thread."""
        try:
            rclpy.init()
            self.node = Node('robotiq_gripper_gui')
            
            # Action client for gripper commands
            self.action_client = ActionClient(
                self.node,
                ParallelGripperCommand,
                '/robotiq_gripper_controller/gripper_cmd'
            )
            
            # Service client for activation
            self.activation_client = self.node.create_client(
                Trigger,
                '/robotiq_activation_controller/reactivate_gripper'
            )
            
            # Subscriber for joint states
            self.joint_state_sub = self.node.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )
            
            # Spin in background
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
        except Exception as e:
            print(f"ROS2 initialization error: {e}")

    def joint_state_callback(self, msg):
        """Update current gripper state from joint state messages."""
        # Look for the gripper joint
        for i, name in enumerate(msg.name):
            if 'knuckle_joint' in name or 'finger_joint' in name:
                if i < len(msg.position):
                    self.current_position = msg.position[i]
                if i < len(msg.velocity):
                    self.current_velocity = msg.velocity[i]
                self.is_moving.set(abs(self.current_velocity) > 0.001)
                break

    def create_widgets(self):
        """Create all GUI widgets."""
        
        # Title
        title_frame = ttk.Frame(self.root)
        title_frame.pack(pady=10, fill=tk.X)
        
        title_label = ttk.Label(
            title_frame,
            text="Robotiq Gripper Controller",
            font=("Helvetica", 18, "bold")
        )
        title_label.pack()
        
        # Status Frame
        status_frame = ttk.LabelFrame(self.root, text="Status", padding=10)
        status_frame.pack(pady=10, padx=20, fill=tk.BOTH)
        
        # Current Position
        ttk.Label(status_frame, text="Current Position:", font=("Helvetica", 10, "bold")).grid(
            row=0, column=0, sticky=tk.W, pady=5
        )
        self.position_label = ttk.Label(
            status_frame,
            text="0.000 m",
            font=("Helvetica", 10),
            foreground="blue"
        )
        self.position_label.grid(row=0, column=1, sticky=tk.W, pady=5, padx=10)
        
        # Position bar
        self.position_bar = ttk.Progressbar(
            status_frame,
            length=200,
            mode='determinate',
            maximum=85  # 0.085m = 85mm
        )
        self.position_bar.grid(row=0, column=2, pady=5, padx=10)
        
        # Current Velocity
        ttk.Label(status_frame, text="Velocity:", font=("Helvetica", 10, "bold")).grid(
            row=1, column=0, sticky=tk.W, pady=5
        )
        self.velocity_label = ttk.Label(
            status_frame,
            text="0.000 m/s",
            font=("Helvetica", 10)
        )
        self.velocity_label.grid(row=1, column=1, sticky=tk.W, pady=5, padx=10)
        
        # Moving indicator
        ttk.Label(status_frame, text="Status:", font=("Helvetica", 10, "bold")).grid(
            row=2, column=0, sticky=tk.W, pady=5
        )
        self.status_indicator = tk.Canvas(status_frame, width=20, height=20)
        self.status_indicator.grid(row=2, column=1, sticky=tk.W, pady=5, padx=10)
        self.status_circle = self.status_indicator.create_oval(2, 2, 18, 18, fill="gray")
        
        self.status_text = ttk.Label(status_frame, text="Unknown", font=("Helvetica", 10))
        self.status_text.grid(row=2, column=2, sticky=tk.W, pady=5)
        
        # Last command result
        ttk.Label(status_frame, text="Last Command:", font=("Helvetica", 10, "bold")).grid(
            row=3, column=0, sticky=tk.W, pady=5
        )
        self.command_result_label = ttk.Label(
            status_frame,
            textvariable=self.last_command_success,
            font=("Helvetica", 10)
        )
        self.command_result_label.grid(row=3, column=1, columnspan=2, sticky=tk.W, pady=5, padx=10)
        
        # Activation Frame
        activation_frame = ttk.LabelFrame(self.root, text="Activation", padding=10)
        activation_frame.pack(pady=10, padx=20, fill=tk.X)
        
        self.activate_btn = ttk.Button(
            activation_frame,
            text="Activate Gripper",
            command=self.activate_gripper,
            style="Accent.TButton"
        )
        self.activate_btn.pack(fill=tk.X, pady=5)
        
        ttk.Label(
            activation_frame,
            text="Note: Gripper must be activated before use",
            font=("Helvetica", 9, "italic"),
            foreground="gray"
        ).pack()
        
        # Control Frame
        control_frame = ttk.LabelFrame(self.root, text="Position Control", padding=10)
        control_frame.pack(pady=10, padx=20, fill=tk.BOTH)
        
        # Position slider
        ttk.Label(control_frame, text="Target Position (radians):", font=("Helvetica", 10)).pack(
            anchor=tk.W, pady=(0, 5)
        )
        
        position_control_frame = ttk.Frame(control_frame)
        position_control_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(position_control_frame, text="Open\n0.0 rad").pack(side=tk.LEFT, padx=5)
        
        self.position_slider = ttk.Scale(
            position_control_frame,
            from_=0.0,
            to=0.8,
            orient=tk.HORIZONTAL,
            variable=self.target_position,
            command=self.update_position_display
        )
        self.position_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        ttk.Label(position_control_frame, text="Closed\n0.8 rad").pack(side=tk.LEFT, padx=5)
        
        self.position_value_label = ttk.Label(
            control_frame,
            text="Target: 0.000 rad",
            font=("Helvetica", 11, "bold"),
            foreground="green"
        )
        self.position_value_label.pack(pady=5)
        
        # Effort slider
        ttk.Label(control_frame, text="Maximum Effort (Newtons):", font=("Helvetica", 10)).pack(
            anchor=tk.W, pady=(10, 5)
        )
        
        effort_control_frame = ttk.Frame(control_frame)
        effort_control_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(effort_control_frame, text="10").pack(side=tk.LEFT, padx=5)
        
        self.effort_slider = ttk.Scale(
            effort_control_frame,
            from_=10.0,
            to=235.0,
            orient=tk.HORIZONTAL,
            variable=self.target_effort,
            command=self.update_effort_display
        )
        self.effort_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        ttk.Label(effort_control_frame, text="235").pack(side=tk.LEFT, padx=5)
        
        self.effort_value_label = ttk.Label(
            control_frame,
            text="Effort: 50.0 N",
            font=("Helvetica", 11, "bold"),
            foreground="orange"
        )
        self.effort_value_label.pack(pady=5)
        
        # Send button
        self.send_btn = ttk.Button(
            control_frame,
            text="Send Command",
            command=self.send_command,
            style="Accent.TButton"
        )
        self.send_btn.pack(fill=tk.X, pady=10)
        
        # Quick Actions Frame
        quick_frame = ttk.LabelFrame(self.root, text="Quick Actions", padding=10)
        quick_frame.pack(pady=10, padx=20, fill=tk.X)
        
        buttons_frame = ttk.Frame(quick_frame)
        buttons_frame.pack(fill=tk.X)
        
        ttk.Button(
            buttons_frame,
            text="Fully Open",
            command=lambda: self.quick_action(0.0, 50.0)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        ttk.Button(
            buttons_frame,
            text="Half Open",
            command=lambda: self.quick_action(0.4, 50.0)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        ttk.Button(
            buttons_frame,
            text="Fully Close",
            command=lambda: self.quick_action(0.8, 50.0)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        # Gentle grip buttons
        gentle_frame = ttk.Frame(quick_frame)
        gentle_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(gentle_frame, text="Gentle Grip:", font=("Helvetica", 9)).pack(
            side=tk.LEFT, padx=5
        )
        
        ttk.Button(
            gentle_frame,
            text="Gentle Close",
            command=lambda: self.quick_action(0.8, 20.0)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        ttk.Button(
            gentle_frame,
            text="Firm Grip",
            command=lambda: self.quick_action(0.8, 100.0)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

    def update_position_display(self, value):
        """Update position value display."""
        self.position_value_label.config(text=f"Target: {float(value):.4f} rad")

    def update_effort_display(self, value):
        """Update effort value display."""
        self.effort_value_label.config(text=f"Effort: {float(value):.1f} N")

    def update_gui(self):
        """Update GUI elements periodically."""
        # Update position display
        self.position_label.config(text=f"{self.current_position:.4f} rad")
        self.position_bar['value'] = (self.current_position / 0.8) * 100  # Convert to percentage
        
        # Update velocity display
        self.velocity_label.config(text=f"{self.current_velocity:.4f} rad/s")
        
        # Update status indicator
        if self.is_moving.get():
            self.status_indicator.itemconfig(self.status_circle, fill="yellow")
            self.status_text.config(text="Moving")
        else:
            self.status_indicator.itemconfig(self.status_circle, fill="green")
            self.status_text.config(text="Idle")
        
        # Schedule next update
        self.root.after(100, self.update_gui)

    def activate_gripper(self):
        """Activate the gripper."""
        if self.activation_client is None:
            messagebox.showerror("Error", "ROS2 not initialized yet. Please wait.")
            return
        
        self.activate_btn.config(state=tk.DISABLED, text="Activating...")
        self.last_command_success.set("Activating...")
        
        def activate_thread():
            try:
                if not self.activation_client.wait_for_service(timeout_sec=5.0):
                    self.root.after(0, lambda: messagebox.showwarning(
                        "Service Unavailable", 
                        "Activation service not available.\n\n"
                        "This is normal when using fake/simulated hardware.\n"
                        "You can use the gripper controls without activation."
                    ))
                    self.root.after(0, lambda: self.last_command_success.set("N/A (Fake hardware)"))
                    self.is_activated.set(True)  # Allow usage anyway
                    return
                
                request = Trigger.Request()
                future = self.activation_client.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
                
                if future.result() and future.result().success:
                    self.is_activated.set(True)
                    self.root.after(0, lambda: messagebox.showinfo(
                        "Success", "Gripper activated successfully!"
                    ))
                    self.root.after(0, lambda: self.last_command_success.set("✓ Activated"))
                else:
                    msg = future.result().message if future.result() else "Unknown error"
                    self.root.after(0, lambda: messagebox.showerror(
                        "Error", f"Activation failed: {msg}"
                    ))
                    self.root.after(0, lambda: self.last_command_success.set(f"✗ Failed: {msg}"))
                    
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror(
                    "Error", f"Error during activation: {e}"
                ))
                self.root.after(0, lambda: self.last_command_success.set(f"✗ Error: {e}"))
            finally:
                self.root.after(0, lambda: self.activate_btn.config(
                    state=tk.NORMAL, text="Activate Gripper"
                ))
        
        threading.Thread(target=activate_thread, daemon=True).start()

    def send_command(self):
        """Send gripper command with current slider values."""
        position = self.target_position.get()
        effort = self.target_effort.get()
        self.execute_command(position, effort)

    def quick_action(self, position, effort):
        """Execute a quick action."""
        self.target_position.set(position)
        self.target_effort.set(effort)
        self.execute_command(position, effort)

    def execute_command(self, position, effort):
        """Execute gripper command."""
        if self.action_client is None:
            messagebox.showerror("Error", "ROS2 not initialized yet. Please wait.")
            return
        
        self.send_btn.config(state=tk.DISABLED, text="Sending...")
        self.last_command_success.set(f"Sending command...")
        
        def send_thread():
            try:
                if not self.action_client.wait_for_server(timeout_sec=5.0):
                    self.root.after(0, lambda: messagebox.showerror(
                        "Error", "Action server not available"
                    ))
                    self.root.after(0, lambda: self.last_command_success.set("✗ Server unavailable"))
                    return
                
                goal = ParallelGripperCommand.Goal()
                goal.command.position = [position]
                goal.command.effort = [effort]
                
                future = self.action_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.root.after(0, lambda: messagebox.showerror(
                        "Error", "Goal rejected by server"
                    ))
                    self.root.after(0, lambda: self.last_command_success.set("✗ Goal rejected"))
                    return
                
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=15.0)
                
                result = result_future.result()
                if result and result.result.reached_goal:
                    pos = result.result.state.position[0] if result.result.state.position else 0.0
                    self.root.after(0, lambda: self.last_command_success.set(
                        f"✓ Success - Pos: {pos:.4f}m"
                    ))
                elif result and result.result.stalled:
                    pos = result.result.state.position[0] if result.result.state.position else 0.0
                    self.root.after(0, lambda: self.last_command_success.set(
                        f"⚠ Stalled - Pos: {pos:.4f}m"
                    ))
                else:
                    self.root.after(0, lambda: self.last_command_success.set("✗ Failed"))
                    
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror(
                    "Error", f"Error sending command: {e}"
                ))
                self.root.after(0, lambda: self.last_command_success.set(f"✗ Error: {e}"))
            finally:
                self.root.after(0, lambda: self.send_btn.config(
                    state=tk.NORMAL, text="Send Command"
                ))
        
        threading.Thread(target=send_thread, daemon=True).start()

    def on_closing(self):
        """Handle window closing."""
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.root.destroy()


def main():
    """Main function."""
    root = tk.Tk()
    
    # Configure style
    style = ttk.Style()
    style.theme_use('clam')
    
    app = RobotiqGripperGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()


if __name__ == '__main__':
    main()
