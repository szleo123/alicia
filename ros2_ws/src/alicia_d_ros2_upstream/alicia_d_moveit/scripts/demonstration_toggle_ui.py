#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


class DemonstrationToggleUI(Node):
    def __init__(self):
        super().__init__("demonstration_toggle_ui")
        self.publisher = self.create_publisher(Bool, "/demonstration", 10)
        self.enabled = False

        self.root = tk.Tk()
        self.root.title("Alicia Hand-Guiding")
        self.root.geometry("260x120")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_label = tk.Label(self.root, text="", font=("TkDefaultFont", 11))
        self.status_label.pack(pady=(14, 10))

        self.toggle_button = tk.Button(
            self.root,
            text="",
            width=22,
            height=2,
            command=self.toggle_mode,
        )
        self.toggle_button.pack()

        self.hint_label = tk.Label(
            self.root,
            text="Toggle /demonstration zero-torque mode",
            font=("TkDefaultFont", 9),
        )
        self.hint_label.pack(pady=(10, 0))

        self.refresh_ui()

    def publish_mode(self):
        msg = Bool()
        msg.data = self.enabled
        self.publisher.publish(msg)
        mode = "Enabled" if self.enabled else "Disabled"
        self.get_logger().info(f"{mode} hand-guiding mode.")

    def refresh_ui(self):
        if self.enabled:
            self.status_label.config(text="Hand-guiding: ON", fg="#0b6e4f")
            self.toggle_button.config(text="Exit Demonstration Mode", bg="#d8f3dc", activebackground="#b7e4c7")
        else:
            self.status_label.config(text="Hand-guiding: OFF", fg="#7f1d1d")
            self.toggle_button.config(text="Enter Demonstration Mode", bg="#fee2e2", activebackground="#fecaca")

    def toggle_mode(self):
        self.enabled = not self.enabled
        self.publish_mode()
        self.refresh_ui()

    def on_close(self):
        if self.enabled:
            self.enabled = False
            self.publish_mode()
        self.root.quit()
        self.root.destroy()


def main():
    rclpy.init()
    node = DemonstrationToggleUI()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    finally:
        executor.cancel()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
