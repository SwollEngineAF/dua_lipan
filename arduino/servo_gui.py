import serial
import tkinter as tk

# Adjust this to match your Arduino port
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def set_servo(channel, value):
    try:
        arduino.write(bytes([channel]))
        arduino.write(f"{value}\n".encode())
    except:
        print(f"Failed to send to servo {channel}")

def create_slider(frame, label, channel, default=300):





























































































































    label_frame = tk.LabelFrame(frame, text=label)
    label_frame.pack(padx=5, pady=5, fill='x')

    slider = tk.Scale(label_frame, from_=150, to=550, orient='horizontal', length=300)
    slider.set(default)
    slider.pack(side='left', padx=5)

    def on_click():
        val = slider.get()
        set_servo(channel, val)

    btn = tk.Button(label_frame, text="Set", command=on_click)
    btn.pack(side='right', padx=5)

root = tk.Tk()
root.title("Robot Arm Servo Control")

frame = tk.Frame(root)
frame.pack(padx=10, pady=10)

create_slider(frame, "Base (Channel 0)", 0)
create_slider(frame, "Right Arm (Channel 4)", 4)
create_slider(frame, "Left Arm (Channel 11)", 11)
create_slider(frame, "Arm (Channel 15)", 15)

root.mainloop()
