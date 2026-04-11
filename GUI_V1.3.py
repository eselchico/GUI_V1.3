"""
Robot Arm Control Panel
=======================
Integrates:
  - Tkinter dark-mode GUI  (Manual, Controller, Gesture/Vision tabs)
  - Dynamixel AX-series motor control via dynamixel_sdk
  - Stepper control via Arduino serial (send_int / receive_bits)
  - Pygame joystick controller with trigger deadman
  - cvzone HandDetector gesture control via webcam thread
"""

import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import threading
import time

import pygame
import serial
import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector
from PIL import Image, ImageTk
from dynamixel_sdk import (
    PortHandler, PacketHandler,
    COMM_SUCCESS,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
)


# ─────────────────────────────────────────────────────────────────
#  Colour palette
# ─────────────────────────────────────────────────────────────────
BG      = "#2c3e50"
BG2     = "#1a252f"
BG3     = "#243342"
FG      = "#ecf0f1"
FG2     = "#95a5a6"
ACCENT  = "#3498db"
ACCENT2 = "#2ecc71"
DANGER  = "#e74c3c"
WARNING = "#f39c12"
BORDER  = "#34495e"

# ─────────────────────────────────────────────────────────────────
#  Hardware / comms constants  — edit these to match your setup
# ─────────────────────────────────────────────────────────────────
SERIAL_PORT   = "COM3"
SERIAL_BAUD   = 115200
DXL_PORT      = "COM4"
DXL_BAUDRATE  = 1000000
DXL_PROTOCOL  = 1.0
DXL_IDS       = [1, 2, 3, 4]
MOVING_SPEED  = 300
MAX_TORQUE    = 300
JOG_SPEED     = 25
DEADZONE      = 0.1
CAM_JOG_SPEED = 0.25
CAM_MAX_DX    = 75
HOMING_TIME   = 3.0          # seconds of no gesture before auto-home

# Dynamixel register addresses
MAXTORQUE_ADDR = 14
TorqueEnable   = 24
GoalPosition   = 30
MovingSpeed    = 32
PresentPosition= 36


# ═════════════════════════════════════════════════════════════════
class RobotControlGUI:
# ═════════════════════════════════════════════════════════════════

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Robot Arm Control Panel")
        self.root.configure(bg=BG)
        self.root.geometry("960x720")
        self.root.resizable(True, True)

        # ── Robot state ───────────────────────────────────────────
        self.GoalPos     = [100] + [512] * 4
        self.currentPos  = [0]   * 5
        self.data        = 0
        self.Rx          = 0
        self.start       = False
        self.moveStepper = False
        self.lastButton  = 0
        self.last_send   = time.time()
        self.send_interval = 0.15 #sets data = 0 after this delay

        self.LETTER_POSES = {
            "M": [
                [0, 200, 300, 400, 512],  # waypoint 1
                [0, 300, 400, 300, 512],  # waypoint 2
                [20, 400, 300, 400, 512],  # waypoint 3
                [40, 512, 512, 512, 512],
                [40, 512, 512, 512, 512],
            ],
            "R": [
                [0, 200, 300, 400, 512],  # waypoint 1
                [0, 300, 400, 300, 512],  # waypoint 2
                [20, 400, 300, 400, 512],  # waypoint 3
                [40, 512, 512, 512, 512],
                [40, 512, 512, 512, 512],
            ],
            "I": [
                [0, 200, 300, 400, 512],  # waypoint 1
                [0, 300, 400, 300, 512],  # waypoint 2
                [20, 400, 300, 400, 512],  # waypoint 3
                [40, 512, 512, 512, 512],
                [40, 512, 512, 512, 512],
            ],
        }

        self.MRI_SEQUENCE = self.LETTER_POSES["M"] + self.LETTER_POSES["R"] + self.LETTER_POSES["I"]

        # ── Gesture state ─────────────────────────────────────────
        self.thumbLeft   = [0] * 4
        self.indexLeft   = [0] * 4
        self.middleLeft  = [0] * 4
        self.thumbRight  = [0] * 4
        self.indexRight  = [0] * 4
        self.middleRight = [0] * 4
        self.fingersLeft  = [0] * 5
        self.fingersRight = [0] * 5
        self.leftDX  = [0] * 2
        self.rightDX = [0] * 2
        self.upLeft  = 0
        self.upRight = 0
        self.homing  = True
        self.homed = False #used to know when can use the slider joint
        self.previousHomingTime = time.time()
        self.previousTime       = time.time()

        # Offsets
        self.thumbOffset           = 40
        self.indexVerticalOffset   = 40
        self.indexHorizontalOffset = 25
        self.middleVerticalOffset  = 50
        self.middleHorizontalOffset= 25

        # ── GUI state ─────────────────────────────────────────────
        self.joint_vars        = [tk.DoubleVar(value=0.0) for _ in range(5)]
        self.joint_vars[0].set(80)#puts in the middle?
        self.joint_entries     = []
        self.controller_active = tk.BooleanVar(value=False)
        self.estop_active      = tk.BooleanVar(value=False)
        self.camera_running    = False
        self._camera_stop      = threading.Event()

        # ── Hardware handles ──────────────────────────────────────
        self.ser           = None
        self.portHandler   = None
        self.packetHandler = None
        self.joysticks     = {}
        self._cap          = None
        self.detector      = None
        self._joy          = None

        # Build GUI first so _log is available
        self._apply_styles()
        self._build_header()
        self._build_notebook()
        self._build_status_bar()

        # Initialise hardware after window exists
        self.root.after(100, self._init_hardware)

        # Recurring loops
        self.root.after(20, self._serial_loop)
        self.root.after(20, self._controller_poll)

    # ═════════════════════════════════════════════════════════════
    #  Hardware initialisation
    # ═════════════════════════════════════════════════════════════

    def _init_hardware(self):
        """Open serial, Dynamixel port, and pygame. Called once after GUI ready."""

        # Serial (Arduino stepper control)
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            time.sleep(1)
            self.ser.reset_input_buffer()
            self._log(f"Serial open: {SERIAL_PORT} @ {SERIAL_BAUD}")
        except Exception as e:
            self._log(f"⚠ Serial failed: {e}")

        # Dynamixel
        try:
            self.portHandler   = PortHandler(DXL_PORT)
            self.packetHandler = PacketHandler(DXL_PROTOCOL)

            if not self.portHandler.openPort():
                self._log("⚠ Dynamixel port failed to open")
                return
            self._log(f"Dynamixel port open: {DXL_PORT}")

            if not self.portHandler.setBaudRate(DXL_BAUDRATE):
                self._log("⚠ Dynamixel baud set failed")
                return
            self._log(f"Dynamixel baud set: {DXL_BAUDRATE}")

            for i, mid in enumerate(DXL_IDS):
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, mid, TorqueEnable, 1)
                pos, _, _ = self.packetHandler.read2ByteTxRx(
                    self.portHandler, mid, PresentPosition)
                self.currentPos[i+1] = pos
                self.GoalPos[i+1]    = pos
                print(pos)
                self.joint_vars[i+1].set(round(self.currentPos[i+1]*0.293 - 150,1))
                self.packetHandler.write2ByteTxRx(
                    self.portHandler, mid, MovingSpeed, MOVING_SPEED)
                self.packetHandler.write2ByteTxRx(
                    self.portHandler, mid, MAXTORQUE_ADDR, MAX_TORQUE)
            self._log("Dynamixel motors initialised.")
            self._update_pose_display()
            self._update_ctrl_joint_positions(self.currentPos)
            self._update_goal_positions(self.GoalPos)

        except Exception as e:
            self._log(f"⚠ Dynamixel init error: {e}")

        # Pygame
        try:
            pygame.init()
            pygame.joystick.init()
            self._log("Pygame initialised.")
        except Exception as e:
            self._log(f"⚠ Pygame init error: {e}")

        # cvzone hand detector
        self.detector = HandDetector(maxHands=2)

        # Initial serial handshake
        if self.ser:
            self.Rx = self._receive_bits() or 0
            self._send_2int(0,0)


    # ═════════════════════════════════════════════════════════════
    #  Serial helpers
    # ═════════════════════════════════════════════════════════════

    def _send_int(self, value: int):
        """Pack lower 3 bits into a byte and send to Arduino."""
        if not self.ser:
            return
        try:
            payload = (value & 0x07).to_bytes(1, byteorder='big')
            self.ser.write(payload)
        except Exception as e:
            self._log(f"Serial TX error: {e}")


    def _send_2int(self, hibyte: int, lowbyte: int):
        """Pack lower 3 bits into a byte and send to Arduino.
            hibyte = what to do
            lowbyte = how to do
            change the & for the hibyte
        """
        if not self.ser:
            return
        try:
            high = hibyte & 0x0F
            low = min(max(0, lowbyte), 160) if high == 8 else 0
            value = bytes([0xFF, high, low])

            #payload = (0xFF << 16 | (hibyte & 0x0F) << 8 | lowbyte).to_bytes(3, byteorder='big')
            print(f'Tx: data: {high}, low: {low}, value: {value}')
            self.ser.write(value)
        except Exception as e:
            self._log(f"Serial TX error: {e}")

    def _receive_bits(self) -> int:
        """Read one raw byte from Arduino. Returns 0 if nothing available."""
        if not self.ser:
            return 0
        try:
            if self.ser.in_waiting >= 1:
                raw  = self.ser.read(1)
                info = raw[0]
                if info in (10, 13):
                    return 0
                return int(info)
        except TypeError:
            return 7   # safety: treat as e-stop
        except Exception:
            return 0
        return 0

    def _receive_2bytes(self) -> int:
        """Read one raw byte from Arduino. Returns 0 if nothing available."""
        if not self.ser:
            return 0
        try:
            if self.ser.in_waiting >= 3:
                if self.ser.read(1)[0] == 0xFF:
                    raw  = self.ser.read(2)
                    hibyte = raw[0]
                    lowbyte = raw[1]
                    print(f'Rx: {list(raw)}')

                    # if hibyte in (10, 13) or lowbyte in (10, 13):
                    #     return 0
                    return int(hibyte << 8 | lowbyte)
        except TypeError:
            hibyte = 7
            lowbyte = 0
            return int(hibyte << 8 | lowbyte)   # safety: treat as e-stop
        except Exception:
            return 0
        return 0

    # ═════════════════════════════════════════════════════════════
    #  Serial send / receive loop  (every send_interval via root.after)
    # ═════════════════════════════════════════════════════════════

    def _serial_loop(self):
        now = time.time()
        #if now - self.last_send > self.send_interval:
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        self._send_2int(self.data, round(self.GoalPos[0]))#lowbyte should be a variable
        time.sleep(0.01)
        self.Rx = self._receive_2bytes() or 0
        self.currentPos[0] = self.Rx & 0xFF
        self.last_send = now

        # E-stop from Arduino (bit 0 = 1 means estop active)
        if (self.Rx >> 8) & 1:
            if not self.estop_active.get():
                self._emergency_stop(from_arduino=True)

        self.homed = True if (self.Rx >> 11) & 1 else False

        if self.camera_running:
            if now - self.last_send > self.send_interval:
                self.data = 0   # reset after each send
        else:
            self.data = 0

        #updates the arduino pos for the controller and camera
        if self.homed and (self.controller_active.get() or self.camera_running):
            self.GoalPos[0] = self.currentPos[0]

        self.root.after(20, self._serial_loop)

    # ═════════════════════════════════════════════════════════════
    #  Dynamixel helpers
    # ═════════════════════════════════════════════════════════════

    def _write_goal_positions(self):
        """Write GoalPos to all 4 Dynamixel motors."""
        if not self.portHandler:
            return
        for i, mid in enumerate(DXL_IDS):
            self.packetHandler.write2ByteTxRx(
                self.portHandler, mid, GoalPosition, self.GoalPos[i+1])

    def _read_current_positions(self):
        """Read present positions from all 4 Dynamixel motors."""
        if not self.portHandler:
            return
        for i, mid in enumerate(DXL_IDS):
            pos, _, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, mid, PresentPosition)
            self.currentPos[i+1] = pos

    def _disable_all_torque(self):
        if not self.portHandler:
            return
        for mid in DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, mid, TorqueEnable, 0)

    def _enable_all_torque(self):
        if not self.portHandler:
            return
        for mid in DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, mid, TorqueEnable, 1)


    def _play_pose_sequence(self, sequence: list, step_delay_ms: int = 1500):
        """
        Step through a list of poses, writing each to the motors.
        Uses root.after() so the GUI stays responsive.

        Parameters
        ----------
        sequence      : list of pose lists e.g. [[J1,J2,J3,J4], ...]
        step_delay_ms : milliseconds to wait between poses
        """
        if self.estop_active.get():
            self._log("⚠ Cannot run sequence — E-stop is active.")
            return
        if not self.portHandler:
            self._log("⚠ Dynamixel not connected.")
            return

        self._sequence = list(sequence)  # copy so original is untouched
        self._sequence_index = 0
        self._sequence_delay = step_delay_ms
        self._log(f"Starting sequence: {len(sequence)} poses, {step_delay_ms}ms per step.")
        self._sequence_step()


    def _sequence_step(self):
        """Execute one pose in the sequence, then schedule the next."""
        if self.estop_active.get():
            self._log("⚠ Sequence aborted — E-stop triggered.")
            return

        if self._sequence_index >= len(self._sequence):
            self._log("Sequence complete.")
            return

        pose = self._sequence[self._sequence_index]
        for i, count in enumerate(pose[1:5]):
            self.GoalPos[i] = max(0, min(1023, count))

        self.data = 8
        self._write_goal_positions()
        self._update_goal_positions(self.GoalPos)
        self._log(f"Pose {self._sequence_index + 1}/{len(self._sequence)}: {pose}")

        self._sequence_index += 1
        self.root.after(self._sequence_delay, self._sequence_step)

    # ═════════════════════════════════════════════════════════════
    #  Styling
    # ═════════════════════════════════════════════════════════════

    def _apply_styles(self):
        s = ttk.Style(self.root)
        s.theme_use("clam")
        s.configure(".", background=BG, foreground=FG,
                    fieldbackground=BG2, bordercolor=BORDER,
                    troughcolor=BG2, font=("Courier New", 10))
        s.configure("TNotebook", background=BG2, borderwidth=0)
        s.configure("TNotebook.Tab", background=BG3, foreground=FG2,
                    padding=[16, 6], font=("Courier New", 10, "bold"))
        s.map("TNotebook.Tab",
              background=[("selected", BG)], foreground=[("selected", ACCENT)])
        s.configure("TFrame", background=BG)
        s.configure("TLabelframe", background=BG, foreground=FG2,
                    bordercolor=BORDER, relief="flat")
        s.configure("TLabelframe.Label", background=BG, foreground=FG2,
                    font=("Courier New", 9))
        s.configure("TLabel", background=BG, foreground=FG,
                    font=("Courier New", 10))
        s.configure("Header.TLabel", background=BG, foreground=FG,
                    font=("Courier New", 13, "bold"))
        s.configure("TButton", background=BG3, foreground=FG, borderwidth=1,
                    relief="flat", padding=[10, 6], font=("Courier New", 10, "bold"))
        s.map("TButton",
              background=[("active", BORDER), ("pressed", BG2)])
        s.configure("Accent.TButton", background=ACCENT, foreground=FG)
        s.map("Accent.TButton",
              background=[("active", "#2980b9"), ("pressed", "#1a6fa8")])
        s.configure("Danger.TButton", background=DANGER, foreground=FG,
                    font=("Courier New", 11, "bold"))
        s.map("Danger.TButton",
              background=[("active", "#c0392b"), ("pressed", "#96281b")])
        s.configure("Success.TButton", background=ACCENT2, foreground=BG)
        s.map("Success.TButton", background=[("active", "#27ae60")])
        s.configure("TSeparator", background=BORDER)
        s.configure("TScrollbar", background=BG3, troughcolor=BG2,
                    bordercolor=BORDER, arrowcolor=FG2)

    # ═════════════════════════════════════════════════════════════
    #  Header
    # ═════════════════════════════════════════════════════════════

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=BG2, height=52)
        hdr.pack(fill="x", side="top")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="◈  ROBOT ARM CONTROL PANEL",
                 bg=BG2, fg=FG,
                 font=("Courier New", 14, "bold")).pack(side="left", padx=20, pady=12)
        tk.Button(hdr, text="⚠  EMERGENCY STOP",
                  bg=DANGER, fg=FG,
                  font=("Courier New", 11, "bold"),
                  relief="flat", bd=0, padx=14,
                  command=self._emergency_stop).pack(side="right", padx=16, pady=10)
        tk.Button(hdr, text="↺  Reset E-Stop",
                  bg=BG3, fg=FG2,
                  font=("Courier New", 9),
                  relief="flat", bd=0, padx=10,
                  command=self._reset_estop).pack(side="right", padx=4, pady=10)
        tk.Label(hdr, text="v2.0", bg=BG2, fg=FG2,
                 font=("Courier New", 9)).pack(side="right", padx=4)

    # ═════════════════════════════════════════════════════════════
    #  Notebook
    # ═════════════════════════════════════════════════════════════

    def _build_notebook(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=(6, 4))
        self.tab_manual     = ttk.Frame(self.notebook)
        self.tab_controller = ttk.Frame(self.notebook)
        self.tab_vision     = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_manual,     text="  Manual Control  ")
        self.notebook.add(self.tab_controller, text="  Controller  ")
        self.notebook.add(self.tab_vision,     text="  Gesture & Vision  ")
        self._build_manual_tab()
        self._build_controller_tab()
        self._build_vision_tab()

    # ═════════════════════════════════════════════════════════════
    #  TAB 1 — Manual Joint Control
    # ═════════════════════════════════════════════════════════════

    def _build_manual_tab(self):
        p = self.tab_manual
        p.configure(style="TFrame")
        ttk.Label(p, text="Joint Angle Control",
                  style="Header.TLabel").grid(row=0, column=0, columnspan=4,
                                              sticky="w", padx=16, pady=(14, 4))
        ttk.Separator(p, orient="horizontal").grid(
            row=1, column=0, columnspan=4, sticky="ew", padx=12, pady=(0, 10))
        p.columnconfigure(1, weight=1)

        joint_names  = ["J1  Stepper", "J2  Shoulder",
                        "J3  Elbow", "J4  Wrist Pitch", "J5  Wrist Roll"]
        joint_ranges = [(0, 160)] + [(-150, 150)] * 4
        self.joint_entries.clear()

        for i, name in enumerate(joint_names):
            row = i + 2
            lo, hi = joint_ranges[i]
            tk.Label(p, text=name, bg=BG, fg=FG2,
                     font=("Courier New", 10),
                     width=18, anchor="w").grid(row=row, column=0,
                                                padx=(16, 4), pady=4, sticky="w")
            sl = tk.Scale(p, variable=self.joint_vars[i],
                          from_=lo, to=hi, orient="horizontal", resolution=0.5,
                          bg=BG, fg=FG, troughcolor=BG2, activebackground=ACCENT,
                          highlightthickness=0, bd=0, showvalue=False,
                          command=lambda val, idx=i: self._on_slider_change(idx, val))
            sl.grid(row=row, column=1, sticky="ew", padx=6, pady=4)
            ent = tk.Entry(p, textvariable=self.joint_vars[i],
                           width=8, bg=BG3, fg=ACCENT, insertbackground=FG,
                           relief="flat", bd=4, font=("Courier New", 10))
            ent.grid(row=row, column=2, padx=4, pady=4)
            ent.bind("<Return>", lambda e, idx=i: self._on_entry_change(idx))
            self.joint_entries.append(ent)
            tk.Label(p, text="°", bg=BG, fg=FG2,
                     font=("Courier New", 10)).grid(row=row, column=3,
                                                    padx=(0, 16), sticky="w")

        btn_frame = tk.Frame(p, bg=BG)
        btn_frame.grid(row=8, column=0, columnspan=4,
                       pady=(14, 10), padx=16, sticky="ew")
        ttk.Button(btn_frame, text="▶  Apply Movement",
                   style="Accent.TButton",
                   command=self._execute_joint_move).pack(side="left", padx=(0, 8))
        ttk.Button(btn_frame, text="⊕  Homing",
                   style = "Accent.TButton" if self.homed else "TButton",
                   command=self._home_position).pack(side="left", padx=(0, 8))
        ttk.Button(btn_frame, text="↺  Reset All",
                   command=self._reset_joints).pack(side="left", padx=(0, 8))
        ttk.Button(btn_frame, text="⚠  E-STOP",
                   style="Danger.TButton",
                   command=self._emergency_stop).pack(side="right")
        #add buttons for writing MRI or smtg idk

        pose_frame = ttk.LabelFrame(p, text=" Current Pose ", padding=10)
        pose_frame.grid(row=9, column=0, columnspan=4,
                        sticky="ew", padx=16, pady=(4, 14))
        pose_frame.columnconfigure(list(range(5)), weight=1)
        self.pose_labels = []
        for i in range(5):
            tk.Label(pose_frame, text=f"J{i+1}",
                     bg=BG, fg=FG2,
                     font=("Courier New", 9)).grid(row=0, column=i, padx=4)
            lbl = tk.Label(pose_frame, text="0.0°",
                           bg=BG3, fg=ACCENT,
                           font=("Courier New", 10, "bold"),
                           width=7, relief="flat", padx=4, pady=2)
            lbl.grid(row=1, column=i, padx=4, pady=2)
            self.pose_labels.append(lbl)
        self._update_pose_display()

    # ═════════════════════════════════════════════════════════════
    #  TAB 2 — External Controller
    # ═════════════════════════════════════════════════════════════

    def _build_controller_tab(self):
        p = self.tab_controller
        ttk.Label(p, text="External Controller",
                  style="Header.TLabel").pack(anchor="w", padx=16, pady=(14, 4))
        ttk.Separator(p, orient="horizontal").pack(fill="x", padx=12, pady=(0, 10))

        content = tk.Frame(p, bg=BG)
        content.pack(fill="both", expand=True, padx=16)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)

        # Left panel
        left = tk.Frame(content, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        ind_frame = tk.Frame(left, bg=BG2)
        ind_frame.pack(fill="x", pady=(0, 10))
        tk.Label(ind_frame, text="Controller Status",
                 bg=BG2, fg=FG2,
                 font=("Courier New", 10, "bold")).pack(pady=(10, 6))
        self.ctrl_canvas = tk.Canvas(ind_frame, width=120, height=120,
                                     bg=BG2, highlightthickness=0)
        self.ctrl_canvas.pack(pady=(0, 10))
        self._draw_controller_indicator(active=False)
        self.ctrl_status_lbl = tk.Label(ind_frame, text="DISCONNECTED",
                                         bg=BG2, fg=DANGER,
                                         font=("Courier New", 11, "bold"))
        self.ctrl_status_lbl.pack(pady=(0, 6))

        tk.Label(ind_frame, text="Deadman: hold LT or RT",
                 bg=BG2, fg=FG2, font=("Courier New", 8)).pack()
        tk.Label(ind_frame, text="Start/Stop: button 5 (RB)",
                 bg=BG2, fg=FG2, font=("Courier New", 8)).pack()
        tk.Label(ind_frame, text="Home: button 15",
                 bg=BG2, fg=FG2, font=("Courier New", 8)).pack(pady=(0, 6))

        btn_row = tk.Frame(ind_frame, bg=BG2)
        btn_row.pack(pady=(0, 6))
        ttk.Button(btn_row, text="Connect",
                   style="Success.TButton",
                   command=self._connect_controller).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Disconnect",
                   command=self._disconnect_controller).pack(side="left", padx=4)

        self.start_lbl = tk.Label(ind_frame, text="● START: OFF",
                                   bg=BG2, fg=DANGER,
                                   font=("Courier New", 9, "bold"))
        self.start_lbl.pack(pady=(0, 8))

        # Joint position display
        joint_pos_frame = ttk.LabelFrame(left, text=" Current Joint Positions ", padding=10)
        joint_pos_frame.pack(fill="x", pady=(10, 0))
        joint_pos_frame.columnconfigure(list(range(5)), weight=1)
        tk.Label(joint_pos_frame, text="Joint",
                 bg=BG, fg=FG2,
                 font=("Courier New", 9, "bold")).grid(row=0, column=0,
                                                       columnspan=5, sticky="w", pady=(0, 4))
        self.ctrl_joint_labels = []
        for i in range(5):
            tk.Label(joint_pos_frame, text=f"J{i+1}",
                     bg=BG, fg=FG2,
                     font=("Courier New", 9)).grid(row=1, column=i, padx=4)
            lbl = tk.Label(joint_pos_frame, text="—",
                           bg=BG3, fg=ACCENT,
                           font=("Courier New", 9, "bold"),
                           width=6, relief="flat", padx=2, pady=2)
            lbl.grid(row=2, column=i, padx=4, pady=2)
            self.ctrl_joint_labels.append(lbl)

        # Right panel — telemetry
        right = tk.Frame(content, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        telem_frame = ttk.LabelFrame(right, text=" Telemetry ", padding=10)
        telem_frame.pack(fill="both", expand=True)

        telem_rows = [
            ("Controller Type",  "—"),
            ("Axes Detected",    "—"),
            ("Buttons Detected", "—"),
            ("Deadzone",         str(DEADZONE)),
            ("Last Axis 0",      "0.000"),
            ("Last Axis 1",      "0.000"),
            ("Last Axis 2",      "0.000"),
            ("Last Axis 3",      "0.000"),
            ("Active Button",    "—"),
        ]
        self.telem_values = {}
        for i, (label, default) in enumerate(telem_rows):
            bg_row = BG3 if i % 2 == 0 else BG
            tk.Label(telem_frame, text=label,
                     bg=bg_row, fg=FG2, font=("Courier New", 9),
                     anchor="w", width=20).grid(row=i, column=0,
                                                sticky="ew", padx=(4, 8), pady=1)
            val = tk.Label(telem_frame, text=default,
                           bg=bg_row, fg=ACCENT,
                           font=("Courier New", 9, "bold"), anchor="w", width=16)
            val.grid(row=i, column=1, sticky="ew", pady=1)
            self.telem_values[label] = val

        # Axis mapping
        map_frame = ttk.LabelFrame(content, text=" Axis → Joint Mapping ", padding=10)
        map_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        map_frame.columnconfigure(list(range(8)), weight=1)
        self.axis_map_vars = []
        for i in range(4):
            tk.Label(map_frame, text=f"Axis {i} →",
                     bg=BG, fg=FG2,
                     font=("Courier New", 9)).grid(row=0, column=i*2,
                                                   padx=(8, 2), pady=4)
            var = tk.StringVar(value=f"J{i+1}")
            cb = ttk.Combobox(map_frame, textvariable=var, width=5,
                              values=["J1","J2","J3","J4","J5","—"],
                              state="readonly")
            cb.grid(row=0, column=i*2+1, padx=(0, 12), pady=4)
            self.axis_map_vars.append(var)

    # ═════════════════════════════════════════════════════════════
    #  TAB 3 — Gesture & Vision
    # ═════════════════════════════════════════════════════════════

    def _build_vision_tab(self):
        p = self.tab_vision
        ttk.Label(p, text="Gesture & Vision Control",
                  style="Header.TLabel").pack(anchor="w", padx=16, pady=(14, 4))
        ttk.Separator(p, orient="horizontal").pack(fill="x", padx=12, pady=(0, 10))

        content = tk.Frame(p, bg=BG)
        content.pack(fill="both", expand=True, padx=16)
        content.rowconfigure(0, weight=3)
        content.rowconfigure(1, weight=1)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=1)

        cam_outer = tk.Frame(content, bg=BG2, bd=1, relief="flat")
        cam_outer.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(0, 8))
        self.camera_label = tk.Label(cam_outer,
                                      text="[ Camera Feed ]\n\nPress Start Camera to begin",
                                      bg=BG2, fg=FG2,
                                      font=("Courier New", 11), anchor="center")
        self.camera_label.pack(fill="both", expand=True, padx=2, pady=2)

        right = tk.Frame(content, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")

        gest_frame = ttk.LabelFrame(right, text=" Gesture State ", padding=10)
        gest_frame.pack(fill="x", pady=(0, 10))
        gesture_rows = [
            ("Left thumb",  "—"), ("Left index",   "—"), ("Left middle", "—"),
            ("Right thumb", "—"), ("Right index",  "—"), ("Right middle","—"),
            ("Homing",      "False"), ("Active mode", "—"),
        ]
        self.gesture_values = {}
        for i, (label, default) in enumerate(gesture_rows):
            bg_row = BG3 if i % 2 == 0 else BG
            tk.Label(gest_frame, text=label,
                     bg=bg_row, fg=FG2, font=("Courier New", 9),
                     anchor="w", width=14).grid(row=i, column=0, sticky="ew", pady=1)
            val = tk.Label(gest_frame, text=default,
                           bg=bg_row, fg=ACCENT2,
                           font=("Courier New", 9, "bold"), anchor="w", width=10)
            val.grid(row=i, column=1, sticky="ew", pady=1)
            self.gesture_values[label] = val

        cam_btn = tk.Frame(right, bg=BG)
        cam_btn.pack(fill="x", pady=(0, 8))
        ttk.Button(cam_btn, text="▶ Start Camera",
                   style="Accent.TButton",
                   command=self._start_camera).pack(fill="x", pady=2)
        ttk.Button(cam_btn, text="■ Stop Camera",
                   command=self._stop_camera).pack(fill="x", pady=2)
        ttk.Button(cam_btn, text="⌂ Home Position",
                   command=self._home_position).pack(fill="x", pady=2)

        gp_frame = ttk.LabelFrame(right, text=" Goal Positions ", padding=6)
        gp_frame.pack(fill="x")
        self.goal_pos_labels = []
        for i in range(5):
            row_bg = BG3 if i % 2 == 0 else BG
            tk.Label(gp_frame, text=f"Motor {i+1}",
                     bg=row_bg, fg=FG2,
                     font=("Courier New", 9), width=9).grid(row=i, column=0,
                                                             sticky="ew", pady=1)
            lbl = tk.Label(gp_frame, text="512",
                           bg=row_bg, fg=WARNING,
                           font=("Courier New", 9, "bold"), width=6)
            lbl.grid(row=i, column=1, sticky="ew", pady=1)
            self.goal_pos_labels.append(lbl)

        log_outer = tk.Frame(content, bg=BG)
        log_outer.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(0, 4))
        tk.Label(log_outer, text="Console Log",
                 bg=BG, fg=FG2,
                 font=("Courier New", 9, "bold")).pack(anchor="w")
        self.console_log = ScrolledText(log_outer, height=6,
                                         bg=BG2, fg=ACCENT2,
                                         insertbackground=FG,
                                         font=("Courier New", 9),
                                         relief="flat", bd=4, wrap="word")
        self.console_log.pack(fill="both", expand=True)
        self.console_log.configure(state="disabled")
        self._log("System ready.")

    # ═════════════════════════════════════════════════════════════
    #  Status bar
    # ═════════════════════════════════════════════════════════════

    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg=BG2, height=26)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)
        self.estop_indicator = tk.Label(bar, text="● ESTOP: OFF",
                                         bg=BG2, fg=ACCENT2,
                                         font=("Courier New", 9, "bold"))
        self.estop_indicator.pack(side="left", padx=12)
        self.ctrl_indicator = tk.Label(bar, text="● CTRL: DISCONNECTED",
                                        bg=BG2, fg=FG2, font=("Courier New", 9))
        self.ctrl_indicator.pack(side="left", padx=12)
        self.cam_indicator = tk.Label(bar, text="● CAM: OFF",
                                       bg=BG2, fg=FG2, font=("Courier New", 9))
        self.cam_indicator.pack(side="left", padx=12)
        tk.Label(bar, text=f"Port: {SERIAL_PORT}  |  Baud: {SERIAL_BAUD}",
                 bg=BG2, fg=FG2, font=("Courier New", 9)).pack(side="right", padx=12)

    # ═════════════════════════════════════════════════════════════
    #  Controller indicator canvas
    # ═════════════════════════════════════════════════════════════

    def _draw_controller_indicator(self, active: bool):
        c = self.ctrl_canvas
        c.delete("all")
        cx, cy, r = 60, 60, 44
        colour = ACCENT2 if active else BG3
        for i in range(3, 0, -1):
            ar = r * i / 3
            c.create_oval(cx-ar, cy-ar, cx+ar, cy+ar,
                          fill=BG2 if i < 3 else colour,
                          outline=colour, width=1 if i < 3 else 2)
        c.create_oval(cx-10, cy-10, cx+10, cy+10,
                      fill=ACCENT2 if active else FG2, outline="")
        if active:
            c.create_oval(cx-r-4, cy-r-4, cx+r+4, cy+r+4,
                          outline=ACCENT2, width=1, dash=(4, 4))

    # ═════════════════════════════════════════════════════════════
    #  Console log
    # ═════════════════════════════════════════════════════════════

    def _log(self, message: str):
        ts = time.strftime("%H:%M:%S")
        self.console_log.configure(state="normal")
        self.console_log.insert("end", f"[{ts}]  {message}\n")
        self.console_log.see("end")
        self.console_log.configure(state="disabled")

    # ═════════════════════════════════════════════════════════════
    #  GUI display helpers
    # ═════════════════════════════════════════════════════════════

    def _on_slider_change(self, joint_idx: int, value: str):
        self._update_pose_display()

    def _on_entry_change(self, joint_idx: int):
        self._update_pose_display()

    def _update_pose_display(self):
        for i, var in enumerate(self.joint_vars):
            try:
                self.pose_labels[i].config(text=f"{var.get():.1f}°")
            except Exception:
                pass

    def _update_ctrl_joint_positions(self, positions: list):
        for i, pos in enumerate(positions[:5]):
            self.ctrl_joint_labels[i].config(text=str(pos))

    def _update_telemetry(self, key: str, value: str):
        if key in self.telem_values:
            self.telem_values[key].config(text=value)

    def _update_goal_positions(self, positions: list):
        for i, pos in enumerate(positions[:5]):
            self.goal_pos_labels[i].config(text=str(pos))

    def _update_gesture_state(self, state: dict):
        for key, val in state.items():
            if key in self.gesture_values:
                self.gesture_values[key].config(text=val)

    # ═════════════════════════════════════════════════════════════
    #  Tab 1 — Manual joint move
    # ═════════════════════════════════════════════════════════════

    def _execute_joint_move(self):
        if self.estop_active.get():
            self._log("⚠ Cannot move — E-stop is active.")
            return
        if not self.portHandler:
            self._log("⚠ Dynamixel not connected.")
            return
        angles = [v.get() for v in self.joint_vars]
        for i in range(1,5):
            counts = int((angles[i] + 150) / 300.0 * 1023)
            self.GoalPos[i] = max(0, min(1023, counts))
            print(f'Motor{i} at output {self.GoalPos[i]}')

        if self.homed:
            self.data = 8
            self.GoalPos[0] = round(angles[0])
        self._write_goal_positions()
        self._update_pose_display()
        self._update_goal_positions(self.GoalPos)
        self._log(f"Move → {[f'{a:.1f}°' for a in angles]}")

    def _reset_joints(self):
        for var in self.joint_vars:
            var.set(0.0)
        self.joint_vars[0].set(80.0)
        self._update_pose_display()
        self._log("Joints reset to 0°.")

    # ═════════════════════════════════════════════════════════════
    #  E-stop
    # ═════════════════════════════════════════════════════════════

    def _emergency_stop(self, from_arduino: bool = False):
        """Halt all motion, disable torque, send stop byte to Arduino."""
        self.estop_active.set(True)
        self.start = False
        self.data  = 0
        #self._disable_all_torque() #is this necessary
        self._send_2int(0,0)
        self.estop_indicator.config(text="● ESTOP: ACTIVE", fg=DANGER)
        if hasattr(self, 'start_lbl'):
            self.start_lbl.config(text="● START: OFF", fg=DANGER)
        src = " (Arduino)" if from_arduino else ""
        self._log(f"⚠ EMERGENCY STOP TRIGGERED{src}")

    def _reset_estop(self):
        """Re-enable torque and clear e-stop flag."""
        self.estop_active.set(False)
        self._enable_all_torque()
        self.estop_indicator.config(text="● ESTOP: OFF", fg=ACCENT2)
        self._log("E-stop reset. Press Start (button 5) to enable motion.")

    # ═════════════════════════════════════════════════════════════
    #  Home
    # ═════════════════════════════════════════════════════════════

    def _home_position(self):
        if self.estop_active.get():
            self._log("⚠ Cannot home — E-stop is active.")
            return
        self.data = 4
        for i in range(4):
            self.GoalPos[i+1] = 512
        self._write_goal_positions()
        self._update_goal_positions(self.GoalPos)
        self._reset_joints()
        self._log("Homing all motors to position 512.")

    # ═════════════════════════════════════════════════════════════
    #  Tab 2 — Controller
    # ═════════════════════════════════════════════════════════════

    def _connect_controller(self):
        try:
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                self._log("⚠ No joystick detected.")
                return
            self._joy = pygame.joystick.Joystick(0)
            self._joy.init()
            self.controller_active.set(True)
            self._draw_controller_indicator(active=True)
            self.ctrl_status_lbl.config(text="CONNECTED", fg=ACCENT2)
            self.ctrl_indicator.config(text="● CTRL: CONNECTED", fg=ACCENT2)
            self._update_telemetry("Controller Type", self._joy.get_name()[:16])
            self._update_telemetry("Axes Detected",    str(self._joy.get_numaxes()))
            self._update_telemetry("Buttons Detected", str(self._joy.get_numbuttons()))
            self._log(f"Controller connected: {self._joy.get_name()}")
        except Exception as e:
            self._log(f"⚠ Controller connect error: {e}")

    def _disconnect_controller(self):
        self.controller_active.set(False)
        self._joy = None
        self._draw_controller_indicator(active=False)
        self.ctrl_status_lbl.config(text="DISCONNECTED", fg=DANGER)
        self.ctrl_indicator.config(text="● CTRL: DISCONNECTED", fg=FG2)
        self._log("Controller disconnected.")

    def _controller_poll(self):
        """Poll joystick every 20 ms — mirrors original controller loop."""
        if self.controller_active.get() and self._joy and not self.estop_active.get():
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    self.lastButton = event.button
                    self._update_telemetry("Active Button", str(event.button))

                    # Toggle start with button 5 (RB)
                    if event.button == 5:
                        self.start = not self.start
                        colour = ACCENT2 if self.start else DANGER
                        self.start_lbl.config(
                            text="● START: ON" if self.start else "● START: OFF",
                            fg=colour)
                        self._log(f"Start {'enabled' if self.start else 'disabled'}.")

                    if event.button in (9, 10):
                        self.moveStepper = True

                elif event.type == pygame.JOYBUTTONUP:
                    if self.lastButton in (9, 10):
                        # Freeze at current position when deadman released
                        self._read_current_positions()
                        for i in range(1,5):
                            self.GoalPos[i] = self.currentPos[i]
                        self.moveStepper = False
                    self._update_telemetry("Active Button", "—")

                elif event.type == pygame.JOYDEVICEADDED:
                    joy = pygame.joystick.Joystick(event.device_index)
                    self.joysticks[joy.get_instance_id()] = joy
                elif event.type == pygame.JOYDEVICEREMOVED:
                    self.joysticks.pop(event.instance_id, None)

            if self.start and self._joy:
                lt = self._joy.get_axis(4)
                rt = self._joy.get_axis(5)
                deadman_held = lt > 0.5 or rt > 0.5

                if deadman_held:
                    for i in range(0,4):
                        axis = self._joy.get_axis(i)
                        self._update_telemetry(f"Last Axis {i}", f"{axis:.3f}")
                        if abs(axis) > DEADZONE:
                            self.GoalPos[i+1] = max(0, min(1023,
                                self.GoalPos[i+1] + int(axis * JOG_SPEED)))
                        if self.portHandler:
                            self.packetHandler.write2ByteTxRx(
                                self.portHandler, DXL_IDS[i],
                                GoalPosition, self.GoalPos[i+1])

                    if self.moveStepper:
                        self.data = 3 if self.lastButton == 10 else 1
                    else:
                        self.data = 0
                else:
                    self.data = 0

                # Homing — button 15
                if self.lastButton == 15:
                    self._home_position()
                    self.lastButton = 0

                self._update_ctrl_joint_positions(self.GoalPos)
                self._update_goal_positions(self.GoalPos)

        self.root.after(20, self._controller_poll)

    # ═════════════════════════════════════════════════════════════
    #  Tab 3 — Camera
    # ═════════════════════════════════════════════════════════════

    def _start_camera(self):
        if self.camera_running:
            return
        try:
            self._cap = cv2.VideoCapture(0)
            if not self._cap.isOpened():
                self._log("⚠ Camera not found.")
                return
            self.camera_running = True
            self._camera_stop.clear()
            t = threading.Thread(target=self._camera_loop, daemon=True)
            t.start()
            self.cam_indicator.config(text="● CAM: ACTIVE", fg=ACCENT2)
            self._log("Camera started.")
        except Exception as e:
            self._log(f"⚠ Camera error: {e}")

    def _stop_camera(self):
        self._camera_stop.set()
        self.camera_running = False
        if self._cap:
            self._cap.release()
            self._cap = None
        self.cam_indicator.config(text="● CAM: OFF", fg=FG2)
        self.camera_label.config(image="",
                                  text="[ Camera Feed ]\n\nPress Start Camera to begin")
        self._log("Camera stopped.")

    def _camera_loop(self):
        """Daemon thread — reads frames, runs gesture logic, updates GUI."""
        while not self._camera_stop.is_set():
            if not self._cap or not self._cap.isOpened():
                break
            good, cam = self._cap.read()
            if not good:
                continue

            hands, cam = self.detector.findHands(cam)

            if not self.estop_active.get():
                self._process_gesture(hands)

            nowTime = time.time()
            try:
                fps = int(1 / max(nowTime - self.previousTime, 0.001))
            except ZeroDivisionError:
                fps = 0
            self.previousTime = nowTime
            cvzone.putTextRect(cam, str(fps), (50, 50), scale=2, thickness=2,
                               colorR=(0, 0, 255))

            try:
                img = ImageTk.PhotoImage(
                    Image.fromarray(cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)))
                self.root.after(0, self._update_camera_label, img)
            except Exception:
                pass

    def _update_camera_label(self, photo_image):
        self.camera_label.config(image=photo_image, text="")
        self.camera_label.image = photo_image

    # ═════════════════════════════════════════════════════════════
    #  Gesture processing (mirrors original camera loop logic)
    # ═════════════════════════════════════════════════════════════

    def _process_gesture(self, hands: list):
        """Process hand landmarks and update GoalPos / self.data."""
        nowTime = time.time()

        # Reset all finger arrays each frame
        self.fingersLeft = [0] * 5
        self.fingersRight = [0] * 5
        self.thumbLeft = [0] * 4
        self.indexLeft = [0] * 4
        self.middleLeft = [0] * 4
        self.thumbRight = [0] * 4
        self.indexRight = [0] * 4
        self.middleRight = [0] * 4

        if hands:
            if len(hands) > 0:
                self._assign_hand_landmarks(hands[0]["type"], hands[0]["lmList"])
            if len(hands) > 1 and hands[0]["type"] != hands[1]["type"]:
                self._assign_hand_landmarks(hands[1]["type"], hands[1]["lmList"])
        else:
            # No hands visible — freeze joints at current position
            self._read_current_positions()
            for i in range(1,5):
                self.GoalPos[i] = self.currentPos[i]



        # ── Finger-up detection ───────────────────────────────────
        # Left thumb → stepper direction
        if abs(self.thumbLeft[0] - self.thumbLeft[2]) > self.thumbOffset and not self.homing:
            self.fingersLeft[0] = 1
        else:
            self.fingersLeft[0] = 0

        # Left index / middle → joints 0 and 1
        if abs(self.indexLeft[1] - self.indexLeft[3]) > self.indexVerticalOffset and not self.homing:
            if abs(self.middleLeft[1] - self.middleLeft[3]) > self.middleVerticalOffset:
                self.fingersLeft[2] = 1;  self.fingersLeft[1] = 0
                self.leftDX[1] = max(min(self.middleLeft[2]  - self.middleLeft[0],  CAM_MAX_DX), -CAM_MAX_DX)
                self.leftDX[0] = 0
            else:
                self.fingersLeft[1] = 1;  self.fingersLeft[2] = 0
                self.leftDX[0] = max(min(self.indexLeft[2]   - self.indexLeft[0],   CAM_MAX_DX), -CAM_MAX_DX)
                self.leftDX[1] = 0
        else:
            self.fingersLeft[1] = self.fingersLeft[2] = 0

        # Right thumb → stepper direction
        if abs(self.thumbRight[0] - self.thumbRight[2]) > self.thumbOffset and not self.homing:
            self.fingersRight[0] = 1
        else:
            self.fingersRight[0] = 0

        # Right index / middle → joints 2 and 3
        if abs(self.indexRight[1] - self.indexRight[3]) > self.indexVerticalOffset and not self.homing:
            if abs(self.middleRight[1] - self.middleRight[3]) > self.middleVerticalOffset:
                self.fingersRight[2] = 1;  self.fingersRight[1] = 0
                self.rightDX[1] = max(min(self.middleRight[2] - self.middleRight[0], CAM_MAX_DX), -CAM_MAX_DX)
                self.rightDX[0] = 0
            else:
                self.fingersRight[1] = 1;  self.fingersRight[2] = 0
                self.rightDX[0] = max(min(self.indexRight[2]  - self.indexRight[0],  CAM_MAX_DX), -CAM_MAX_DX)
                self.rightDX[1] = 0
        else:
            self.fingersRight[1] = self.fingersRight[2] = 0

        # Stepper command from thumbs
        if not self.homing:
            if self.fingersLeft[0] == 1 and self.fingersRight[0] != 1:
                self.data = 3
            elif self.fingersRight[0] == 1 and self.fingersLeft[0] != 1:
                self.data = 1
            else:
                self.data = 0

        # Joint movements from finger DX
        if self.fingersLeft[1]  and abs(self.leftDX[0])  > self.indexHorizontalOffset:
            self.GoalPos[1] += int(self.leftDX[0]  * CAM_JOG_SPEED)
        if self.fingersLeft[2]  and abs(self.leftDX[1])  > self.indexHorizontalOffset:
            self.GoalPos[2] += int(self.leftDX[1]  * CAM_JOG_SPEED)
        if self.fingersRight[1] and abs(self.rightDX[0]) > self.indexHorizontalOffset:
            self.GoalPos[3] += int(self.rightDX[0] * CAM_JOG_SPEED)
        if self.fingersRight[2] and abs(self.rightDX[1]) > self.indexHorizontalOffset:
            self.GoalPos[4] += int(self.rightDX[1] * CAM_JOG_SPEED)

        # Clamp and write to motors
        for i in range(1,5):
            self.GoalPos[i] = max(0, min(1023, self.GoalPos[i]))
        self._write_goal_positions()

        # upLeft/upRight for homing timer
        self.upLeft = 1 if self.fingersLeft[0] and any(self.fingersLeft[2:5]) else 0
        self.upRight = 1 if self.fingersRight[0] and any(self.fingersRight[2:5]) else 0
        print(f"Left Hand: {self.fingersLeft}, Right Hand: {self.fingersRight}")

        if not self.upLeft and not self.upRight:
            self.previousHomingTime = nowTime
            self.homing = False

        if nowTime - self.previousHomingTime > HOMING_TIME:
            self.data = 4
            self.homing = True
            for i in range(1, 5):
                self.GoalPos[i] = 512


        # Update GUI panels (main-thread safe)
        self.root.after(0, self._update_goal_positions, list(self.GoalPos))
        self.root.after(0, self._update_gesture_state, {
            "Left thumb":   str(self.fingersLeft[0]),
            "Left index":   str(self.fingersLeft[1]),
            "Left middle":  str(self.fingersLeft[2]),
            "Right thumb":  str(self.fingersRight[0]),
            "Right index":  str(self.fingersRight[1]),
            "Right middle": str(self.fingersRight[2]),
            "Homing":       str(self.homing),
            "Active mode":  "Camera",
        })

    def _assign_hand_landmarks(self, hand_type: str, lm: list):
        """Extract thumb / index / middle landmarks and ring/pinky homing flags."""
        if hand_type == "Left":
            self.thumbLeft[0],   self.thumbLeft[1]   = lm[2][0:2]
            self.thumbLeft[2],   self.thumbLeft[3]   = lm[4][0:2]
            self.indexLeft[0],   self.indexLeft[1]   = lm[5][0:2]
            self.indexLeft[2],   self.indexLeft[3]   = lm[8][0:2]
            self.middleLeft[0],  self.middleLeft[1]  = lm[9][0:2]
            self.middleLeft[2],  self.middleLeft[3]  = lm[12][0:2]
            self.fingersLeft[3] = 1 if abs(lm[3*4+1][1] - lm[3*5+(4-3)][1]) > self.indexVerticalOffset else 0
            self.fingersLeft[4] = 1 if abs(lm[4*4+1][1] - lm[4*5+(4-4)][1]) > self.indexVerticalOffset else 0

        elif hand_type == "Right":
            self.thumbRight[0],  self.thumbRight[1]  = lm[2][0:2]
            self.thumbRight[2],  self.thumbRight[3]  = lm[4][0:2]
            self.indexRight[0],  self.indexRight[1]  = lm[5][0:2]
            self.indexRight[2],  self.indexRight[3]  = lm[8][0:2]
            self.middleRight[0], self.middleRight[1] = lm[9][0:2]
            self.middleRight[2], self.middleRight[3] = lm[12][0:2]
            self.fingersRight[3] = 1 if abs(lm[3 * 4 + 1][1] - lm[3 * 5 + (4 - 3)][1]) > self.indexVerticalOffset else 0
            self.fingersRight[4] = 1 if abs(lm[4 * 4 + 1][1] - lm[4 * 5 + (4 - 4)][1]) > self.indexVerticalOffset else 0


# ─────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    root.configure(bg=BG)
    app = RobotControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()