import logging
import tkinter as tk
import tkinter.font as tkFont
from queue import Queue
from tkinter import filedialog, ttk
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

from integral_timber_joints.process.action import *
from integral_timber_joints.process.movement import *

from process_controller.background_command import *
from process_controller.ProcessModel import (RobotClampExecutionModel,
                                                 RunStatus)

logger_ui = logging.getLogger("app.gui")


#############
# UI Creation
#############

def create_execution_gui(root, q):
    tk.font_key = tkFont.Font(family="Lucida Grande", size=10)
    tk.font_value = tkFont.Font(family="Lucida Console", size=20)
    tk.big_button_font = tkFont.Font(family="Lucida Console", size=15)
    tk.small_button_font = tkFont.Font(family="Lucida Console", size=8)
    tk.big_status_font = tkFont.Font(
        family="Lucida Console", size=25, weight='bold')

    ui_handles = {}
    ui_handles['root'] = root
    ui_handles['ros'] = create_ui_ros(root, q)
    ui_handles['process'] = create_ui_process(root, q)
    ui_handles['exe'] = create_ui_execution(root, q)
    ui_handles['offset'] = create_ui_offset(root, q)
    return ui_handles


def create_ui_ros(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="ROS Connection")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_robot_ros_connect_button_click(event=None):
        logger_ui.info("Button Pressed: Connect to Robot ROS Host")
        ip = robot_ip_entrybox.get()
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_ROBOT_CONNECT, ip=ip))

    tk.Label(frame, text="ROS Robot Host IP Address: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['robot_ip_entry'] = tk.StringVar(value="127.0.0.0")
    robot_ip_entrybox = tk.Entry(
        frame, textvariable=ui_handles['robot_ip_entry'])
    robot_ip_entrybox.pack(side=tk.LEFT)
    tk.Button(frame, text="Connect",
              command=on_robot_ros_connect_button_click).pack(side=tk.LEFT)

    def on_open_settings_button_click(event=None):
        logger_ui.info("Button Pressed: Open Settings")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_OPEN_SETTING))
    tk.Button(frame, text="Speed Settings",
              command=on_open_settings_button_click).pack(side=tk.LEFT)
    # Status Label
    tk.Label(frame, text="Status: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['robot_status'] = tk.StringVar(value="Not Connected")
    tk.Label(frame, textvariable=ui_handles['robot_status'], font=tk.font_key, anchor=tk.SE).pack(
        side=tk.LEFT, fill=tk.Y, padx=10)

    # Clamp Connections
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_clamp_ros_connect_button_click(event=None):
        logger_ui.info("Button Pressed: Connect to Clamps ROS Host")
        ip = clamp_ip_entrybox.get()
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_CLAMP_CONNECT, ip=ip))

    tk.Label(frame, text="ROS Clamp Host IP Address: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['clamp_ip_entry'] = tk.StringVar(value="127.0.0.0")
    clamp_ip_entrybox = tk.Entry(
        frame, textvariable=ui_handles['clamp_ip_entry'])
    clamp_ip_entrybox.pack(side=tk.LEFT)
    tk.Button(frame, text="Connect",
              command=on_clamp_ros_connect_button_click).pack(side=tk.LEFT)
    # Status Label
    tk.Label(frame, text="Status: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['clamp_status'] = tk.StringVar(value="Not Connected")
    tk.Label(frame, textvariable=ui_handles['clamp_status'], font=tk.font_key, anchor=tk.SE).pack(
        side=tk.LEFT, fill=tk.Y, padx=10)

    return ui_handles


def create_ui_process(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Process JSON")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_load_process_button_click(event=None):
        logger_ui.info("Button Pressed: Load Json")
        filename = filedialog.askopenfilename(
            initialdir="/", title="Select file", filetypes=(("json files", "*.json"), ("all files", "*.*")))
        if filename == "":
            logger_ui.info("User canceled the Load Json File Dialog.")
        else:
            logger_ui.info(
                "User Selected %s from Load Json File Dialog." % filename)
            q.put(SimpleNamespace(
                type=ProcessControllerBackgroundCommand.MODEL_LOAD_PROCESS, json_path=filename))

    def on_load_ext_movement_button_click(event=None):
        logger_ui.info("Button Pressed: Load Ext Movement")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_LOAD_EXT_MOVEMENT))

    tk.Label(frame, text="Process JSON: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    # Status Label
    ui_handles['process_status'] = tk.StringVar(value="Not Loaded")
    tk.Label(frame, textvariable=ui_handles['process_status'], font=tk.font_key, anchor=tk.SE).pack(
        side=tk.LEFT, fill=tk.Y, padx=10)
    # Load Process Button
    tk.Button(frame, text="Load Json File.",
              command=on_load_process_button_click).pack(side=tk.LEFT)
    # Load External Movement Button
    tk.Button(frame, text="Reload External Json",
              command=on_load_ext_movement_button_click).pack(side=tk.LEFT)

    # DropDown List of all beams
    def on_goto_beam_combobox_selected(eventObject):
        beam_id = ui_handles['goto_beam_combobox'].get()
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_TREEVIEW_GOTO_BEAM, beam_id=beam_id))

    ui_handles['goto_beam_value'] = tk.StringVar(value="")
    goto_beam_combobox = ttk.Combobox(frame, width=27, textvariable=ui_handles['goto_beam_value'])
    goto_beam_combobox.pack(side=tk.LEFT, padx=6)
    goto_beam_combobox.bind("<<ComboboxSelected>>", on_goto_beam_combobox_selected)
    ui_handles['goto_beam_combobox'] = goto_beam_combobox

    # Test button
    def on_test_button0_click(event=None,):
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.TEST, id=0))
    tk.Button(frame, text="TestButton0",
              command=on_test_button0_click).pack(side=tk.LEFT, padx=6)

    # Test button
    def on_test_button1_click(event=None,):
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.TEST, id=1))
    tk.Button(frame, text="TestButton1 - Show JSON",
              command=on_test_button1_click).pack(side=tk.LEFT, padx=6)

    # Test button
    def on_restart_camera_click(event=None,):
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_RESTART_CAMERA))
    tk.Button(frame, text="Restart Docking Camera",
              command=on_restart_camera_click).pack(side=tk.LEFT, padx=6)

    # Second Frame holds the treeview for process Movements and Actions List
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=1, side=tk.TOP, padx=6, pady=3)
    tree = ttk.Treeview(frame, selectmode='browse')

    tree["columns"] = ("movement_id", "description", "details",
                       "traj_points", "speed_type", "speed", "conf_st", "conf_en", 'beam_id')
    tree["displaycolumns"] = (
        "movement_id", "description", "details", "traj_points", "speed_type", "speed", "conf_st", "conf_en")

    tree.column("#0", width=150, minwidth=20, stretch=tk.NO)
    tree.column("movement_id", width=100, minwidth=30, stretch=tk.NO)
    tree.column("description", width=180, minwidth=30, stretch=tk.NO)
    tree.column("details", width=200, minwidth=30)
    tree.column("traj_points", width=100, minwidth=20, stretch=tk.NO)
    tree.column("speed_type", width=100, minwidth=20, stretch=tk.NO)
    tree.column("speed", width=40, minwidth=20, stretch=tk.NO)
    tree.column("conf_st", width=40, minwidth=20, stretch=tk.NO)
    tree.column("conf_en", width=40, minwidth=20, stretch=tk.NO)

    tree.heading("#0", text="Name", anchor=tk.W)
    tree.heading("movement_id", text="movement_id", anchor=tk.W)
    tree.heading("description", text="Description", anchor=tk.W)
    tree.heading("details", text="Details", anchor=tk.W)
    tree.heading("traj_points", text="TrajectoryPoints", anchor=tk.W)
    tree.heading("speed_type", text="Speed Type", anchor=tk.W)
    tree.heading("speed", text="mm/s", anchor=tk.W)
    tree.heading("conf_st", text="conf_st", anchor=tk.W)
    tree.heading("conf_en", text="conf_en", anchor=tk.W)
    tree.heading("beam_id", text="beam_id", anchor=tk.W)

    tree.pack(fill=tk.BOTH, expand=1, padx=6, pady=3, side=tk.LEFT)

    # scrollbar
    vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    # vsb.place(relx=0.978, rely=0.175, relheight=0.713, relwidth=0.020)
    vsb.pack(side='right', fill='y')
    tree.configure(yscrollcommand=vsb.set)

    ui_handles['tree'] = tree
    return ui_handles


def create_ui_execution(root, q: Queue):
    """Creates Lower UI Frame for execution status and controls"""
    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Execution / Run")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    left_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=30)
    left_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    # Button Handle
    def on_run_button_click(event=None):
        logger_ui.info("Button Pressed: RUN")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_RUN))
    ui_handles['run_button'] = tk.Button(
        left_frame, text="RUN", command=on_run_button_click, font=tk.big_button_font, width=15, state="disabled")
    ui_handles['run_button'].pack(side=tk.TOP)

    def on_step_button_click(event=None):
        logger_ui.info("Button Pressed: STEP")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_STEP))
    ui_handles['step_button'] = tk.Button(
        left_frame, text="STEP", command=on_step_button_click, font=tk.big_button_font, width=15, height=3, state="disabled")
    ui_handles['step_button'].pack(side=tk.TOP)

    def on_step_from_point_button_click(event=None):
        logger_ui.info("Button Pressed: STEP FRON POINT")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_STEP_FROM_POINT))
    ui_handles['step_from_pt_button'] = tk.Button(
        left_frame, text="STEP from Pt", command=on_step_from_point_button_click, font=tk.big_button_font, width=15, state="disabled")
    ui_handles['step_from_pt_button'].pack(side=tk.TOP)

    def on_stop_button_click(event=None):
        logger_ui.info("Button Pressed: STOP")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_STOP))
    ui_handles['stop_button'] = tk.Button(
        left_frame, text="STOP", command=on_stop_button_click, font=tk.big_button_font, width=15, state="disabled")
    ui_handles['stop_button'].pack(side=tk.TOP)

    run_status_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    run_status_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    ui_handles['exe_status'] = tk.StringVar(value="Stopped")
    ui_handles['exe_status_label'] = tk.Label(run_status_frame, textvariable=ui_handles['exe_status'], font=tk.big_status_font, anchor=tk.CENTER, height=2)
    ui_handles['exe_status_label'].pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    def on_confirm_button_click(event=None):
        logger_ui.info("Button Pressed: Confirm")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_CONFIRM))
        confirm_button.config(state="disabled")

    ui_handles['confirm_button_text'] = tk.StringVar(value="Confirm?")
    ui_handles['confirm_button'] = confirm_button = tk.Button(run_status_frame, textvariable=ui_handles['confirm_button_text'],
                                                              command=on_confirm_button_click, font=tk.big_button_font, width=20, height=3, state="disabled", bg='grey')
    ui_handles['confirm_button'].pack(side=tk.BOTTOM)

    robot_status_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    robot_status_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    ui_handles['last_completed_trajectory_point'] = tk.StringVar(value=" - ")
    tk.Label(robot_status_frame, text="Last Trajectory Point", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    tk.Label(robot_status_frame, textvariable=ui_handles['last_completed_trajectory_point'], font=tk.big_status_font, anchor=tk.CENTER).pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    ui_handles['last_executed_movement'] = tk.StringVar(value=" - ")
    tk.Label(robot_status_frame, text="Last Movement", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    tk.Label(robot_status_frame, textvariable=ui_handles['last_executed_movement'], font=tk.big_status_font, anchor=tk.CENTER).pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    ui_handles['toolchanger_signal'] = tk.StringVar(value=" - ")
    tk.Label(robot_status_frame, text="Toolchanger signal", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    tk.Label(robot_status_frame, textvariable=ui_handles['toolchanger_signal'], font=tk.big_status_font, anchor=tk.CENTER).pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    ui_handles['start_end_distance'] = tk.StringVar(value=" - ")
    tk.Label(robot_status_frame, text="Movement Distance", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    tk.Label(robot_status_frame, textvariable=ui_handles['start_end_distance'], font=tk.big_status_font, anchor=tk.CENTER).pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    right_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    right_frame.pack(fill=tk.BOTH, expand=1, side=tk.LEFT, padx=6, pady=3)

    clamp_status_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    clamp_status_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    ui_handles['clamps_running'] = tk.StringVar(value=" - ")
    tk.Label(clamp_status_frame, text="Clamps Running", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    ui_handles['clamps_running_label'] = tk.Label(clamp_status_frame, textvariable=ui_handles['clamps_running'], font=tk.big_status_font, anchor=tk.CENTER)
    ui_handles['clamps_running_label'].pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    ui_handles['clamps_last_cmd_success'] = tk.StringVar(value=" - ")
    tk.Label(clamp_status_frame, text="Clamps Last Command", anchor=tk.W).pack(side=tk.TOP, fill=tk.BOTH, padx=10)
    ui_handles['clamps_last_cmd_success_label'] = tk.Label(clamp_status_frame, textvariable=ui_handles['clamps_last_cmd_success'], font=tk.big_status_font, anchor=tk.CENTER)
    ui_handles['clamps_last_cmd_success_label'].pack(side=tk.TOP, fill=tk.BOTH, padx=10)

    def on_goto_start_state_button_click(event=None):
        logger_ui.info("Button Pressed: GOTO Start State")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_GOTO_START_STATE))
    tk.Button(right_frame, text="GOTO Start State", command=on_goto_start_state_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_goto_end_state_button_click(event=None):
        logger_ui.info("Button Pressed: GOTO END State")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_GOTO_END_STATE))
    tk.Button(right_frame, text="GOTO End State", command=on_goto_end_state_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_print_summary_button_click(event=None):
        logger_ui.info("Button Pressed: Print Selected Beam Action Summary")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.PRINT_ACTION_SUMMARY))
    tk.Button(right_frame, text="Print Selected Beam Action Summary",
              command=on_print_summary_button_click, font=tk.small_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_shake_gantry_button_click(event=None):
        logger_ui.info("Button Pressed: Shake Gantry")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_SHAKE_GANTRY_POPUP))
    tk.Button(right_frame, text="SHAKE Gantry", command=on_shake_gantry_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_tool_changer_probe_button_click(event=None):
        logger_ui.info("Button Pressed: Compare joint values")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_TOOLCHANGER_PROBE))
    tk.Button(right_frame, text="Tool Changer Probe", command=on_tool_changer_probe_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_compare_joint_value_button_click(event=None):
        logger_ui.info("Button Pressed: Compare joint values")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_COMPARE_JOINT_VALUES))
    tk.Button(right_frame, text="Compare Joint Values", command=on_compare_joint_value_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)


    right_frame_2 = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    right_frame_2.pack(fill=tk.BOTH, expand=1, side=tk.LEFT, padx=6, pady=3)

    # Enable softmove
    ui_handles['softmove_enable'] = tk.BooleanVar(value=False)
    row_frame = ttk.Frame(right_frame_2)
    row_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=0, pady=3)
    tk.Label(row_frame, text="Softmove Enable", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=3)
    tk.Checkbutton(row_frame, variable=ui_handles['softmove_enable']).pack(side=tk.RIGHT, fill=tk.BOTH)

    # Soft Direction Dropdown
    choices = {'Z', 'XY', 'XYZ', 'XYRZ'}
    ui_handles['soft_direction'] = tk.StringVar(value="XY")

    row_frame = ttk.Frame(right_frame_2)
    row_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=0, pady=3)
    tk.Label(row_frame, text="Soft Direction", anchor=tk.W).pack(side=tk.LEFT, padx=10)
    popupMenu = tk.OptionMenu(row_frame, ui_handles['soft_direction'], *choices).pack(side=tk.RIGHT, fill=tk.BOTH, padx=10)

    # Soft Amount
    ui_handles['stiffness_soft_dir'] = tk.StringVar(value="99")
    ui_handles['stiffness_nonsoft_dir'] = tk.StringVar(value="99")
    row_frame = ttk.Frame(right_frame_2)
    row_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=0, pady=3)
    tk.Label(row_frame, text="Stiffness Soft Dir", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=3)
    tk.Entry(row_frame, textvariable=ui_handles['stiffness_soft_dir'], width=10,  justify=tk.CENTER).pack(side=tk.RIGHT, fill=tk.BOTH)
    row_frame = ttk.Frame(right_frame_2)
    row_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=0, pady=3)
    tk.Label(row_frame, text="Stiffness Non-Soft Dir", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=3)
    tk.Entry(row_frame, textvariable=ui_handles['stiffness_nonsoft_dir'], width=10,  justify=tk.CENTER).pack(side=tk.RIGHT, fill=tk.BOTH)

    def on_goto_start_state_button_click(event=None):
        logger_ui.info("Button Pressed: Robot Soft Mode")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_SOFTMODE_ENABLE))
    tk.Button(right_frame_2, text="Robot Soft Mode", command=on_goto_start_state_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    def on_goto_end_state_button_click(event=None):
        logger_ui.info("Button Pressed: Robot Hard Mode")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_SOFTMODE_DISABLE))
    tk.Button(right_frame_2, text="Robot Hard Mode", command=on_goto_end_state_button_click,
              font=tk.big_button_font, width=20).pack(fill=tk.X, side=tk.TOP)

    return ui_handles


def create_ui_offset(root, q: Queue):
    """Creates Lower UI Frame for execution status and controls"""
    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Robot Offset / Setting")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)

    def create_text_field(name, frame, default_value = 0):
        ui_handles[name] = tk.StringVar(value=str(default_value))
        tk.Label(frame, text=name, anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=1)
        tk.Entry(frame, textvariable=ui_handles[name], width=10,  justify=tk.CENTER).pack(side=tk.LEFT, fill=tk.BOTH)

    # Correct by flange frame
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="Correction from Flange Frame:", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=1)

    create_text_field("Visual_X", frame)
    create_text_field("Visual_Y", frame)
    create_text_field("Visual_Z", frame)

    def on_compute_visual_alignment_click(event=None):
        logger_ui.info("Button Pressed: Compute Gantry Correction")
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_COMPUTE_VISUAL_CORRECTION))
    tk.Button(frame, text="Compute Gantry Correction", command=on_compute_visual_alignment_click,
              font=tk.small_button_font, width=25).pack(fill=tk.X, side=tk.LEFT)

    # Acceleration Setting

    tk.Label(frame, text="Acceleration:", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=1)
    create_text_field("Free_Acc", frame, 20)
    create_text_field("Free_Ramp", frame, 20)
    create_text_field("Linear_Acc", frame, 40)
    create_text_field("Linear_Ramp", frame, 40)

    # Correct by Joint Axis

    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="Correction by Joint (mm or deg)):", anchor=tk.W).pack(side=tk.LEFT, padx=10, pady=1)

    create_text_field("Ext_X", frame)
    create_text_field("Ext_Y", frame)
    create_text_field("Ext_Z", frame)

    create_text_field("Rob_J1", frame)
    create_text_field("Rob_J2", frame)
    create_text_field("Rob_J3", frame)
    create_text_field("Rob_J4", frame)
    create_text_field("Rob_J5", frame)
    create_text_field("Rob_J6", frame)

    return ui_handles


#####################################
# Helper functions to get values
#####################################

def get_softness_enable(guiref):
    return guiref['exe']['softmove_enable'].get()


def get_stiffness_soft_dir(guiref):
    return int(float(guiref['exe']['stiffness_soft_dir'].get()))


def get_stiffness_nonsoft_dir(guiref):
    return int(float(guiref['exe']['stiffness_nonsoft_dir'].get()))


def get_soft_direction(guiref):
    return guiref['exe']['soft_direction'].get()


def get_ext_offsets(guiref):
    offset = [
        float(guiref['offset']['Ext_X'].get()),
        float(guiref['offset']['Ext_Y'].get()),
        float(guiref['offset']['Ext_Z'].get()),
    ]
    return offset


def get_joint_offsets(guiref):
    offset = [
        float(guiref['offset']['Rob_J1'].get()),
        float(guiref['offset']['Rob_J2'].get()),
        float(guiref['offset']['Rob_J3'].get()),
        float(guiref['offset']['Rob_J4'].get()),
        float(guiref['offset']['Rob_J5'].get()),
        float(guiref['offset']['Rob_J6'].get()),
    ]
    return offset


def get_free_move_acc_ramp(guiref):
    offset = [
        float(guiref['offset']['Free_Acc'].get()),
        float(guiref['offset']['Free_Ramp'].get()),
    ]
    return offset


def get_linear_move_acc_ramp(guiref):
    offset = [
        float(guiref['offset']['Linear_Acc'].get()),
        float(guiref['offset']['Linear_Ramp'].get()),
    ]
    return offset


def apply_ext_offsets(guiref, ext_values):
    return [i+j for i, j in zip(ext_values, get_ext_offsets(guiref))]


def apply_joint_offsets(guiref, ext_values):
    return [i+j for i, j in zip(ext_values, get_joint_offsets(guiref))]


#####################################
# Helper functions for tree view
#####################################


def init_actions_tree_view(guiref, model: RobotClampExecutionModel):
    """ Initialize the actions treeview after laoding process

    The dictionary guiref['process']['item_ids'] holds a list of
    ttk TreeView iid values equal to each row's iid.

    - Beams : 'b_%i' = beam_id
    - Acion : 'a_%i' % action.act_n
    - Movement: "m%i_%i" % (action.act_n, move_n)
    """
    tree = guiref['process']['tree']  # type: ttk.Treeview
    process = model.process
    beam_item = None
    beam_id = ""

    # Place tree off screen for faster perfornance
    # tree.place(x=5000,y=110)
    guiref['process']['item_ids'] = []  # type : List[str]
    tree.delete(*tree.get_children())
    seq_n = None
    for i, action in enumerate(process.actions):
        # Beam Row
        if seq_n != action.seq_n:
            seq_n = action.seq_n

            beam_id = process.assembly.sequence[seq_n]
            beam_item = tree.insert(
                parent="", index="end", iid=beam_id, text="Beam %s" % beam_id, open=True,
                values=("", "", "", "-", "", "", "-", "-", beam_id))
            guiref['process']['item_ids'].append(beam_id)
        # Action Row
        description = action.__class__.__name__
        action_item = tree.insert(parent=beam_item, index="end", iid=action.tree_row_id, text="Action %i" % action.act_n,
                                  values=("", description, "%s" % action, "-", "", "", "-", "-", beam_id), open=True)
        guiref['process']['item_ids'].append(action.tree_row_id)

        # Movement Rows
        for j, movement in enumerate(action.movements):
            # Add row and delegate the fill-in to update_treeview_row()
            movement_item = tree.insert(parent=action_item, index="end", iid=movement.tree_row_id, text="Movement %i" % j,
                                        values=("", "", "", "", "", "", "", "", beam_id))
            update_treeview_row(guiref, model, movement)
            guiref['process']['item_ids'].append(movement.tree_row_id)

            # tree.see(movement_item)
            if tree.selection() == ():
                tree.selection_set(movement_item)

    # Pack the tree back on screen after updating
    # tree.pack(fill=tk.BOTH, expand=1, padx=6, pady=3, side=tk.LEFT)

    # Make sure we sees the first selected row
    tree.see(tree.selection())
    logger_ui.info("Actions Treeview Updated")


def update_treeview_row(guiref, model, movement):
    # type: (dict, RobotClampExecutionModel, Movement) -> None
    tree = guiref['process']['tree']  # type: ttk.Treeview
    tree_row_id = movement.tree_row_id

    # Retrive the values by row, change them and set them back.
    # This is the pattern supported by ttk
    row = tree.item(tree_row_id)
    values = row['values']

    # Generate trajectory description:
    if isinstance(movement, RoboticMovement):
        if movement.trajectory is None:
            traj_description = "Missing"
        else:
            traj_description = "%i Points" % len(movement.trajectory.points)
    else:
        traj_description = "- nil -"

    values[0] = movement.movement_id           # movement_id
    values[1] = movement.__class__.__name__    # description
    values[2] = movement.tag  # details
    values[3] = traj_description
    values[4] = movement.speed_type if hasattr(movement, 'speed_type') else ""
    values[5] = model.settings[movement.speed_type] if hasattr(movement, 'speed_type') else ""
    values[6] = "Yes" if model.process.movement_has_start_robot_config(movement) else ""
    values[7] = "Yes" if model.process.movement_has_end_robot_config(movement) else ""
    # values[8] = values[6] # Dont change the beam_id

    tree.item(tree_row_id, values=values)


def treeview_get_selected_id(guiref):
    # type: (dict) -> str
    """Returns the currently selected item's iid.
    In the format of "m%i_%i" % (act_n, mov_n)

    Beaware it may not be a movement."""
    tree = guiref['process']['tree']  # type: ttk.Treeview
    tree_row_id = tree.selection()[0]
    return tree_row_id


def treeview_get_selected_item_beam_id(guiref):
    # type: (dict) -> str
    """Returns the currently selected item's associated beam_id
    """
    tree = guiref['process']['tree']  # type: ttk.Treeview
    try:
        tree_row_id = tree.selection()[0]
    except:
        return None
    row = tree.item(tree_row_id)
    column_id = tree["columns"].index('beam_id')
    # Example print(row)
    # {'text': 'Beam b0', 'image': '', 'values': ['', '', '', '', '', '', 'b0'], 'open': 1, 'tags': ''}
    # {'text': 'Action 0', 'image': '', 'values': ['', 'LoadBeamAction', "Operator load Beam ('b0') for pickup", '', '', '', 'b0'], 'open': 1, 'tags': ''}
    # {'text': 'Movement 0', 'image': '', 'values': ['A0_M0', 'OperatorLoadBeamMovement', "Load Beam ('b0') for pickup at Frame(Point(22875.898, 5076.094, 620.825), Vector(0.000, 1.000, 0.000), Vector(-1.000, 0.000, 0.000)) (Side 3 face up).", '', '', '', 'b0'], 'open': 0, 'tags': ''}
    return row['values'][column_id]


def treeview_select_next_movement(guiref):
    # type: (dict) -> str
    """Select the next available movement"""

    tree = guiref['process']['tree']  # type: ttk.Treeview
    item_ids = guiref['process']['item_ids']
    index = item_ids.index(treeview_get_selected_id(guiref))
    if index < len(item_ids) - 1:
        new_selection = item_ids[index + 1]  # type: str
        tree.selection_set([new_selection])
        # Recursively call itself if the selected item is not a movement
        if not new_selection.startswith('m'):
            logger_ui.info("Skip selecting this movement: %s" % new_selection)
            treeview_select_next_movement(guiref)
        else:
            logger_ui.info("Next movement selected: %s" % new_selection)
            tree.see(new_selection)
    else:
        logger_ui.info("Cannot select next movement because end reached.")

    # Will return the selected move_id
    return treeview_get_selected_id(guiref)

#####################################
# Helper functions for buttons
#####################################


def enable_run_buttons(guiref):
    # Enable Run buttons
    guiref['exe']['run_button'].config(state="normal")
    guiref['exe']['step_button'].config(state="normal")
    guiref['exe']['step_from_pt_button'].config(state="normal")
    guiref['exe']['stop_button'].config(state="normal")


def disable_run_buttons(guiref):
    # Enable Run buttons
    guiref['exe']['run_button'].config(state="disabled")
    guiref['exe']['step_button'].config(state="disabled")
    guiref['exe']['step_from_pt_button'].config(state="disabled")
    guiref['exe']['stop_button'].config(state="disabled")

#################
# POP UP Windows
#################


class SettingsPopupWindow(object):
    def __init__(self, master, path):
        existing_setting = open(path, "r")

        top = self.top = tk.Toplevel(master)
        self.l = tk.Label(top, text="Change the settings here")
        self.l.pack()
        self.e = tk.Text(top)
        self.e.pack(side=tk.LEFT, fill=tk.BOTH)
        self.e.insert(tk.END, existing_setting.read())
        self.b = tk.Button(top, text='Save', command=self.cleanup)
        self.b.pack()

    def cleanup(self):
        self.value = self.e.get('1.0', tk.END)
        self.top.destroy()




if __name__ == "__main__":
    # Root TK Object
    root = tk.Tk()
    root.title("Robot and Clamps Assembly Process Execution")
    root.geometry("1500x800")

    guiref = create_execution_gui(root, None)
    guiref['root'] = root

    # Start the TK GUI Thread
    tk.mainloop()
