import argparse
import datetime
import logging
import queue
import time
import tkinter as tk
from enum import Enum
from functools import partial
from threading import Thread
from tkinter import ttk
from types import SimpleNamespace

from clamp_controller.ClampModel import ClampModel
from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall

from process_controller.background_command import *
from process_controller.execute_movement import *
from process_controller.GUI import *
from process_controller.ProcessModel import (RobotClampExecutionModel, RunStatus)


def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.
# Described in https://maldus512.medium.com/how-to-setup-correctly-an-application-with-python-and-tkinter-107c6bc5a45

# Model is a single RobotClampExecutionModel() object
# View is the TKInter UI that is created by process_controller.GUI.create_execution_gui()
# Controller for the UI is the tk.mainloop().
# Controller for background task is the background_thread() that runs on a separate thread.
# - The background thread implemented a time-share based multi-task execution.
# - It is separated to handle_ui_commands() and update_status()


# Initialize logger for the controller

logger_bg = logging.getLogger("app.bg")
logger_run = logging.getLogger("app.run")
logger_ctr = logging.getLogger("app.ctr")
logger_ros = logging.getLogger("app.ros")
logger_sync = logging.getLogger("app.sync")
# Initialize multi-tasking variables.
last_status_update_time = 0


def background_thread(guiref, model: RobotClampExecutionModel, q):
    """ Background thread to handel UI commands and other tasks.
    Tasks higher up in the list have higher priority.

    Task must return True if it was executed and return False if it was not executed."""
    logger_bg.info("Background Thread Started")
    while True:
        if execute_background_commands(guiref, model, q):
            continue
    logger_bg.info("Background Thread Stopped")


def execute_background_commands(guiref, model: RobotClampExecutionModel, q):
    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):

            # Handelling MODEL_LOAD_PROCESS upon closure of dialog
            if msg.type == ProcessControllerBackgroundCommand.MODEL_LOAD_PROCESS:
                logger_bg.info(
                    "Relaying BackgroundCommand: MODEL_LOAD_PROCESS")
                guiref['root'].wm_state('iconic')
                disable_run_buttons(guiref)
                model.load_process(msg.json_path)
                if model is not None:
                    guiref['process']['process_status'].set(
                        model.process_description)
                init_actions_tree_view(guiref, model)
                guiref['process']['goto_beam_combobox']['values'] = tuple(model.process.assembly.sequence)
                enable_run_buttons(guiref)
                guiref['root'].wm_state('zoomed')

            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_OPEN_SETTING):
                # model.open_setting_file()
                path = model.settings_file_path_default()
                master = guiref['root']
                popup = SettingsPopupWindow(master, path)
                master.wait_window(popup.top)
                if hasattr(popup, 'value'):
                    with open(path, 'w') as f:
                        f.write(popup.value)
                    model.load_settings()
                    logger_bg.info("Settings Saved")

            # Handle UI_TREEVIEW_GOTO_BEAM
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_TREEVIEW_GOTO_BEAM, check_loaded_process=True, check_status_is_stopped=True):
                tree = guiref['process']['tree']
                # Screw tree to the bottom and scrow back up ensures the item of interest
                # is on the first row in the treeview
                if msg.beam_id in guiref['process']['item_ids']:
                    tree.see(guiref['process']['item_ids'][-1])
                    tree.see(msg.beam_id)
                    logger_bg.info("Scrolling Treeview to see %s" % msg.beam_id)
                else:
                    logger_bg.warning("Cannot find Treeview item: %s" % msg.beam_id)

            # Handle UI_UPDATE_STATUS
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_UPDATE_STATUS):
                ui_update_run_status(guiref, model)

            # Handelling UI_RUN
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_RUN,
                            check_loaded_process=True, check_robot_connection=True):
                # Change Status
                model.run_status = RunStatus.RUNNING
                model.alternative_start_point = 0
                # Start a new thread if the current one is not active
                if model.run_thread is None or not model.run_thread.isAlive():
                    model.run_thread = Thread(target=program_run_thread, args=(guiref, model, q), daemon=True)
                    model.run_thread.name = "program_run_thread"
                    model.run_thread.start()
                ui_update_run_status(guiref, model)
                # Dont do anyhting if program is already running

            # Handelling UI_STEP
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_STEP,
                            check_loaded_process=True, check_robot_connection=True):
                # Change Status
                model.run_status = RunStatus.STEPPING_FORWARD
                model.alternative_start_point = 0
                # Start a new thread if the current one is not active
                if model.run_thread is None or not model.run_thread.isAlive():
                    model.run_thread = Thread(target=program_run_thread, args=(guiref, model, q), daemon=True)
                    model.run_thread.name = "program_run_thread"
                    model.run_thread.start()
                ui_update_run_status(guiref, model)

            # Handelling UI_STEP_FROM_POINT
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_STEP_FROM_POINT,
                            check_loaded_process=True, check_robot_connection=True, check_status_is_stopped=True, check_selected_is_movement=True):

                # Retrive movement and check if state is available
                tree_row_id = treeview_get_selected_id(guiref)
                movement = model.movements[tree_row_id]
                if not hasattr(movement, 'trajectory'):
                    return True
                max_start_number = len(movement.trajectory.points) - 1

                suggest_starting_step = None
                if hasattr(movement, "last_completed_point"):
                    suggest_starting_step = min(movement.last_completed_point + 1 , len(movement.trajectory.points) - 1)
                popup = AlternativeStartPointWindow(guiref['root'], max_start_number, suggest_starting_step)
                guiref['root'].wait_window(popup.top)
                n = popup.value
                if n is None:
                    return True
                if n > max_start_number:
                    logger_bg.warning("Input number larger than number of trajectory points")
                    return True

                # Change Status
                model.run_status = RunStatus.STEPPING_FORWARD_FROM_PT
                model.alternative_start_point = n
                # Start a new thread if the current one is not active
                if model.run_thread is None or not model.run_thread.isAlive():
                    model.run_thread = Thread(
                        target=program_run_thread, args=(guiref, model, q), daemon=True)
                    model.run_thread.name = "program_run_thread"
                    model.run_thread.start()
                ui_update_run_status(guiref, model)

            # Handelling UI_STOP
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_STOP):
                if model.run_status != RunStatus.STOPPED:
                    model.run_status = RunStatus.STOPPED
                else:
                    # Second Press will kill the run thread
                    if model.run_thread is not None:
                        try:
                            model.run_thread._stop()
                            model.run_thread = None
                        except:
                            pass
                    # Just to be safe. Issue Stop Clamps if they are connected
                    if model.ros_clamps is not None:
                        model.ros_clamps.send(ROS_STOP_ALL_COMMAND())
                ui_update_run_status(guiref, model)

            # Handelling UI_CONFIRM
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_CONFIRM):
                model.operator_confirm = True
                ui_update_run_status(guiref, model)

            # Handelling UI_ROBOT_CONNECT
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_ROBOT_CONNECT):
                # Connect to new ROS host
                guiref['ros']['robot_status'].set("Connecting to Robot Host")
                if model.connect_ros_robots(msg.ip, q):
                    guiref['ros']['robot_status'].set(
                        "Connected to Robot Host")
                    logger_ctr.info("Robot Host Connected")
                else:
                    guiref['ros']['robot_status'].set("Not Connected")
                    logger_ctr.info("Robot Host connection not successful")

            # Handelling UI_CLAMP_CONNECT
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_CLAMP_CONNECT):
                # Connect to new ROS host
                guiref['ros']['clamp_status'].set("Connecting to Clamp Hose")
                if model.connect_ros_clamps(msg.ip, partial(ros_clamps_callback, guiref, model, q)):
                    guiref['ros']['clamp_status'].set(
                        "Connected to Clamp Host")
                    logger_ctr.info("Clamp Host Connected")
                else:
                    guiref['ros']['clamp_status'].set("Not Connected")
                    logger_ctr.info("Clamp Host connection not successful")

            # Handelling PRINT_ACTION_SUMMARY
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.PRINT_ACTION_SUMMARY):
                beam_id = treeview_get_selected_item_beam_id(guiref)
                if beam_id is not None:
                    model.process.get_movement_summary_by_beam_id(beam_id)

            # Handelling UI_LOAD_EXT_MOVEMENT
            # if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_LOAD_EXT_MOVEMENT, check_loaded_process=True):
            #     movements_modified = model.process.load_external_movements(os.path.dirname(model.process_path))
            #     for movement in movements_modified:
            #         update_treeview_row(guiref, model, movement)

            # Handelling UI_GOTO_START_STATE
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_GOTO_START_STATE,
                            check_loaded_process=True, check_robot_connection=True, check_status_is_stopped=True, check_selected_is_movement=True):
                # Retrive movement and check if state is available
                tree_row_id = treeview_get_selected_id(guiref)
                movement = model.movements[tree_row_id]
                if not model.process.movement_has_start_robot_config(movement):
                    logger_bg.warning(
                        "Selected item does not have a START robot config")
                    return False

                # Construct and send rrc command
                config = model.process.get_movement_start_robot_config(movement)
                message = "Jogging to START State of %s " % movement.movement_id
                jog_thread = Thread(target=execute_jog_robot_to_config, args=(guiref, model, config, message, q), daemon=True)
                jog_thread.name = "Jogging Thread"
                jog_thread.start()

            # Handelling UI_GOTO_END_STATE
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_GOTO_END_STATE,
                            check_loaded_process=True, check_robot_connection=True, check_status_is_stopped=True, check_selected_is_movement=True):
                # Retrive movement and check if state is available
                tree_row_id = treeview_get_selected_id(guiref)
                movement = model.movements[tree_row_id]
                if not model.process.movement_has_end_robot_config(movement):
                    logger_bg.warning(
                        "Selected item does not have an END robot config")
                    return False

                # Construct and send rrc command
                config = model.process.get_movement_end_robot_config(movement)
                message = "Jogging to End State of %s " % movement.movement_id
                jog_thread = Thread(target=execute_jog_robot_to_config, args=(guiref, model, config, message, q), daemon=True)
                jog_thread.name = "Jogging Thread"
                jog_thread.start()

            # Handelling UI_SHAKE_GANTRY_POPUP
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_SHAKE_GANTRY_POPUP):
                dialog = ShakeGantryPopup(guiref, model, q)

            # Handelling UI_SHAKE_GANTRY
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_SHAKE_GANTRY, check_robot_connection=True, check_status_is_stopped=True):
                shake_amount = msg.shake_amount
                shake_speed = msg.shake_speed
                shake_repeat = msg.shake_repeat
                jog_thread = Thread(target=execute_ui_shake_gantry, args=(guiref, model, shake_amount, shake_speed, shake_repeat, q), daemon=True)
                jog_thread.name = "Jogging Thread"
                jog_thread.start()

            # Handelling UI_COMPARE_JOINT_VALUES
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_COMPARE_JOINT_VALUES, check_robot_connection=True, check_status_is_stopped=True, check_selected_is_movement=True):
                tree_row_id = treeview_get_selected_id(guiref)
                movement = model.movements[tree_row_id]
                if not model.process.movement_has_end_robot_config(movement):
                    logger_bg.warning(
                        "Selected item does not have an END robot config")
                    return False

                jog_thread = Thread(target=execute_compare_joint_values, args=(guiref, model, movement), daemon=True)
                jog_thread.name = "Joint Value Compare Thread"
                jog_thread.start()

            # Handelling UI_TOOLCHANGER_PROBE
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_TOOLCHANGER_PROBE, check_robot_connection=True, check_status_is_stopped=True):
                jog_thread = Thread(target=execute_ui_toolchanger_probe, args=(guiref, model, q), daemon=True)
                jog_thread.name = "Toolchanger Probe Thread"
                jog_thread.start()

            # Handelling Restart Camera
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_RESTART_CAMERA, check_robot_connection=True):
                model.ros_robot.send(rrc.SystemSetDigital("doUnitR11Out1",0))
                time.sleep(2)
                model.ros_robot.send(rrc.SystemSetDigital("doUnitR11Out1",1))


            # Handelling UI_SOFTMODE_ENABLE
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_SOFTMODE_ENABLE, check_robot_connection=True, check_status_is_stopped=True):
                jog_thread = Thread(target=robot_softmove_blocking_thread, args=(model, True, get_soft_direction(guiref), get_stiffness_soft_dir(guiref), get_stiffness_nonsoft_dir(guiref)), daemon=True)
                jog_thread.name = "Jog SoftMove Thread"
                jog_thread.start()

            # Handelling UI_SOFTMODE_DISABLE
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_SOFTMODE_DISABLE, check_robot_connection=True, check_status_is_stopped=True):
                jog_thread = Thread(target=robot_softmove_blocking_thread, args=(model, False), daemon=True)
                jog_thread.name = "Jog SoftMove Thread"
                jog_thread.start()

            # Handelling UI_COMPUTE_VISUAL_CORRECTION
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.UI_COMPUTE_VISUAL_CORRECTION, check_selected_is_movement=True):
                tree_row_id = treeview_get_selected_id(guiref)
                movement = model.movements[tree_row_id]  # type: RoboticMovement
                compute_visual_correction(guiref, model, movement)

            # Handelling TEST
            if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.TEST):
                tree_row_id = treeview_get_selected_id(guiref)
                if msg.id == 0:
                    movement = model.movements[tree_row_id]
                    test0(msg, guiref, model, movement)
                elif msg.id == 1:
                    if bg_cmd_check(msg, guiref, model, ProcessControllerBackgroundCommand.TEST, check_selected_is_movement=True):
                        movement = model.movements[tree_row_id]
                        show_movement_json_popup(guiref, model, movement)
                else:
                    logger_ctr.error("Unhandled Test Msg, id = %s" % msg.id)

            # Returns True if a command is consumed

            return True
    except queue.Empty:
        return False


def robot_softmove_blocking_thread(model: RobotClampExecutionModel, enable: bool, soft_direction="Z", stiffness=99, stiffness_non_soft_dir=100):
    model.run_status = RunStatus.JOGGING
    robot_softmove_blocking(model, enable, soft_direction, stiffness, stiffness_non_soft_dir)
    model.run_status = RunStatus.STOPPED


def bg_cmd_check(msg, guiref, model: RobotClampExecutionModel,
                 target_bg_command: ProcessControllerBackgroundCommand,
                 check_loaded_process=False,
                 check_robot_connection=False,
                 check_status_is_stopped=False,
                 check_selected_is_movement=False):
    if msg.type != target_bg_command:
        return False
    if check_loaded_process:
        if model.process is None:
            logger_bg.warning("Load Process first.")
            return False
    if check_robot_connection:
        if (model.ros_robot is None) or (not model.ros_robot.ros.is_connected):
            logger_bg.warning("Connect ROS Robot first.")
            return False
    if check_status_is_stopped:
        if model.run_status != RunStatus.STOPPED:
            logger_bg.warning(
                "Run Status is not stopped: %s. Stop it first." % model.run_status)
            return False
    if check_selected_is_movement:
        tree_row_id = treeview_get_selected_id(guiref)
        if not tree_row_id.startswith('m'):
            logger_bg.warning(
                "Selected item is not a movement: %s. Cannot proceed." % tree_row_id)
            return False
    # Return True if every check is passed
    logger_bg.info("Processing BG Command: %s" % target_bg_command)
    return True


def robot_goto_end_frame(guiref, model: RobotClampExecutionModel, q):
    # Run the Selected Item. If it is not a Movement, do nothing
    move_id = treeview_get_selected_id(guiref)
    if not move_id.startswith('m'):
        logger_run.info("Selected item is not a movement")
        return False

    movement = model.movements[move_id]  # type: Movement

    if not hasattr(movement, 'target_frame'):
        logger_run.info("Selected movement does not have end frame.")
        return False

    robot_goto_frame(model, movement.target_frame, 300)
    return True


def wait_for_opeartor_confirm(guiref, model: RobotClampExecutionModel, message: str = "Confirm"):
    """This is a blocking call that enables the confirm button witgh a message.
    Returns True if Operator presses the button.
    Returns False if model.run_status changes to STOPPED indicating a stop.
    """
    button = guiref['exe']['confirm_button']
    guiref['exe']['confirm_button_text'].set(message)
    button.config(state="normal", bg='orange')
    model.operator_confirm = False

    # print message to TP if available
    if model.ros_robot is not None:
        model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["CONFIRM : %s ..." % (message[:50])], []))

    guiref['exe']['exe_status'].set("Paused")
    while (True):
        if model.operator_confirm:
            button.config(state="disabled", bg='grey')
            ui_update_run_status(guiref, model)
            return True
        if model.run_status == RunStatus.STOPPED:
            button.config(state="disabled", bg='grey')
            ui_update_run_status(guiref, model)
            return False


def program_run_thread(guiref, model: RobotClampExecutionModel, q):
    """ Thread for running programms. This thread exist when Run or Step is pressed.
    This thread will run at least one movement, and be terminated if:
    - model.run_status

    Care should be taken not to have opened resources that cannot be released
    as this thread could be terminated abrubtly.
    """
    logger_run.info("Program Run Thread Started")
    while True:

        # Run the Selected Item. If it is not a Movement, select the next movement.
        move_id = treeview_get_selected_id(guiref)
        if not move_id.startswith('m'):
            move_id = treeview_select_next_movement(guiref)

        movement = model.movements[move_id]  # type: Movement

        # Pause before
        if movement.operator_stop_before is not None:
            confirm = wait_for_opeartor_confirm(
                guiref, model, movement.operator_stop_before)
            if not confirm:
                logger_run.info(
                    "Operator stop not confirmed before movement. Run Thread Ended.")
                q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
                break

        # Execution
        success = execute_movement(guiref, model, movement)
        ui_update_run_status(guiref, model)
        # Pause after
        if movement.operator_stop_after is not None:
            confirm = wait_for_opeartor_confirm(
                guiref, model, movement.operator_stop_after)
            if not confirm:
                logger_run.info(
                    "Operator stop not confirmed after movement. Run Thread Ended.")
                q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
                break

        # Terminate if execution is not success, do not increment pointer.
        if not success:
            model.run_status = RunStatus.ERROR
            logger_run.info(
                "Execution Error. Program Stopped. Run Thread Ended.")
            q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
            break

        # Move program pointer to next movement
        treeview_select_next_movement(guiref)

        # Stop execution if it is currently in STEPPING mode.
        if model.run_status == RunStatus.STEPPING_FORWARD or model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
            model.run_status = RunStatus.STOPPED

        # Status note that the RUN Thread is Stopped
        if model.run_status == RunStatus.STOPPED:
            logger_run.info("Program Stopped. Run Thread Ended.")
            q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
            break


def initialize_logging(filename: str, log_file_level=logging.DEBUG, console_level=logging.DEBUG):
    # Logging Setup
    logger = logging.getLogger("app")
    logger.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # create file handler which logs even debug messages
    log_file_handler = logging.FileHandler(filename)
    log_file_handler.setLevel(log_file_level)
    log_file_handler.setFormatter(formatter)
    # create console handler with a higher log level
    log_console_handler = logging.StreamHandler()
    log_console_handler.setLevel(console_level)
    log_console_handler.setFormatter(formatter)
    # add the handlers to logger
    logger.addHandler(log_file_handler)
    logger.addHandler(log_console_handler)
    logger.info("App Started")


def ros_clamps_callback(guiref=None, model: RobotClampExecutionModel = None, q=None, message=None):
    # ROS command comes from a separate thread.
    # To maintain single threaded access to the Radio / Clamp,
    # we convert the ROS Command to a BackgroundCommand and place it in background command queue
    if message is None:
        return

    ui_update_run_status(guiref, model)

    # ! At the moment there is no command coming back from the ClampController that needs to be placed
    # ! in the queue. The ros_clamp object (RemoteClampFunctionCall) can already update its own clamps status


def ui_update_run_status(guiref, model: RobotClampExecutionModel):
    """Update the labels and indications related to execution"""
    if model.run_status == RunStatus.STOPPED:
        guiref['exe']['exe_status'].set("Stopped")
        guiref['exe']['exe_status_label'].config(bg="gray")
    if model.run_status == RunStatus.STEPPING_FORWARD:
        guiref['exe']['exe_status'].set("Stepping")
        guiref['exe']['exe_status_label'].config(bg="green")
    if model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
        guiref['exe']['exe_status'].set("Stepping Fm Pt")
        guiref['exe']['exe_status_label'].config(bg="green")
    if model.run_status == RunStatus.RUNNING:
        guiref['exe']['exe_status'].set("Running")
        guiref['exe']['exe_status_label'].config(bg="green")
    if model.run_status == RunStatus.ERROR:
        guiref['exe']['exe_status'].set("Error Stopped")
        guiref['exe']['exe_status_label'].config(bg="red")
    if model.run_status == RunStatus.JOGGING:
        guiref['exe']['exe_status'].set("Jogging")
        guiref['exe']['exe_status_label'].config(bg="green")

    # Clamps Status
    if model.ros_clamps is not None:
        if model.ros_clamps.last_sent_message is not None:
            command = model.ros_clamps.last_sent_message.command
            guiref['exe']['clamps_last_cmd_success'].set(command.__class__.__name__)
            guiref['exe']['clamps_last_cmd_success_label'].config(bg="gray")

            if command.status == ROS_COMMAND.RUNNING:
                guiref['exe']['clamps_running'].set("Running")
                guiref['exe']['clamps_running_label'].config(bg="green")
            if command.status == ROS_COMMAND.NOT_STARTED:
                guiref['exe']['clamps_running'].set("NotStarted")
                guiref['exe']['clamps_running_label'].config(bg="orange")
            if command.status == ROS_COMMAND.SUCCEED:
                guiref['exe']['clamps_running'].set("Succeed")
                guiref['exe']['clamps_running_label'].config(bg="gray")
            if command.status == ROS_COMMAND.FAILED:
                guiref['exe']['clamps_running'].set("Failed")
                guiref['exe']['clamps_running_label'].config(bg="red")


def test0(msg, guiref, model, movement):
    # Open a dialog and ask user to key in three offset values
    dialog = VisualOffsetPopup(guiref, model, movement)
    guiref['root'].wait_window(dialog.window)
    print("dialog.accpet = %s" % dialog.accpet)


def show_movement_json_popup(guiref, model, movement):
    # Open a dialog and ask user to key in three offset values
    dialog = MovementJsonPopup(guiref, model, movement)
    guiref['root'].wait_window(dialog.window)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='CLI RobotClampExecution.')
    parser.add_argument('-f', default='',  help='Load Process File on Start')
    parser.add_argument('-robotip', default='192.168.0.120',
                        help='IP for Robot ROS Host')
    parser.add_argument('-clampip', default='192.168.0.120',
                        help='IP for Clamp ROS Host')
    args = parser.parse_args()
    print(args)
    # Initialize Logger
    initialize_logging("ProcessExeController." +
                       datetime.date.today().strftime("%Y-%m-%d") + ".debug.log",
                       console_level=logging.INFO,
                       log_file_level=logging.DEBUG)

    # Root TK Object
    root = tk.Tk()
    root.title("Robot and Clamps Assembly Process Execution")
    root.geometry("1500x600")

    # Command queue
    q = queue.Queue()
    # Create Model
    model = RobotClampExecutionModel()
    # Get GUI Reference
    guiref = create_execution_gui(root, q)
    guiref['root'] = root
    # Start the background thread that processes UI commands
    t1 = Thread(target=background_thread, args=(guiref, model, q))
    t1.daemon = True
    t1.start()

    # Override default ip
    guiref['ros']['robot_ip_entry'].set(args.robotip)  # VM Address
    guiref['ros']['clamp_ip_entry'].set(args.clampip)  # VM Address
    guiref['exe']['softmove_enable'].set(1)

    if not args.f == "":
        logger_ctr.info(
            "Command Line contain -f. Load Json File at %s." % args.f)
        q.put(SimpleNamespace(
            type=ProcessControllerBackgroundCommand.MODEL_LOAD_PROCESS, json_path=args.f))

    # Start the TK GUI Thread
    tk.mainloop()


# Development command line start with sample file opened:
# python C:\Users\leungp\Documents\GitHub\clamp_controller\src\robot_clamp_controller\run.py -f C:\Users\leungp\Documents\GitHub\integral_timber_joints\external\itj_design_study\210128_RemodelFredPavilion\twelve_pieces_process.json -robotip 192.168.20.128
