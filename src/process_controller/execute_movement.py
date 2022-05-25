import datetime
import logging
import time
from types import SimpleNamespace

import compas_rrc as rrc
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.point import Point
from compas.geometry.primitives.vector import Vector
from compas_fab.robots import to_degrees
from compas_fab.robots.configuration import Configuration
from compas_fab.backends.ros.messages import ROSmsg
from integral_timber_joints.process.action import *

from process_controller.background_command import *
from process_controller.ProcessModel import *
from process_controller.GUI import *
from process_controller.rrc_instructions import *
from process_controller.execute_helper import *
from process_controller.execute_popup import *

from clamp_controller.ScrewdriverModel import g_status_dict
from clamp_controller.CommanderGUI import *
from clamp_controller.RosCommand import *

logger_exe = logging.getLogger("app.exe")

"""
This file contains the procedural routines that is used to execute a movement.
Different Movement class are handeled by different `execute_` functions.
Connections to the various drivers are located in `RobotClampExecutionModel`.
- Calls are made to the Robot Controller via the `AbbClient` from the compas_rrc library. (`model.ros_robot`)
- Calls to the Remote Tools Controller is via the `RemoteClampFunctionCall` (`model.ros_clamps`)
- (Both objects are connected via ROS, however other connection objects can also be added if needed)

Many of the `execute_` functions are stoppable from the UI. This is used for any motion that may take a long
time to accomplish or may fail in an unexpected way. Therefor all functions that implements a waiting loop should
repeatedly check if `model.run_status == RunStatus.STOPPED`.

The `execute_` functions should return True if the procedure is executed successfully, and false if otherwise.
In case of execution failure, the `execute_` function is responsible for putting the system in a safe (stopped) state
before returning.

"""
# Tool data settings in Robot Studio:
# TASK PERS tooldata t_A067_Tool:=[TRUE,[[0,0,0],[1,0,0,0]],[2.12,[0,0,45],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG1000:=[TRUE,[[0,0,0],[1,0,0,0]],[10.72,[0,0,105],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG500:=[TRUE,[[0,0,0],[1,0,0,0]],[9.04,[0,0,110],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_CL3:=[TRUE,[[0,0,0],[1,0,0,0]],[7.99,[-30,4,73],[1,0,0,0],0,0,0]];
INTERMEDIATE_ZONE = rrc.Zone.Z1
FINAL_ZONE = rrc.Zone.FINE

# TIMEOUT values in seconds
CLAMP_START_RUNNING_TIMEOUT = 0.7
CLAMP_CONTROLLER_ALIVE_TIMEOUT = 3.5
CLAMP_COMMAND_ACK_TIMEOUT = 0.7


def current_milli_time(): return int(round(time.time() * 1000))


def execute_movement(guiref, model: RobotClampExecutionModel, movement: Movement):
    """Executes a Movement.
    This function is a switch board deligating the actual implementation to one of the
    `execute_` functions below.
    This function will return True if the movement is executed to completion without problem.

    This funcion will block until the movement is completed or if it is canceled from the UI.
    (`model.run_status == RunStatus.STOPPED.`)
    """

    logger_exe.info("Executing %s (%s): %s" % (movement, movement.movement_id, movement.tag))

    guiref['exe']['last_executed_movement'].set(movement.movement_id)

    model.ros_robot.send(rrc.CustomInstruction(
        'r_A067_TPPlot', ["Executing Movement (%s)" % (movement.movement_id)], []))
    model.ros_robot.send(rrc.CustomInstruction(
        'r_A067_TPPlot', ["- %s" % (movement.tag[:40]) + '...'], []))

    if isinstance(movement, OperatorLoadBeamMovement):
        print("Operator should load beam %s. Grasp face is %s." %
              (movement.beam_id, movement.grasp_face))
        return True

    elif isinstance(movement, RoboticClampSyncLinearMovement):
        return execute_robotic_clamp_sync_linear_movement(guiref, model, movement)

    elif isinstance(movement, RobotScrewdriverSyncLinearMovement):
        return execute_robotic_clamp_sync_linear_movement(guiref, model, movement)

    elif isinstance(movement, RoboticFreeMovement):
        return execute_robotic_free_movement(guiref, model, movement)

    elif isinstance(movement, RoboticLinearMovement):
        return execute_robotic_linear_movement(guiref, model, movement)

    elif isinstance(movement, ClampsJawMovement):
        return execute_clamp_jaw_movement(guiref, model, movement)

    elif isinstance(movement, ScrewdriverMovement):
        return execute_screwdriver_movement(guiref, model, movement)

    elif isinstance(movement, RoboticDigitalOutput):
        tool_id = movement.tool_id  # type: str
        tool_type = type(model.process.tool(tool_id))
        digital_output = movement.digital_output
        if (digital_output == DigitalOutput.OpenGripper or digital_output == DigitalOutput.CloseGripper) and tool_type == Screwdriver:
            return execute_robotic_digital_output_screwdriver(guiref, model, movement)
        else:
            return execute_robotic_digital_output(guiref, model, movement)

    elif isinstance(movement, OperatorAttachToolMovement):
        return execute_operator_attach_tool_movement(guiref, model, movement)

    elif isinstance(movement, OperatorAddJogOffset):
        return execute_operator_add_jog_offset_movement(guiref, model, movement)

    elif isinstance(movement, AcquireDockingOffset):
        return execute_acquire_docking_offset(guiref, model, movement)

    elif isinstance(movement, OperatorAddVisualOffset):
        return execute_operator_add_visual_offset_movement(guiref, model, movement)

    elif isinstance(movement, RemoveOperatorOffset):
        return execute_remove_operator_offset_movement(guiref, model, movement)

    elif isinstance(movement, CancelRobotOffset):
        return execute_remove_operator_offset_movement(guiref, model, movement)

    elif isinstance(movement, SetWorkpieceWeight):
        return execute_set_workpiece_weight(guiref, model, movement)

    # Catch all during Development
    else:
        execute_some_delay(model, movement)
        logger_exe.warning("execution code for %s - %s does not exist." % (movement.__class__.__name__, movement.tag))
        return False

#####################################################
# Sub functions that handels different Movement Types
#####################################################


def grip_load_instruction_from_beam(model: RobotClampExecutionModel, beam_id: str):
    beam = model.process.assembly.beam(beam_id)

    # rough eastimate of beam weight
    beam_volume = beam.width * beam.length * beam.height  # mm
    beam_density = 460  # kg/m^3
    beam_weight = beam_density * beam_volume * 1e-9

    # Estimate of beam cog
    gripper_grasp_dist_from_start = model.process.assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
    beam_grasp_offset = beam.length / 2 - gripper_grasp_dist_from_start

    # The mass (weight) of the load in kg.
    # Must be bigger then 0
    mass = beam_weight
    # The center of gravity of the payload expressed in mm in the tool coordinate system.
    # Minimum 1 value bigger then 0
    cog_x = beam_grasp_offset
    cog_y = 0
    cog_z = 241.6
    # The orientation of the axes of moment.
    # These are the principal axes of the payload moment of inertia with origin in center of gravity.
    # Expressed in quaternians
    aom_q1 = 1
    aom_q2 = 0
    aom_q3 = 0
    aom_q4 = 0
    # The moment of inertia of the load around the axis of moment expressed in kgm2.
    # Correct definition of the moments of inertia will allow optimal utilization of the path planner and axes control.
    # This may be of special importance when handling large sheets of metal, and so on.
    # All moments of inertia ix, iy, and iz equal to 0 kgm2 imply a point mass.
    # Normally, the moments of inertia must only be defined when the distance from the mounting flange to the center of gravity
    # is less than the maximal dimension of the load.
    inertia_x = 0
    inertia_y = 0
    inertia_z = 0
    logger_exe.info("Grip Load generated for beam %s, beam_weight = %skg, beam_grasp_offset = %smm" % (beam_id, beam_weight, beam_grasp_offset))
    return rrc.CustomInstruction('r_A067_GripLoad', [], [mass, cog_x, cog_y, cog_z, aom_q1, aom_q2, aom_q3, aom_q4, inertia_x, inertia_y, inertia_z], feedback_level=rrc.FeedbackLevel.DONE)


def execute_robotic_digital_output(guiref, model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
    """Performs RoboticDigitalOutput Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """
    future_results = []  # type: List[rrc.FutureResult]
    # Open Gripper Valve
    if movement.digital_output == DigitalOutput.OpenGripper:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 0, feedback_level=1)))

    # Close Gripper Valve
    if movement.digital_output == DigitalOutput.CloseGripper:
        # Digital Output
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 1, feedback_level=1)))

    # Lock Tool Valve
    if movement.digital_output == DigitalOutput.LockTool:
        # Set tool data
        tool_data_name = 't_A067_Tool_' + model.process.tool(movement.tool_id).type_name
        logger_exe.info("Locking to tool %s (new tooldata = %s)" % (movement.tool_id, tool_data_name))

        future_results.append(model.ros_robot.send(
            rrc.SetTool(tool_data_name, feedback_level=rrc.FeedbackLevel.DONE)))
        # Digital Out
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA1', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB1', 0, feedback_level=1)))

    # Unlock Tool Valve
    if movement.digital_output == DigitalOutput.UnlockTool:
        # Disconnect air supply to gripper feed through first
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 0, feedback_level=1)))
        # This is the valve for the tool changer
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA1', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB1', 1, feedback_level=1)))

        # Set grip data to `zero` and tool data to `no tool`
        future_results.append(model.ros_robot.send(
            rrc.CustomInstruction('r_A067_GripUnload', [], [], feedback_level=rrc.FeedbackLevel.DONE)))
        tool_data_name = 't_A067_Tool'
        future_results.append(model.ros_robot.send(
            rrc.SetTool(tool_data_name, feedback_level=rrc.FeedbackLevel.DONE)))
        logger_exe.info("Unlocking tool %s (new tooldata = %s)" % (movement.tool_id, tool_data_name))

    def wait_for_digital_signal(io_name, signal_value_to_wait_for):
        while (True):
            future = send_and_wait_unless_cancel(model, rrc.ReadDigital(io_name))
            # User pressed cancel
            if not future.done:
                return False
            # Check signal, if it is equal to expected value, we return
            if future.value == signal_value_to_wait_for:
                return True
            # If not, we wait a tiny bit, send another read signal and check again
            time.sleep(0.5)

    while (True):
        # * Check for IO completion
        if all([future.done for future in future_results]):
            if movement.digital_output == DigitalOutput.LockTool:
                # Assert ToolChanger Lock Check signal is HIGH (1)
                if not wait_for_digital_signal('diUnitR11In3', 1):
                    logger_exe.warning("UI stop button pressed before getting confirm signal.")
                    return False
                else:
                    logger_exe.info("Toolchanger sensor signal received. DigitalOutput.LockTool is successful")
                    guiref['exe']['toolchanger_signal'].set("Locked")
                    time.sleep(0.3)
                    return True

            if movement.digital_output == DigitalOutput.UnlockTool:
                # Assert ToolChanger Lock Check signal is HIGH (1)
                if not wait_for_digital_signal('diUnitR11In3', 0):
                    logger_exe.warning("UI stop button pressed before getting confirm signal.")
                    return False
                else:
                    logger_exe.info("Toolchanger sensor signal received. DigitalOutput.UnlockTool is successful")
                    guiref['exe']['toolchanger_signal'].set("Unlocked")
                    time.sleep(0.3)
                    return True

            else:
                # Add some delay after the action and assume it is done
                future = send_and_wait_unless_cancel(model, rrc.WaitTime(2))
                if not future.done:
                    logger_exe.warning("UI stop button pressed before WaitTime(2) is over.")
                    return False
                else:
                    return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning(
                "UI stop button pressed before RoboticDigitalOutput (%s) is completed." % movement)
            return False
        time.sleep(0.05)


def execute_robotic_digital_output_screwdriver(guiref, model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Screwdriver gripper movement cannot start because Clamp ROS is not connected")
        return False

    # * Send command to Clamp Controller, wait for ROS controller to ACK
    if movement.digital_output == DigitalOutput.OpenGripper:
        # * Open Gripper
        command = ROS_SCREWDRIVER_GRIPPER_COMMAND(movement.tool_id, False)
        logger_exe.info("Sending ROS_SCREWDRIVER_GRIPPER_COMMAND (Extend = False) Movement (%s)" % (movement.movement_id))
    elif movement.digital_output == DigitalOutput.CloseGripper:
        # * Close Gripper
        command = ROS_SCREWDRIVER_GRIPPER_COMMAND(movement.tool_id, True)
        logger_exe.info("Sending ROS_SCREWDRIVER_GRIPPER_COMMAND (Extend = True) Movement (%s)" % (movement.movement_id))
    else:
        logger_exe.error("Screwdriver Movement (%s) not supported for digital_output type: %s" % (movement.movement_id, movement.digital_output))
        return False

    message = model.ros_clamps.send(command)

    while (True):
        # * Sucess Condition - command succeed
        if command.status == ROS_COMMAND.SUCCEED:
            logger_exe.info("Screwdriver Gripper Movement (%s) completed." % movement.movement_id)
            return True

        # * Fail Condition - command failed
        if command.status == ROS_COMMAND.FAILED:
            logger_exe.info("Screwdriver Gripper Movement (%s) stopped or jammed." % movement.movement_id)
            return False

        # * Fail Condition - Clamp Command not ACKed
        if not message.acked and message.ms_since_send > CLAMP_COMMAND_ACK_TIMEOUT * 1000:
            logger_exe.warning("RoboticDigitalOutput (%s) stopped because ACK not received from clamp controller within %s sec." %
                               (movement.movement_id, CLAMP_COMMAND_ACK_TIMEOUT))
            model.ros_clamps.send(ROS_STOP_COMMAND([movement.tool_id]))
            model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Clamp Controller no ACK, cannot continue.']))
            return False

        # * Stop Condition - User pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("Screwdriver Gripper Movement (%s) stopped by user." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND([movement.tool_id]))
            return False

        time.sleep(0.02)


def execute_operator_attach_tool_movement(guiref, model: RobotClampExecutionModel, movement: OperatorAttachToolMovement):
    robotic_digital_output_movement = RoboticDigitalOutput(DigitalOutput.CloseGripper, movement.tool_id, tag=movement.tag)
    robotic_digital_output_movement.movement_id = movement.movement_id
    return execute_robotic_digital_output_screwdriver(guiref, model, robotic_digital_output_movement)


def execute_robotic_free_movement(guiref, model: RobotClampExecutionModel, movement: RoboticFreeMovement):
    """Performs RoboticFreeMovement Movement

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """

    STEPS_TO_BUFFER = 5  # Number of steps to allow in the robot buffer

    if movement.trajectory is None:
        logger_exe.warning("Attempt to execute movement with no trajectory")
        return False

    # Set Acceleration Settings
    if movement.__class__ == RoboticLinearMovement:
        acc, ramp = get_linear_move_acc_ramp(guiref)
    else:
        acc, ramp = get_free_move_acc_ramp(guiref)
    result = send_and_wait_unless_cancel(model, rrc.SetAcceleration(acc, ramp))
    if result.done == False:
        logger_exe.warning("execute_robotic_*_movement() stopped beacause user canceled while SetAcceleration().")
        return False

    # We store the future results of the movement commands in this list.
    # This allow us to monitor the results and keep a known number of buffer points
    futures = []
    position_readout_futures = []
    active_point = 0
    position_readout_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # Check softmove state and send softmove command if state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    if get_softness_enable(guiref):
        if not robot_softmove_blocking(model, enable=movement.softmove, soft_direction=get_soft_direction(guiref),
                                       stiffness=get_stiffness_soft_dir(guiref), stiffness_non_soft_dir=get_stiffness_nonsoft_dir(guiref)):
            logger_exe.warning("execute_robotic_*_movement() stopped beacause robot_softmove_blocking() failed.")
            return False

    # In case of a START_FROM_PT
    # The active point number is shifted
    if model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
        active_point = model.alternative_start_point
        position_readout_point = active_point

    guiref['exe']['last_completed_trajectory_point'].set(" - ")
    guiref['exe']['last_deviation'].set(" - ")
    for current_step, point in enumerate(movement.trajectory.points):
        # Skip points in the case of a halfway start
        if current_step < active_point:
            # insert a dummy future to fill the gap
            futures.append(None)
            position_readout_futures.append(None)
            continue

        # Format movement and send robot command
        logger_exe.info("Sending command %i of %i" % (current_step, total_steps))
        instruction = trajectory_point_to_instruction(model, movement, guiref, current_step)
        futures.append(model.ros_robot.send(instruction))

        # Send command to read position back
        # position_readout_futures.append(model.ros_robot.send(rrc.GetRobtarget()))

        # Lopping while active_point is just STEPS_TO_BUFFER before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - STEPS_TO_BUFFER:
                break
            # Advance pointer when future is done
            if futures[active_point].done:
                # Human readable progress counter
                completed_progress_index = active_point
                if model.run_status in [RunStatus.RUNNING, RunStatus.STEPPING_FORWARD_FROM_PT, RunStatus.STEPPING_FORWARD]:
                    completed_progress_index += model.alternative_start_point
                    movement.last_completed_point = completed_progress_index  # Kept for the Step-from-Point Dialogbox
                if model.run_status == RunStatus.STEPPING_BACKWARD_FROM_PT:
                    completed_progress_index = model.alternative_start_point - completed_progress_index + 1
                    movement.last_completed_point = completed_progress_index  # Kept for the Step-from-Point Dialogbox

                # Logging and advance active_point pointer
                logger_exe.info("Point %i is done. Delta time %f seconds." %
                                (active_point, (datetime.datetime.now() - last_time).total_seconds()))
                last_time = datetime.datetime.now()
                guiref['exe']['last_completed_trajectory_point'].set(str(active_point))
                active_point += 1

            # Read the RobTarget as it comes back, compute deviation
            # if position_readout_futures[position_readout_point].done:
            #     if movement.path_from_link is not None:
            #         target_frame = movement.path_from_link["robot11_tool0"][position_readout_point]  # type: Frame
            #         target_frame = frame_to_millimeters(target_frame)
            #         target_ext_axes = to_millimeters(movement.trajectory.points[position_readout_point].prismatic_values)
            #         actual_frame, actual_ext_axes = position_readout_futures[position_readout_point].value
            #         actual_ext_axes = actual_ext_axes.prismatic_values
            #         deviation_frame = target_frame.point.distance_to_point(actual_frame.point)
            #         deviation_ext_axes = Point(*target_ext_axes).distance_to_point(Point(*actual_ext_axes))
            #         logger_exe.info("Deviation for point %i: Frame Deviation = %.2fmm, ExtAxes Deviation = %.2fmm" %
            #                         (position_readout_point, deviation_frame, deviation_ext_axes))
            #         guiref['exe']['last_deviation'].set("%.2fmm" % (deviation_frame))
            #     position_readout_point += 1

            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warning("UI stop button pressed before Robotic Free Movement (%s) is completed." % (movement.movement_id))
                return False

    # Final deviation
    deviation = check_deviation(model, movement.target_frame)
    if deviation is not None:
        logger_exe.info("Movement (%s) target frame deviation %s mm" % (movement.movement_id, deviation))
        guiref['exe']['last_deviation'].set("%.2fmm" % (deviation))
    else:
        logger_exe.warning("execute_robotic_*_movement stopped before deviation result (future not arrived)")
        return False

    return True


def execute_robotic_linear_movement(guiref, model: RobotClampExecutionModel, movement: RoboticLinearMovement):

    return execute_robotic_free_movement(guiref, model, movement)


def clear_robot_movement_buffers(model: RobotClampExecutionModel):
    """This function clears any left-over or half-finished motion commands in the robot controller.
    User have to be running the RAPID program to complete this procedure.

    - First a 'do_A067_Interrupt` signal is set to 1 via webservice (almost immediate)
    - Using stnadard channel, retrive signal 'do_A067_InterruptDone' and wait.
        - This ensures the RAPID program ran and all buffered motion are consummed
    - Check if the 'do_A067_InterruptDone' is 1 and if so, reset 'do_A067_Interrupt` signal to 0

    """
    model.ros_robot.send(rrc.SystemSetDigital('do_A067_Interrupt', 1))
    while (True):
        result = model.ros_robot.send_and_wait(rrc.SystemGetDigital('do_A067_InterruptDone'))
        # logger_exe.info ("do_A067_InterruptDone = %s" % result)
        if result == 1.0:
            break
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("clear_robot_movement_buffers() stopped by user before do_A067_InterruptDone signal is high.")
            return False

    model.ros_robot.send(rrc.SetDigital('do_A067_Interrupt', 0))
    return True


def execute_robotic_clamp_sync_linear_movement(guiref, model: RobotClampExecutionModel, movement: RoboticClampSyncLinearMovement):
    """Performs RoboticClampSyncLinearMovement Movement

    Syncronization
    --------------
    1. User Press Play on TP
    2. Send movement command to Clamp Controller (wait for message ACK)
    3. Send movement to Robot Controller (no wait)

    Checks during Execution
    -----------------------
    - Stop Everything if the Clamp Controller did not send back first status update within timeout
    - Stop Everything if the Clamp Controller's second and subsequent status update age is larger than max_age
    - Stop Everything if the Clamp Controller reports a failed sync move.
    - Send new trajectory point to Robot Controller when buffering point is < STEPS_TO_BUFFER
    - Stop Everything if the Robot Controller Motion Task is Stopped


    Execution blocking behaviour
    ----------------------------
    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    Cancel
    ------
    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will give up, stop robot and clamps, and return False.
    """

    if movement.trajectory is None:
        logger_exe.warning("Attempt to execute movement with no trajectory")
        return False

    # * Set Acceleration Settings to 100% (minimize aceleration phase)
    result = send_and_wait_unless_cancel(model, rrc.SetAcceleration(100, 100))
    if result.done == False:
        logger_exe.warning("execute_robotic_clamp_sync_linear_movement() stopped beacause user canceled while SetAcceleration().")
        return False

    # * Prepare Movement command parameters
    tool_ids = None
    if type(movement) == RoboticClampSyncLinearMovement:
        tool_ids = movement.clamp_ids
        position = movement.jaw_positions[0]
    elif type(movement) == RobotScrewdriverSyncLinearMovement:
        tool_ids = movement.screwdriver_ids
        position = movement.screw_positions[0]

    velocity = model.settings[movement.speed_type]

    # * Check softmove state and send softmove command if state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    if get_softness_enable(guiref):
        if not robot_softmove_blocking(model, enable=movement.softmove, soft_direction=get_soft_direction(guiref),
                                       stiffness=get_stiffness_soft_dir(guiref), stiffness_non_soft_dir=get_stiffness_nonsoft_dir(guiref)):
            logger_exe.warning("execute_robotic_clamp_sync_linear_movement() stopped beacause robot_softmove_blocking() failed.")
            return False

    # * Clear (potentially any) Robot Movement Buffered in the Robot Controller (RAPID side)
    if not clear_robot_movement_buffers(model):
        logger_exe.warning("execute_robotic_clamp_sync_linear_movement() stopped beacause clear_robot_movement_buffers() failed.")
        return False

    # * Check to ensures Speed Ratio is 100 %
    if not ensure_speed_ratio(model, model.ros_robot, 100):
        logger_exe.warning("execute_robotic_clamp_sync_linear_movement() stopped beacause ensure_speed_ratio() failed.")
        return False

    def stop_clamps():
        """Function to stop all active clamps"""
        model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))

    def stop_robots():
        """Function to stop active robot"""
        model.ros_robot.send(rrc.SystemSetDigital('do_A067_Interrupt', 1))
        model.ros_robot.send(rrc.SetDigital('do_A067_Interrupt', 0))

    # * User Press PLAY to Sart the Movement
    result = send_and_wait_unless_cancel(model, rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move begins']))
    if result.done == False:
        logger_exe.warning("execute_robotic_clamp_sync_linear_movement() stopped beacause user canceled while waiting for TP Press Play.")
        return False

    # * Retrive screwdriver motion parameters
    clamps_pos_velo = [(tool_id, position, velocity) for tool_id in tool_ids]
    power_percentage = convert_movement_speed_type_to_power_poercentage(movement)
    allowable_target_deviation = movement.allowable_target_deviation

    # * Send movement to Clamp Controller, wait for ROS controller to ACK
    logger_exe.info("Sending ROS_VEL_GOTO_COMMAND to clamp for %s to %smm Started" % (tool_ids, position))

    clamp_command = ROS_VEL_GOTO_COMMAND(clamps_pos_velo, power_percentage, allowable_target_deviation)
    message = model.ros_clamps.send(clamp_command)

    while (True):
        # Sucess Condition - ACK Received
        if message.acked == True:
            logger_exe.info("send ACK received.")
            break
        # * Fail Condition - Clamp Command not ACKed in time
        if message.ms_since_send > CLAMP_COMMAND_ACK_TIMEOUT * 1000:
            logger_exe.warning("RoboticClampSync Movement (%s) stopped because ACK not received from clamp controller within %s sec." %
                               (movement.movement_id, CLAMP_COMMAND_ACK_TIMEOUT))
            stop_clamps()
            model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Clamp Controller no ACK, cannot continue.']))
            return False
        # * Fail Condition - UI Cancel
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("RoboticClampSync Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
            return False
        time.sleep(0.05)

    # * Deviation Calculation is not implemented yet (We will need get Joints and do FK here and cmopared with TrajPt)
    guiref['exe']['last_completed_trajectory_point'].set(" - ")
    guiref['exe']['last_deviation'].set(" - ")

    # In case of a START_FROM_PT
    # The active point number is shifted
    starting_traj_point = 0
    if model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
        starting_traj_point = model.alternative_start_point
    if model.run_status == RunStatus.STEPPING_BACKWARD_FROM_PT:
        starting_traj_point = model.alternative_start_point

    # * Send all trajectory points to the robot
    logger_exe.info("Sending %i MoveToJoints commands to Robot." % (len(movement.trajectory.points)))
    futures = []

    if model.run_status != RunStatus.STEPPING_BACKWARD_FROM_PT:
        for current_step in range(starting_traj_point, len(movement.trajectory.points)):
            instruction = trajectory_point_to_instruction(model, movement, guiref, current_step)
            futures.append(model.ros_robot.send(instruction))
    else:
        for current_step in range(starting_traj_point, -1, -1):  # Backwards
            instruction = trajectory_point_to_instruction(model, movement, guiref, current_step)
            futures.append(model.ros_robot.send(instruction))

    # * Monitoring robot and clamp the same time.
    active_traj_point_n = 0
    total_traj_points = len(movement.trajectory.points)

    def robot_movement_completed():
        return futures[-1].done

    while(True):
        # Monitor the finished robot points
        if active_traj_point_n < len(futures) and futures[active_traj_point_n].done:
            # Human readable progress counter
            completed_progress_index = active_traj_point_n + 1
            if model.run_status in [RunStatus.RUNNING, RunStatus.STEPPING_FORWARD_FROM_PT, RunStatus.STEPPING_FORWARD]:
                completed_progress_index += model.alternative_start_point
                movement.last_completed_point = completed_progress_index  # Kept for the Step-from-Point Dialogbox
            if model.run_status == RunStatus.STEPPING_BACKWARD_FROM_PT:
                completed_progress_index = model.alternative_start_point - completed_progress_index + 1
                movement.last_completed_point = completed_progress_index  # Kept for the Step-from-Point Dialogbox

            guiref['exe']['last_completed_trajectory_point'].set(" %i / %i " % (completed_progress_index, total_traj_points))
            logger_exe.info("Robot Traj Point %i / %i completed." % (completed_progress_index, total_traj_points))

            active_traj_point_n += 1

        # * Monitor if the Clamp have started running
        if clamp_command.status == ROS_COMMAND.NOT_STARTED:
            if current_milli_time() - message.send_time > CLAMP_START_RUNNING_TIMEOUT * 1000:
                logger_exe.warning("Sync Lost: Clamp status did not start running within %s sec" % CLAMP_START_RUNNING_TIMEOUT)
                stop_clamps()
                stop_robots()
                model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['"Sync Lost: Clamp status did not start running.']))
                model.ros_robot.send(rrc.Stop())
                return False

        # * Monitor if the Clamp Controller is still alive
        if clamp_command.status == ROS_COMMAND.RUNNING:
            if model.ros_clamps.last_received_message_age_ms > CLAMP_CONTROLLER_ALIVE_TIMEOUT * 1000:
                logger_exe.warning("Sync Lost: Clamp Commander not alive for more than %s sec" % CLAMP_CONTROLLER_ALIVE_TIMEOUT)
                stop_clamps()
                stop_robots()
                model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Sync Lost: Clamp Commander not alive.']))
                model.ros_robot.send(rrc.Stop())
                return False

        # * Monitor if the Clamp Controller reports a failed sync move.
        if clamp_command.status == ROS_COMMAND.FAILED:
            logger_exe.warning("Sync Lost: Clamp Command Failed.")
            stop_robots()
            stop_clamps()
            model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Sync Lost: Clamp Command Failed.']))
            model.ros_robot.send(rrc.Stop())
            return False

        # * Monitor if the Robot Controller Motion Task is Stopped
        task_excstate = model.ros_robot.send_and_wait(rrc.GetTaskExecutionState('T_ROB11'))
        # The task_excstate is still `started` even when the motion is completed.
        if task_excstate != "started":
            logger_exe.warning("Sync Lost: Robot Controller Task stopped.")
            stop_clamps()
            stop_robots()
            return False

        # * Monitor if UI Cancel
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("Sync Movement stopped from controller UI")
            stop_clamps()
            stop_robots()
            model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Sync Movement stopped from controller UI.']))
            model.ros_robot.send(rrc.Stop())
            return False

        # * Condition to exit loop when both clamps and robot are finished
        if robot_movement_completed() and clamp_command.status == ROS_COMMAND.SUCCEED:
            logger_exe.info("RobClamp Sync Move Completed")
            model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move Completed']))
            break

        time.sleep(0.05)

    ############################

    # * Final deviation
    # deviation = check_deviation(model, movement.target_frame)
    # if deviation is not None:
    #     logger_exe.info("Movement (%s) target frame deviation %s mm" % (movement.movement_id, deviation))
    #     guiref['exe']['last_deviation'].set("%.2fmm" % (deviation))
    # else:
    #     logger_exe.warning("execute_robotic_clamp_sync_linear_movement stopped before deviation result (future not arrived)")
    #     return False
    return True


def execute_clamp_jaw_movement(guiref, model: RobotClampExecutionModel, movement: ClampsJawMovement):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Clamp movement cannot start because Clamp ROS is not connected")
        return False

    # Remove clamp prefix:
    tool_ids = movement.clamp_ids
    positions = movement.jaw_positions
    velocity = model.settings[movement.speed_type]

    clamps_pos_velo = [(tool_id, position, velocity) for tool_id, position in zip(tool_ids, positions)]
    power_percentage = convert_movement_speed_type_to_power_poercentage(movement)
    allowable_target_deviation = movement.allowable_target_deviation

    command = ROS_VEL_GOTO_COMMAND(clamps_pos_velo, power_percentage, allowable_target_deviation)
    message = model.ros_clamps.send(command)

    # Wait for Clamp Controller to ACK
    while (True):
        if message.acked:
            logger_exe.info("Clamp Jaw Movement (%s) with %s Started" % (movement.movement_id, tool_ids))
            break

        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("Clamp Jaw Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
            return False

    # Wait for clamp to complete
    while (True):
        # * Monitor if the command succeeded
        if command.status == ROS_COMMAND.SUCCEED:
            logger_exe.info("Clamp Jaw Movement (%s) completed." % movement.movement_id)
            return True

        # * Monitor if the command failed
        if command.status == ROS_COMMAND.FAILED:
            logger_exe.info("Clamp Jaw Movement (%s) stopped or jammed." % movement.movement_id)
            return False

        # * Monitor if the Clamp Controller is still alive
        if command.status == ROS_COMMAND.RUNNING:
            if model.ros_clamps.last_received_message_age_ms > CLAMP_CONTROLLER_ALIVE_TIMEOUT * 1000:
                logger_exe.warning("Sync Lost: Clamp Commander not alive for more than %s sec" % CLAMP_CONTROLLER_ALIVE_TIMEOUT)
                model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
                return False

        # * Check if user pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning(
                "UI stop button pressed before Robotic Clamp Jaw Movement (%s) is completed." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
            return False


def execute_screwdriver_movement(guiref, model: RobotClampExecutionModel, movement: ScrewdriverMovement):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Clamp movement cannot start because Clamp ROS is not connected")
        return False

    # Remove clamp prefix:
    tool_ids = movement.tool_ids
    positions = movement.target_positions
    velocity = model.settings[movement.speed_type]

    clamps_pos_velo = [(tool_id, position, velocity) for tool_id, position in zip(tool_ids, positions)]
    allowable_target_deviation = movement.allowable_target_deviation
    power_percentage = convert_movement_speed_type_to_power_poercentage(movement)

    command = ROS_VEL_GOTO_COMMAND(clamps_pos_velo, power_percentage, allowable_target_deviation)
    message = model.ros_clamps.send(command)

    # Wait for Clamp Controller to ACK
    while (True):
        if message.acked:
            logger_exe.info("Clamp Jaw Movement (%s) with %s Started" % (movement.movement_id, tool_ids))
            break

        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("Clamp Jaw Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
            return False

    # Wait for clamp to complete
    while (True):
        # * Monitor if the command succeeded
        if command.status == ROS_COMMAND.SUCCEED:
            logger_exe.info("Clamp Jaw Movement (%s) completed." % movement.movement_id)
            return True

        # * Monitor if the command failed
        if command.status == ROS_COMMAND.FAILED:
            logger_exe.info("Clamp Jaw Movement (%s) stopped or jammed." % movement.movement_id)
            return False

        # * Monitor if the Clamp Controller is still alive
        if command.status == ROS_COMMAND.RUNNING:
            if model.ros_clamps.last_received_message_age_ms > CLAMP_CONTROLLER_ALIVE_TIMEOUT * 1000:
                logger_exe.warning("Sync Lost: Clamp Commander not alive for more than %s sec" % CLAMP_CONTROLLER_ALIVE_TIMEOUT)
                model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
                return False

        # * Check if user pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning(
                "UI stop button pressed before Robotic Clamp Jaw Movement (%s) is completed." % (movement.movement_id))
            model.ros_clamps.send(ROS_STOP_COMMAND(tool_ids))
            return False


def execute_operator_add_jog_offset_movement(guiref, model: RobotClampExecutionModel, movement: OperatorAddJogOffset):
    """Performs OperatorAddJogOffset Movement by stopping execution and ask user to jog (using the robot TP).
    Upon continuing program from robot TP, the difference between the robot flange frame and the original frame
    is used as grantry XYZ offset.

    A move is performed to make sure the offset is applied on external axis.
    The robot joints will go back to the previous joint values in movement.end_state['robot'].
    """
    # Read current robot external axis and joint values
    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        starting_robot_joints, starting_ext_axis = future.value
        logger_exe.info("Initial Joints: %s, %s" % (starting_robot_joints, starting_ext_axis))
    else:
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) GetJoints is completed." % (movement.movement_id))
        return False

    ############################################

    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Press STOP, JOG to align. Press PLAY when finished"], []))
    logger_exe.info("rrc.stop() issued")
    future = send_and_wait_unless_cancel(model, rrc.CustomInstruction('r_A067_Stop', [], []))
    if future.done:
        logger_exe.info("rrc.stop() returned")
    else:
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) rrc.Stop is completed." % (movement.movement_id))
        return False

    # Use UI Button to pause and confirm
    # button = guiref['exe']['confirm_button']
    # guiref['exe']['confirm_button_text'].set("Confirm after JOG")
    # button.config(state="normal", bg='orange')
    # model.operator_confirm = False

    # guiref['exe']['exe_status'].set("Paused")
    # from process_controller.run import ui_update_run_status
    # while (True):
    #     if model.operator_confirm:
    #         button.config(state="disabled", bg='grey')
    #         ui_update_run_status(guiref, model)
    #         break
    #     if model.run_status == RunStatus.STOPPED:
    #         button.config(state="disabled", bg='grey')
    #         ui_update_run_status(guiref, model)
    #         return False

    ############################################

    # Upon restart, read the current robot frame.
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        new_frame, new_ext_axes = future.value
        logger_exe.info("Frame after Jog: %s" % (new_frame))
    else:
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) GetRobTarget is completed." % (movement.movement_id))
        return False

    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        new_robot_joints, new_ext_axes = future.value
        logger_exe.info("Joints after Jog: %s, %s" % (new_robot_joints, new_ext_axes))
    else:
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) GetJoints is completed." % (movement.movement_id))
        return False
    ############################################

    # Compute offset and apply to external axis.
    offset = Vector.from_start_end(movement.original_frame.point, new_frame.point)
    logger_exe.info("Jogged Cartsian Offset = %s (amount = %4g mm)" % (offset, offset.length))

    # Compute Joint 6 offset
    end_state_robot_joints = to_degrees(movement.end_state['robot'].kinematic_config.revolute_values)
    joint_6_offset = new_robot_joints[5] - end_state_robot_joints[5]
    logger_exe.info("Jogged joint 6 offset = %s deg" % (joint_6_offset))

    # YZ axis of external axis has flipped direction
    guiref['offset']['Ext_X'].set("%.4g" % round(offset.x, 4))
    guiref['offset']['Ext_Y'].set("%.4g" % round(-1 * offset.y, 4))
    guiref['offset']['Ext_Z'].set("%.4g" % round(-1 * offset.z, 4))
    guiref['offset']['Rob_J1'].set("0")
    guiref['offset']['Rob_J2'].set("0")
    guiref['offset']['Rob_J3'].set("0")
    guiref['offset']['Rob_J4'].set("0")
    guiref['offset']['Rob_J5'].set("0")
    guiref['offset']['Rob_J6'].set("%.4g" % round(joint_6_offset, 4))

    # Movement to make sure joints have no offset
    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Offset registered: X %.3g Y %.3g Z %.3g." % (offset.x, offset.y, offset.z)], []))
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Press PLAY to accept with a gantry move."], []))
    logger_exe.info("rrc.stop() issued")
    future = send_and_wait_unless_cancel(model, rrc.Stop())
    if not future.done:
        logger_exe.info("rrc.stop() returned")
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) rrc.Stop is completed." % (movement.movement_id))
        return False

    move_joint_instruction = robot_state_to_instruction(guiref, model, movement.end_state['robot'].kinematic_config, 30, rrc.Zone.FINE)
    future = send_and_wait_unless_cancel(model, move_joint_instruction)

    # Logging Gantry move is complete
    if future.done:
        model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Gantry Move Complete"], []))
        logger_exe.info("OperatorAddJogOffset gantry move is complete")
    else:
        logger_exe.warning("UI stop button pressed before OperatorAddJogOffset (%s) TPPlot is completed." % (movement.movement_id))
        return False

    return True


def execute_remove_operator_offset_movement(guiref, model: RobotClampExecutionModel, movement: RemoveOperatorOffset):
    """Performs RemoveOperatorOffset Movement by cenceling all offset. """
    guiref['offset']['Ext_X'].set("0")
    guiref['offset']['Ext_Y'].set("0")
    guiref['offset']['Ext_Z'].set("0")
    guiref['offset']['Rob_J1'].set("0")
    guiref['offset']['Rob_J2'].set("0")
    guiref['offset']['Rob_J3'].set("0")
    guiref['offset']['Rob_J4'].set("0")
    guiref['offset']['Rob_J5'].set("0")
    guiref['offset']['Rob_J6'].set("0")
    logger_exe.info("Operator offset removed.")
    return True


def execute_set_workpiece_weight(guiref, model: RobotClampExecutionModel, movement: SetWorkpieceWeight):
    """Performs SetWorkpieceWeight Movement by calling the corrisponding robot function. """
    if (model.ros_robot is None) or not model.ros_robot.ros.is_connected:
        logger_exe.info(
            "Robot movement cannot start because Robot ROS is not connected")
        return False
        beam = model.process.assembly.beam(beam_id)

    # The mass (weight) of the load in kg.
    # Must be bigger then 0
    mass = movement.weight_kg
    if mass > 0:
        # The center of gravity of the payload expressed in mm in the tool coordinate system.
        # Minimum 1 value bigger then 0
        cog_x, cog_y, cog_z = movement.center_of_gravity

        # The orientation of the axes of moment.
        # These are the principal axes of the payload moment of inertia with origin in center of gravity.
        # Expressed in quaternians
        aom_q1 = 1
        aom_q2 = 0
        aom_q3 = 0
        aom_q4 = 0
        # The moment of inertia of the load around the axis of moment expressed in kgm2.
        # Correct definition of the moments of inertia will allow optimal utilization of the path planner and axes control.
        # This may be of special importance when handling large sheets of metal, and so on.
        # All moments of inertia ix, iy, and iz equal to 0 kgm2 imply a point mass.
        # Normally, the moments of inertia must only be defined when the distance from the mounting flange to the center of gravity
        # is less than the maximal dimension of the load.
        inertia_x = 0
        inertia_y = 0
        inertia_z = 0
        logger_exe.info("Sending r_A067_GripLoad for %s: mass = %skg, cog = %smm" % (movement.beam_id, movement.weight_kg, movement.center_of_gravity))
        instruction = rrc.CustomInstruction('r_A067_GripLoad', [], [mass, cog_x, cog_y, cog_z, aom_q1, aom_q2, aom_q3, aom_q4, inertia_x, inertia_y, inertia_z], feedback_level=rrc.FeedbackLevel.DONE)
    else:
        logger_exe.info("Sending r_A067_GripUnload.")
        instruction = rrc.CustomInstruction('r_A067_GripUnload', [], [], feedback_level=rrc.FeedbackLevel.DONE)

    future = send_and_wait_unless_cancel(model, instruction)
    return future.done

#####################################################
# Execute functions that are not based on Movement
#####################################################


def execute_jog_robot_to_config(guiref, model, config: Configuration, message: str, q):
    """Performs RoboticDigitalOutput Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """
    # Construct and send rrc command
    model.run_status = RunStatus.JOGGING
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
    ext_values = to_millimeters(config.prismatic_values)
    joint_values = to_degrees(config.revolute_values)

    # Apply Offsets
    ext_values = apply_ext_offsets(guiref, ext_values)
    joint_values = apply_joint_offsets(guiref, joint_values)

    if message != "":
        model.ros_robot.send(rrc.PrintText(message))

    instruction = rrc.MoveToJoints(joint_values, ext_values, 1000, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
    future = send_and_wait_unless_cancel(model, instruction)
    if future.done:
        logger_exe.info("execute_jog_robot_to_config complete")
        model.run_status = RunStatus.STOPPED
    else:
        logger_exe.warning("UI stop button pressed before MoveToJoints in JogRobotToState Movement is completed.")

    # Trigger update Status
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
    return future.done


def execute_operator_add_visual_offset_movement(guiref, model: RobotClampExecutionModel, movement: OperatorAddVisualOffset):
    """Performs OperatorAddVisualOffset by having an interactive dialog
    Allowing user to key in offset value in flange frame.
    The offset is applied to the gantry and """

    # Open a dialog and ask user to key in three offset values
    # Apply offset to flange frame offset, jog robot to the movement.end_state
    if not model.process.movement_has_end_robot_config(movement):
        logger_exe.warning("Error Attempt to execute OperatorAddVisualOffset but the movement end_state does not have robot config.")
        return False

    dialog = VisualOffsetPopup(guiref, model, movement)
    guiref['root'].wait_window(dialog.window)
    if dialog.accept:
        return True
    else:
        return False


def execute_acquire_docking_offset(guiref, model: RobotClampExecutionModel, movement: AcquireDockingOffset):
    """Performs AcquireDockingOffset by retriving the docking marker location.
    """
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "AcquireDockingOffset cannot start because Clamp ROS is not connected")
        return False

    # Use end config of previous robotic movement, or start config of next movement
    prev_robotic_movement = model.process.get_prev_robotic_movement(movement)
    next_robotic_movement = model.process.get_next_robotic_movement(movement)
    original_config = model.process.get_movement_end_robot_config(prev_robotic_movement) or model.process.get_movement_start_robot_config(next_robotic_movement)
    if original_config is None:
        logger_exe.warning("Error Attempt to execute execute_acquire_docking_offset but the movement end_state does not have robot config.")
        return False

    camera_stream_name = movement.tool_id
    logger_exe.info("Waiting for marker from camera %s" % (movement.tool_id))

    import roslibpy

    def markers_transformation_callback(message_string):

        # Print it to UI and keep track of one way latency.
        T = Transformation.from_data(json.loads(message_string['data']))
        model.ros_clamps.markers_transformation[camera_stream_name].append(T)
        logger_exe.debug("Received message_string = %s" % (message_string))

    # * Wait for marker transformation to come back
    max_iteration = 5
    for i in range(max_iteration):
        # * Reset list of observed transforamtion
        model.ros_clamps.markers_transformation[camera_stream_name] = []
        listener = roslibpy.Topic(model.ros_clamps, '/' + camera_stream_name, 'std_msgs/String')
        listener.subscribe(markers_transformation_callback)

        # * Store existing offset values
        prev_offset = []
        prev_offset.append(float(guiref['offset']['Ext_X'].get()))
        prev_offset.append(float(guiref['offset']['Ext_Y'].get()))
        prev_offset.append(float(guiref['offset']['Ext_Z'].get()))

        # * Acquire camera transformation and compute offset
        _last_check_index = 0
        while (True):
            # Check if camera frame have arrived or not

            num_markers = len(model.ros_clamps.markers_transformation[camera_stream_name])
            if num_markers > 5 and num_markers > _last_check_index:

                # * Double check the last two frames are agreeing with each other
                t_camera_from_observedmarker = model.ros_clamps.markers_transformation[camera_stream_name][-1]
                new_offset, correction_amount_XY, correction_amount_Z = compute_marker_correction(guiref, model, movement, t_camera_from_observedmarker)
                t_camera_from_observedmarker_2 = model.ros_clamps.markers_transformation[camera_stream_name][-2]
                new_offset_2, _, _ = compute_marker_correction(guiref, model, movement, t_camera_from_observedmarker_2)
                agreeable_threshold = 0.2
                difference = [abs(i-j) for i, j in zip(new_offset, new_offset_2)]

                # * If all difference agree, they can be
                if all([d < agreeable_threshold for d in difference]):
                    logger_exe.info("AcquireDockingOffset stream = %s attempt %s received t_camera_from_marker = %s" % (camera_stream_name, i, t_camera_from_observedmarker))
                    listener.unsubscribe()
                    break
                else:
                    logger_exe.info("AcquireDockingOffset last 2 frames correction larger than agreeable_threshold (%s) Difference = %s" % (agreeable_threshold, difference))
                _last_check_index = num_markers

            # Check if user pressed stop button in UI
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warning(
                    "UI stop button pressed before AcquireDockingOffset (%s) is completed." % movement.movement_id)
                listener.unsubscribe()
                return False

        # * If the offsets are smaller than threshold (converged), break from multiple run loop and return
        convergence_XY = 1.0
        convergence_Z = 0.5
        if (correction_amount_XY < convergence_XY and correction_amount_Z < convergence_Z):
            logger_exe.info("Correction converged below threshold in %i move: XY = %1.2f (threshold = %1.2f), Z = %1.2f (threshold = %1.2f)" %
                            (i, correction_amount_XY, convergence_XY, correction_amount_Z, convergence_Z))
            logger_exe.info("Current Gantry Offset Values: X=%s Y=%s Z=%s " %
                            (guiref['offset']['Ext_X'].get(), guiref['offset']['Ext_Y'].get(), guiref['offset']['Ext_Z'].get()))

            return True

        # * Sanity check
        correction_max = 30
        if (correction_amount_XY > correction_max and correction_amount_Z > correction_max):
            logger_exe.warning("Correction larger than allowable threshold: XY = %1.2f (threshold = %1.2f), Z = %1.2f (threshold = %1.2f)" % (correction_amount_XY, correction_max, correction_amount_Z, correction_max))
            return False

        logger_exe.info("Correction amount (relative to current position) (in Flange Coordinates): XY = %1.2f , Z = %1.2f" % (correction_amount_XY, correction_amount_Z))

        # * Apply new offsets to existing offsets
        guiref['offset']['Ext_X'].set("%.4g" % round(prev_offset[0] + new_offset[0], 4))
        guiref['offset']['Ext_Y'].set("%.4g" % round(prev_offset[1] + new_offset[1], 4))
        guiref['offset']['Ext_Z'].set("%.4g" % round(prev_offset[2] + new_offset[2], 4))
        # * Move the robot to end config of movement with new offset
        config = original_config
        ext_values = apply_ext_offsets(guiref, to_millimeters(config.prismatic_values))
        joint_values = apply_joint_offsets(guiref, to_degrees(config.revolute_values))

        logger_exe.info("Moving robot to new offset value.")
        instruction = rrc.MoveToJoints(joint_values, ext_values, 500, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
        future = send_and_wait_unless_cancel(model, instruction)
        if future.done:
            logger_exe.info("Robot move to new offset value complete.")
        else:
            logger_exe.warning("UI stop button pressed before MoveToJoints in AcquireDockingOffset Movement is completed.")
            return False

    logger_exe.warning("AcquireDockingOffset exhausted maxIteration %s without convergence" % max_iteration)
    return False


def execute_shake_gantry(guiref, model: RobotClampExecutionModel, shake_amount, shake_speed, shake_repeat, q):
    """This function is scheduled by the ShakeGantryPopup to create and send the shaking commands.
    This function is intended to be run in a separate deamon thread.

    """
    model.run_status = RunStatus.JOGGING
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))

    def failure_routine(message):
        logger_exe.info(message)
        model.run_status = RunStatus.STOPPED
        q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
        return False

    # * Enable softmove state.
    if not robot_softmove_blocking(model, enable=True, soft_direction="XY",
                                   stiffness=10, stiffness_non_soft_dir=90):
        return failure_routine("execute_shake_gantry() stopped beacause robot_softmove_blocking() failed.")

    # * Get current joint values
    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        robot_joints, external_axes = future.value
        logger_exe.info("execute_shake_gantry begins at robot_joints = %s, external_axes = %s" % (robot_joints, external_axes))
    else:
        return failure_routine("execute_shake_gantry canceled")

    # * Set acceleration to 100% for maximum shakiness
    model.ros_robot.send(rrc.SetAcceleration(100, 100))
    e_pts = []
    for a in range(3):
        for r in range(shake_repeat):
            # Left
            e_pts.append(deepcopy(external_axes))
            e_pts[-1][a] += shake_amount
            # Right
            e_pts.append(deepcopy(external_axes))
            e_pts[-1][a] -= shake_amount
        # Center
        e_pts.append(deepcopy(external_axes))

    # * Send all points
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Shake Gantry Begins']))

    for ext_axes in e_pts:
        # Send robot command
        # robot_11.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.FINE))
        model.ros_robot.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.Z0))

    future = send_and_wait_unless_cancel(model, rrc.MoveToJoints(robot_joints, external_axes, shake_speed, rrc.Zone.FINE))
    if future.done:
        logger_exe.info("execute_shake_gantry shaking complete")
    else:
        return failure_routine("execute_shake_gantry shaking canceled")

    # * Check toolchanger signal status
    future = send_and_wait_unless_cancel(model, rrc.ReadDigital('diUnitR11In3'))
    if future.done:
        # * Check signal, if it is equal to expected value, we return
        if future.value == 1:
            guiref['exe']['toolchanger_signal'].set("Locked")
        else:
            guiref['exe']['toolchanger_signal'].set("Unlocked")
    else:
        return failure_routine("execute_shake_gantry read digital canceled")

    # * Disable softmove state.
    if not robot_softmove_blocking(model, enable=False):
        return failure_routine("execute_shake_gantry() stopped beacause robot_softmove_blocking() failed.")

    model.run_status = RunStatus.STOPPED
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))

    return True


def execute_compare_joint_values(guiref, model, movement: RoboticMovement):
    """
    Read and display the last trajectory point values from the selected movement
    Dispaly the values with joint offsets added.
    Read from the robot current joint values
    Display the difference

    """
    # Construct and send rrc command
    config = model.process.get_movement_end_robot_config(movement)
    target_e = to_millimeters(config.prismatic_values)
    target_j = to_degrees(config.revolute_values)
    logger_exe.info("Current Robot Joints (Ext, Joint): %s, %s" % (target_e, target_j))

    # Apply Offsets
    target_e = apply_ext_offsets(guiref, target_e)
    target_j = apply_joint_offsets(guiref, target_j)
    logger_exe.info("Current Robot Joints (Ext, Joint): %s, %s" % (target_e, target_j))

    model.run_status = RunStatus.JOGGING
    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    model.run_status = RunStatus.STOPPED
    if future.done:
        _joint_values, _ext_values = future.value
        actual_j = list(_ext_values)[:3]
        actual_e = list(_joint_values)[:6]
        logger_exe.info("Current Robot Joints (Ext, Joint): %s, %s" % (actual_e, actual_j))
        error_j = [a-b for a, b in zip(actual_j, target_j)]
        error_e = [a-b for a, b in zip(actual_e, target_e)]
        logger_exe.info("Error (Ext, Joint): %s, %s" % (error_e, error_j))
    else:
        logger_exe.warning("UI stop button pressed before MoveToJoints in JogRobotToState Movement is completed.")

    return future.done


def execute_ui_toolchanger_probe(guiref, model, q):
    model.run_status = RunStatus.JOGGING
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
    success = execute_toolchanger_probe(guiref, model, q)
    model.run_status = RunStatus.STOPPED
    q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))


def execute_toolchanger_probe(guiref, model, q, probing_increment=0.5, maximum_probing_distance=15):
    """
    Move robot grantry towards the flange (=toolchanger)'s Z+ direction
    until probing switch signal is LOW.

    Funcion will return True if the switch successfully went LOW within a
    certain amount of probing distance. Otherwise return False.
    The probing distance is calculated from the actual starting value of the gantry position,
    not the

    When successful, the probing offset is set back to UI's offset.
    If unsuccessful, the UI's gantry offset is not changed.
    """
    # * Holder for offset
    new_offset = [0, 0, 0]
    prev_offset = get_ext_offsets(guiref)
    distance_since_start = 0

    # * Aquire starting gantry position and frame
    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        starting_j, starting_e = future.value
        logger_exe.info("execute_toolchanger_probe begins at external_axes = %s" % (starting_e))
    else:
        logger_exe.info("execute_toolchanger_probe cancelled.")
        return False

    future = send_and_wait_unless_cancel(model, rrc.GetFrame())
    if future.done:
        starting_frame = future.value
        logger_exe.info("execute_toolchanger_probe begins at starting_frame = %s" % (starting_frame))
    else:
        logger_exe.info("execute_toolchanger_probe cancelled.")
        return False

    # * Compute direction from probing_increment
    direction_vector = starting_frame.zaxis.unitized().scaled(probing_increment)

    while (distance_since_start < maximum_probing_distance):
        # * Check if probe switch is LOW
        future = send_and_wait_unless_cancel(model, rrc.ReadDigital('diUnitR11In5'))
        if future.done:
            # Exist condition for success
            if future.value == 0:
                logger_exe.info("Probe Touched.")
                # * Apply new offsets to existing offsets in UI
                guiref['offset']['Ext_X'].set("%.4g" % round(prev_offset[0] + new_offset[0], 4))
                guiref['offset']['Ext_Y'].set("%.4g" % round(prev_offset[1] + new_offset[1], 4))
                guiref['offset']['Ext_Z'].set("%.4g" % round(prev_offset[2] + new_offset[2], 4))
                logger_exe.info("execute_toolchanger_probe success after %smm, new gantry offset = %s" % (distance_since_start, get_ext_offsets(guiref)))
                return True
        else:
            logger_exe.info("execute_toolchanger_probe cancelled.")
            return False

        # * Compute new ext axis value
        new_offset[0] += direction_vector.x
        new_offset[1] += direction_vector.y * -1
        new_offset[2] += direction_vector.z * -1
        distance_since_start += probing_increment

        ext_values = apply_ext_offsets(guiref, list(starting_e)[:3])
        new_e = [p + q for p, q in zip(new_offset, ext_values)]

        # * Move gantry
        logger_exe.info("Moving gantry, current probing distance = %smm" % (distance_since_start))
        instruction = rrc.MoveToJoints(starting_j, new_e, 500, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
        future = send_and_wait_unless_cancel(model, instruction)
        if not future.done:
            logger_exe.warning("UI stop button pressed before MoveToJoints in JogRobotToState Movement is completed.")
            return False

    logger_exe.info("execute_toolchanger_probe failed after probing for %smm. Gantry offset unchanged." % distance_since_start)
    return False


def execute_some_delay(model: RobotClampExecutionModel, movement: Movement):
    for _ in range(10):
        time.sleep(0.3)
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warning("UI stop button pressed before execute_some_delay (%s) is completed." % (movement.movement_id))
            return False
    return True


def robot_goto_frame(model: RobotClampExecutionModel,  frame, speed):
    """Blocking call to go to a frame. Not cancelable."""
    instruction = rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR)
    send_and_wait_unless_cancel(model, instruction)


def robot_softmove(model: RobotClampExecutionModel, enable: bool, soft_direction="Z", stiffness=99, stiffness_non_soft_dir=100):
    """Non-blocking call to enable or disable soft move. Not cancelable.
    `soft_direction` modes available are "Z", "XY", "XYZ", "XYRZ"
    - use "XY" or "XYRZ" for pushing hard but allow deviation
    - use "Z" to avoid pushing hard but be accurate on other axis.

    `stiffness` in the specified direction, 0 is softest, 100 is stiffness
    `stiffness` in the other non specified direction, 0 is softest, 100 is stiffness

    future result is returned.
    """
    # model.ros_robot.send(rrc.SetTool('t_A067_T1_Gripper'))  # TODO: This should not be hard coded.
    if enable:
        future = model.ros_robot.send(rrc.CustomInstruction("r_A067_ActSoftMove",
                                                            string_values=[soft_direction],
                                                            float_values=[stiffness, stiffness_non_soft_dir], feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmove(Enabled) command sent: soft_direction = %s, stiffness = %i, stiffness_non_soft_dir = %i" %
                        (soft_direction, stiffness, stiffness_non_soft_dir))
    else:
        future = model.ros_robot.send(rrc.CustomInstruction(
            "r_A067_DeactSoftMove",  feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmove(Disable) command sent.")

    return future


def robot_softmove_blocking(model: RobotClampExecutionModel, enable: bool, soft_direction="Z", stiffness=99, stiffness_non_soft_dir=100):
    """Blocking call to change robot's softmove state. It can be used to enable or disable the softmove.
    When success, the state is set to model.ros_robot_state_softmove_enabled

    If the current model.ros_robot_state_softmove_enabled is the same as the `enable` input,
    no command will be sent and will immediately return True.

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the command is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """

    if (model.ros_robot_state_softmove_enabled is None) or (model.ros_robot_state_softmove_enabled != enable):

        result = robot_softmove(model, enable, soft_direction=soft_direction, stiffness=stiffness,
                                stiffness_non_soft_dir=stiffness_non_soft_dir)
        while (True):
            # Wait until softmove state is set.
            if result.done:
                logger_exe.info("Softmove is now %s" % ("Enabled" if enable else "Disabled"))
                model.ros_robot_state_softmove_enabled = enable
                return True
            # Stop waiting if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warning(
                    "robot_softmove_blocking stopped before future.done arrived")
                return False

    else:
        logger_exe.info("robot_softmove_blocking() skipped - current softmove state (%s) is the same." % model.ros_robot_state_softmove_enabled)
        return True

#########################################
# Visual Correction Helper Functions
#########################################


def compute_marker_correction(guiref, model: RobotClampExecutionModel, movement: AcquireDockingOffset, t_camera_from_observedmarker: Transformation, ):
    """Compute the gantry offset from the marker position.
    The movement must have a target_frame.

    Returns  (new_offset, correction_amount_XY, correction_amount_Z)
    - new_offset : three offset values that can be applied to gantry offset
    - correction_amount_XY : Correction amount in the Flange's coordinates
    - correction_amount_Z : Correction amount in the Flange's coordinates

    """

    # Retrive the selected movement target frame
    if not hasattr(movement, 'target_frame'):
        logger_model.warning("compute_visual_correction used on movement %s without target_frame" % (movement.movement_id))
        return False
    current_movement_target_frame = movement.target_frame
    t_world_from_currentflange = Transformation.from_frame(current_movement_target_frame)

    # * From CAD
    import clamp_controller
    import os.path as path
    camera_stream_name = movement.tool_id
    clamp_controller_path = path.dirname(path.dirname(path.dirname(clamp_controller.__file__)))
    json_path = path.join(clamp_controller_path, "calibrations", camera_stream_name + "_t_flange_from_marker.json")
    with open(json_path, 'r') as f:
        t_flange_from_marker = json.load(f, cls=DataDecoder)

    # * From Marker Calibration
    json_path = path.join(clamp_controller_path, "calibrations", camera_stream_name + "_t_camera_from_marker.json")
    with open(json_path, 'r') as f:
        t_camera_from_marker = json.load(f, cls=DataDecoder)

    # * Observation: Observed Marker in Camera Frame
    t_camera_from_observedmarker = t_camera_from_observedmarker

    # * Calcualtion: Camera in Flange Frame
    t_flange_from_camera = t_flange_from_marker * t_camera_from_marker.inverse()

    # * Calculation: New Flange Frame in current flange frame
    t_observedmarker_from_newflange = t_flange_from_marker.inverse()
    t_flange_from_newflange = t_flange_from_camera * t_camera_from_observedmarker * t_observedmarker_from_newflange  # type: Transformation
    v_flange_correction = t_flange_from_newflange.translation_vector
    v_world_flange_correction = v_flange_correction.transformed(t_world_from_currentflange)

    # * Convert correction vector to gantry offset values
    new_offset = []
    new_offset.append(v_world_flange_correction.x)
    new_offset.append(-1 * v_world_flange_correction.y)
    new_offset.append(-1 * v_world_flange_correction.z)

    # logger_exe.info("Marker correction t_flange_from_newflange = %s , v_world_flange_correction = %s" % (v_flange_correction, v_world_flange_correction))
    correction_amount_XY = Vector(v_flange_correction.x, v_flange_correction.y).length
    correction_amount_Z = v_flange_correction.z

    return (new_offset, correction_amount_XY, correction_amount_Z)


#########################
# rrc Helper Functions
#########################

def send_and_wait_unless_cancel(model: RobotClampExecutionModel, instruction: ROSmsg) -> rrc.FutureResult:
    """Send instruction and wait for feedback.

    This is a blocking call, it will return if the ABBClient
    send the requested feedback, or if model.run_status == RunStatus.STOPPED

    Returns the Future Result. If future.done == True, the result will contain the feedback.
    Otherwise the result is not received. Meaning the function returned from user cancel.

    Args:
        instruction: ROS Message representing the instruction to send.
        model: RobotClampExecutionModel where model.ros_robot contains a ABBClient
    """
    instruction.feedback_level = rrc.FeedbackLevel.DONE
    future = model.ros_robot.send(instruction)
    while (True):
        if future.done:
            return future
        if model.run_status == RunStatus.STOPPED:
            return future


def check_deviation(model: RobotClampExecutionModel, target_frame: Frame) -> Optional[float]:
    # Final deviation
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        actual_frame, ext_axes = future.value
        deviation = target_frame.point.distance_to_point(actual_frame.point)
        return deviation
    else:
        return None


def ensure_speed_ratio(model: RobotClampExecutionModel, robot: rrc.AbbClient, target_speed_ratio: float = 100):
    """Ensures the Speed Ratio of the robot matches the given `target_speed_ratio`.
    Otherwise, will write to TP to prompt user to change.
    User press PLAY after changing the ratio.

    Returns True if the user successfully changed the speed ratio to match.
    Returns False if
    """
    while (True):
        speed_ratio = robot.send_and_wait(GetSpeedRatio(), 2)
        print("speed_ratio = %s" % (speed_ratio))

        if abs(speed_ratio - target_speed_ratio) < 0.1:
            return True

        # Print Message to TP and ask for chaning the speed ratio.
        robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Change Speed Ratio to %i percent. Press PLAY to Continue.' % target_speed_ratio]))
        robot.send(rrc.Stop())
        robot.send_and_wait(rrc.WaitTime(0.1))
        # Allow UI to cancel
        if model.run_status == RunStatus.STOPPED:
            return False
