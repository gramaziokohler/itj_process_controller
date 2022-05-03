import datetime
import json
import logging
from collections import OrderedDict
from enum import Enum
from functools import partial
from threading import Thread
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple
from copy import deepcopy
import time

from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall
from compas.utilities import DataDecoder
from compas_fab.backends.ros import RosClient
from compas_rrc import AbbClient
from integral_timber_joints.process import *
from integral_timber_joints.process.movement import *
from process_controller.background_command import *


from roslibpy import Ros
import compas_rrc as rrc
import os   # Save file location
import tempfile  # Save file location

logger_model = logging.getLogger("app.model")
logger_ros = logging.getLogger("app.ros")


class RunStatus(Enum):
    RUNNING = auto()
    STEPPING_FORWARD = auto()
    STEPPING_FORWARD_FROM_PT = auto()
    STEPPING_BACKWARD_FROM_PT = auto()
    ERROR = auto()
    STOPPED = auto()
    JOGGING = auto()


class RobotClampExecutionModel(object):

    def __init__(self):
        ##################
        # Process File
        ##################
        self.process: RobotClampAssemblyProcess = None
        self.process_path = ""

        # Ordered Dict for easy accessing the movements via move_id
        self.movements = OrderedDict()

        self.logger = logging.getLogger("app.mdl")

        ####################################
        # ROS Robot and Clamp Connections
        ####################################

        self.ros_clamps: RemoteClampFunctionCall = None
        self.ros_clamps_status = {}

        self.ros_robot: AbbClient = None
        self.operator_confirm = False

        # Robot states
        self.ros_robot_state_softmove_enabled = None

        #################
        # Execution
        #################

        # Flag to indicate if an movement is active
        self.run_status: RunStatus = RunStatus.STOPPED
        self.run_thread: Thread = None
        self.alternative_start_point = 0 # Index for Stepping from a specific trajectory point

        # Pointer to the currently selected action
        self.current_action: Action = None
        # Pointer to the currently selected movement
        self.current_movement: Movement = None

        # Settings
        self.settings = {}  # type: dict(str, str)
        self.load_settings()

    def load_settings(self, path=None):
        if path is None:
            path = os.path.join(tempfile.gettempdir(),
                                "itj_process_execution_setting.json")

        # Default Settings
        self.settings = {
            # Speed Settings in mm/s
            'speed.transfer.rapid': 500,
            'speed.transfer.caution': 100,
            'speed.transit.rapid': 800,
            'speed.toolchange.approach.withtool': 20,
            'speed.toolchange.approach.notool': 40,
            'speed.toolchange.retract.withtool': 20,
            'speed.toolchange.retract.notool': 40,
            'speed.toolchange.retract.clamp_on_structure': 5,
            'speed.toolchange.post_retract.withtool': 80,
            'speed.toolchange.pre_approach.withtool': 80,
            'speed.docking.approach.clamp_on_structure': 5,
            'speed.docking.approach.screwdriver_on_structure': 5,
            'speed.assembly.screw_approach': 10,
            'speed.assembly.screw_assemble': 0.7,
            'speed.assembly.screw_tighten': 0.7,
            'speed.assembly.screw_retract': 0.8,
            'speed.assembly.screw_pullout': 10,
            'speed.assembly.inclamp': 50,   # Sliding beam into clamp.
            'speed.assembly.noclamp': 20,   # Simply putting it down
            'speed.assembly.clamping': 2,  #
            'speed.gripper.approach': 50,  # Approaching Pickup Station
            'speed.gripper.retract': 50,    # Retract after placing
            'speed.clamp.rapid': 5,
            'robot.joint_offset': [0, 0, 0, 0, 0, 0]
        }

        # Load Previously saved settings if exist
        if os.path.exists(path):
            with open(path, 'r') as f:
                self.settings.update(json.load(f))
            logger_model.info("Settings loaded from %s." % path)

        # Save it back to disk (useful during development)
        with open(path, 'w') as f:
            json.dump(self.settings, f, indent=4, sort_keys=True)
            logger_model.info("Settings saved to %s." % path)

    def load_process(self, json_path):
        """Load the Process object from json file, adds a tree_row_id into each action and movement."""

        with open(json_path, 'r') as f:
            self.process = json.load(f, cls=DataDecoder)
        logger_model.info("load_process(): %s" % self.process_description)
        self.process_path = json_path

        # Load external movements if possible
        external_movements = self.process.load_external_movements(os.path.dirname(self.process_path))
        logger_model.info("External Movemenmts: %s movements loaded" % len(external_movements))

        # mark_movements_as_soft_move
        _mark_movements_as_softmove(self.process)
        # some moves do not need operator stop anymore
        # _mark_movements_operator_stop(self.process)
        # Insert operator offsets
        # _insert_offset_movements(self.process)

        # Organize movements into an OrderedDict collection for UI manupulation
        self.movements = OrderedDict()  # type : OrderedDict(Movement)
        for i, action in enumerate(self.process.actions):
            action.tree_row_id = 'a_%i' % action.act_n
            for move_n, movement in enumerate(action.movements):
                movement.tree_row_id = "m_%s" % (movement.movement_id)
                self.movements[movement.tree_row_id] = movement

    def settings_file_path_default(self):
        return os.path.join(tempfile.gettempdir(), "itj_process_execution_setting.json")

    def open_setting_file(self, path=None):
        """Opens the setting fiel in note pad """
        if path is None:
            path = self.settings_file_path_default()
        # Load Previously saved settings if exist
        if os.path.exists(path):
            osCommandString = "notepad.exe " + path
            os.system(osCommandString)

    def connect_ros_clamps(self, ip, status_change_callback=None):
        """Function to connect to ROS CLamps Client.
        Returns True on successful connection"""

        # Disconnect from previous host
        if (self.ros_clamps is not None) and (self.ros_clamps.is_connected):
            try:
                self.ros_clamps.close()
                logger_model.info("Previous Clamps ROS host disconnected")
            except:
                pass

        self.ros_clamps = RemoteClampFunctionCall(ip, status_change_callback=status_change_callback)  # ros_clamps_callback disabled
        try:
            # This runs in a separate thread
            self.ros_clamps.run(timeout=2)
            time.sleep(0.5)
            logger_model.info("Clamps ROS host connected. ip= %s" % ip)
            return True
        except:
            logger_model.info("Failed to connect to Clamps ROS host. ip= %s" % ip)

            self.ros_clamps = None
            return False

    def connect_ros_robots(self, ip, q):
        """Function to connect to ROS CLamps Client.
        Returns True on successful connection"""

        # Disconnect from previous host
        if (self.ros_robot is not None) and (self.ros_robot.ros.is_connected):
            try:
                self.ros_robot.close()
                logger_model.info("Previous ABB Robot ROS host disconnected")
            except:
                pass

        try:
            ros = RosClient(ip)
            # This runs in a separate thread
            ros.run()
            self.ros_robot = rrc.AbbClient(ros, '/rob1')
            time.sleep(0.5)
            logger_model.info("ABB Robot ROS host connected")
            return True
        except:
            self.ros_robot = None
            return False
            pass

    # def joint_offset(self, robot_joint_values):
    #     assert len(robot_joint_values) == 6
    #     return [(value + offset) for (value, offset) in zip(robot_joint_values, self.settings["robot.joint_offset"])]

    @property
    def process_description(self):
        if self.process is None:
            return "Process Not Loaded"
        else:
            return "Process Loaded (%i Beams, %i Actions, %i Movements)" % (
                len(self.process.assembly.sequence),
                len(self.process.actions),
                len(self.process.movements),
            )

#######################
# Functions for modifying process only for execution
#


def _mark_movements_as_softmove(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Marks selected movements of the assembly process as soft move."""
    count_RoboticClampSyncLinearMovement = 0
    count_RobotScrewdriverSyncLinearMovement = 0
    count_RobotScrewdriverRetractMovement = 0
    for i, action in enumerate(process.actions):
        for j, movement in enumerate(action.movements):
            # Default
            movement.softmove = False
            # Softmove cases
            if isinstance(movement, RoboticClampSyncLinearMovement):
                movement.softmove = True
                count_RoboticClampSyncLinearMovement += 1
            # Softmove cases
            if isinstance(movement, RobotScrewdriverSyncLinearMovement):
                movement.softmove = True
                count_RobotScrewdriverSyncLinearMovement += 1
            # Softmove cases
            if isinstance(action, RetractScrewdriverFromBeamAction):
                if isinstance(movement, RoboticLinearMovement):
                    movement.softmove = True
                    count_RobotScrewdriverRetractMovement += 1

    logger_model.info("%i RoboticClampSyncLinearMovement Marked as Soft Move" % (count_RoboticClampSyncLinearMovement))
    logger_model.info("%i RobotScrewdriverSyncLinearMovement Marked as Soft Move" % (count_RobotScrewdriverSyncLinearMovement))
    logger_model.info("%i count_RobotScrewdriverRetractMovement Marked as Soft Move" % (count_RobotScrewdriverRetractMovement))


def _mark_movements_operator_stop(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Marks selected movements to have more or less operator stops"""
    for i, action in enumerate(process.actions):

        # No need confirm after tool changer undocked
        if isinstance(action, PlaceClampToStructureAction):
            action.movements[3].operator_stop_before = None
            action.movements[3].operator_stop_after = None
            action.movements[5].operator_stop_before = "Confirm Clamp is stable"
            action.movements[5].operator_stop_after = None

        if isinstance(action, PickClampFromStructureAction):
            action.movements[1].operator_stop_before = None
            action.movements[1].operator_stop_after = None


class OperatorAddJogOffset(Movement):
    def __init__(self, original_frame=None, tag=None):
        # type: (Frame, str) -> OperatorAddJogOffset
        Movement.__init__(self, planning_priority=-1, tag=tag)
        self.original_frame = original_frame.copy()  # type: Frame
        self.tag = tag or "Opeartor Jog Robot to obtain a cartesian offset."

    def __str__(self):
        return "OperatorAddJogOffset for target frame at %s." % (self.original_frame)


class OperatorAddVisualOffset(Movement):
    def __init__(self, target_frame=None, tag=None):
        # type: (Frame, str) -> OperatorAddVisualOffset
        Movement.__init__(self, planning_priority=-1, tag=tag)
        self.target_frame = target_frame.copy()  # type: Frame
        self.tag = tag or "Opeartor key-in visual offset, resulting in cartesian offset."

    def __str__(self):
        return "OperatorAddVisualOffset for target frame at %s." % (self.target_frame)


class RemoveOperatorOffset(Movement):
    def __init__(self, beam_id=None, tag=None):
        # type: (str, Frame, str) -> OperatorAddVisualOffset
        Movement.__init__(self, planning_priority=-1, tag=tag)
        self.tag = tag or "Removing operator offset"


def _insert_offset_movements(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Adds OperatorAddJogOffset, OperatorAddVisualOffset and RemoveOperatorOffset movements of the assembly process as soft move"""
    for i, action in enumerate(process.actions):
        # Aligning clamp pins with holes on joint
        if isinstance(action, PlaceClampToStructureAction):
            # Retrive the second linear approach movement
            approach_movement = action.movements[2]  # type: RoboticLinearMovement
            offset_on = OperatorAddJogOffset(approach_movement.target_frame, tag="Jog offset until gripper pin in hole")
            offset_on.movement_id = "A%i_O0" % action.act_n
            offset_on.end_state = deepcopy(approach_movement.end_state)

            offset_off = RemoveOperatorOffset(tag="Remove offset after clamp placement")
            offset_off.movement_id = "A%i_O1" % action.act_n
            offset_off.end_state = deepcopy(action.movements[-1].end_state)

            action.movements.insert(3, offset_on)
            action.movements.append(offset_off)

        # Aligning joints before clamping
        if isinstance(action, BeamPlacementWithClampsAction):
            # Align timber joints before clamping
            pre_clamp_movement = action.movements[1]  # type: RoboticLinearMovement
            offset_on = OperatorAddJogOffset(pre_clamp_movement.target_frame, tag="Jog offset until joints align")
            offset_on.movement_id = "A%i_O0" % action.act_n
            offset_on.end_state = deepcopy(action.movements[2].end_state)

            offset_off = RemoveOperatorOffset(tag="Remove offset after beam placement")
            offset_off.movement_id = "A%i_O1" % action.act_n
            offset_off.end_state = deepcopy(action.movements[-1].end_state)

            action.movements.insert(3, offset_on)
            action.movements.append(offset_off)

        # Aligning clamp pickup via docking camera
        if isinstance(action, PickClampFromStructureAction):
            # Align timber joints before clamping
            pre_docking_movement = action.movements[0]  # type: RoboticLinearMovement
            offset_on = OperatorAddVisualOffset(pre_docking_movement.target_frame, tag="Key in visual target offset.")
            offset_on.movement_id = "A%i_O0" % action.act_n
            offset_on.end_state = deepcopy(action.movements[0].end_state)

            offset_off = RemoveOperatorOffset(tag="Remove offset after beam placement")
            offset_off.movement_id = "A%i_O1" % action.act_n
            offset_off.end_state = deepcopy(action.movements[-1].end_state)

            action.movements.insert(1, offset_on)
            action.movements.append(offset_off)
