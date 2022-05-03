import logging

from compas.geometry.primitives.point import Point
from compas_fab.robots import to_degrees

from process_controller.GUI import *
from process_controller.ProcessModel import *

logger_exe = logging.getLogger("app.exe")


def trajectory_point_to_instruction(model: RobotClampExecutionModel, movement: Movement, guiref, point_n: int,
                                    apply_offset: bool = True,
                                    fine_zone_points: int = 1,
                                    fine_zone = rrc.Zone.FINE,
                                    intermediate_zone = rrc.Zone.Z1,
                                    feedback_level: rrc.FeedbackLevel = rrc.FeedbackLevel.DONE,
                                    ) -> rrc.MoveToJoints:
    """Create a rrc.MoveToJoints instruction for a trajectory point
    taking into account of the point_n in determining speed and zone
    apply offset set in the gui if apply_offset is true"""

    # Retrive a specific point in the trajectory.
    point = movement.trajectory.points[point_n]

    # Speed and Zone
    speed = model.settings[movement.speed_type]
    if point_n < len(movement.trajectory.points) - fine_zone_points:
        zone = intermediate_zone
    else:
        zone = fine_zone

    return robot_state_to_instruction(guiref, model, point, speed, zone, feedback_level)


def robot_state_to_instruction(guiref, model: RobotClampExecutionModel, robot_config: Configuration, speed, zone,
                               feedback_level: rrc.FeedbackLevel = rrc.FeedbackLevel.DONE,
                               apply_offset: bool = True):
    """Create rrc.MoveToJoints instruction from robot_state
    apply offset set in the gui if apply_offset is true"""
    assert len(robot_config.prismatic_values) == 3
    assert len(robot_config.revolute_values) == 6

    # Ext Axis and Joint Values
    ext_values = to_millimeters(robot_config.prismatic_values)
    joint_values = to_degrees(robot_config.revolute_values)

    # Apply Axis / Joints Offsets
    if apply_offset:
        ext_values = apply_ext_offsets(guiref, ext_values)
        joint_values = apply_joint_offsets(guiref, joint_values)

    # Command to Move
    return rrc.MoveToJoints(joint_values, ext_values, speed, zone, feedback_level=feedback_level)


def convert_movement_speed_type_to_power_poercentage(movement):
    if movement.speed_type == "speed.assembly.screw_assemble":
        power_percentage = 90
    elif movement.speed_type == "speed.assembly.screw_retract":
        power_percentage = 95
    elif movement.speed_type == "speed.assembly.screw_tighten":
        power_percentage = 80
    else:
        power_percentage = 99
    return power_percentage

##################
# Helper Functions
##################


def to_millimeters(meters):
    """Convert a list of floats representing meters to a list of millimeters.

    Parameters
    ----------
    radians : :obj:`list` of :obj:`float`
        List of distance values in meters.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of millimeters.
    """
    return [m * 1000.0 for m in meters]


def frame_to_millimeters(frame_in_meters: Frame):
    """Convert a list of floats representing meters to a list of millimeters.

    Parameters
    ----------
    radians : :obj:`list` of :obj:`float`
        List of distance values in meters.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of millimeters.
    """
    new_point = Point(frame_in_meters.point.x * 1000.0, frame_in_meters.point.y * 1000.0, frame_in_meters.point.z * 1000.0, )
    return Frame(new_point, frame_in_meters.xaxis, frame_in_meters.yaxis)


def compute_visual_correction(guiref, model: RobotClampExecutionModel, movement: RoboticMovement):
    """Compute the gantry offset from the visual offset in gui.
    The movement must have a target_frame"""
    align_X = guiref['offset']['Visual_X'].get()
    align_Y = guiref['offset']['Visual_Y'].get()
    align_Z = guiref['offset']['Visual_Z'].get()

    # Retrive the selected movement target frame
    if not hasattr(movement, 'target_frame'):
        logger_exe.warning("compute_visual_correction used on movement %s without target_frame" % (movement.movement_id))
        return False
    current_movement_target_frame = movement.target_frame

    from compas.geometry.primitives.vector import Vector
    from compas.geometry.transformations.transformation import Transformation

    T = Transformation.from_frame(current_movement_target_frame)
    flange_vector = Vector(align_X, align_Y, align_Z)
    world_vector = flange_vector.transformed(T)
    guiref['offset']['Ext_X'].set("%.4g" % round(world_vector.x, 4))
    guiref['offset']['Ext_Y'].set("%.4g" % round(-1 * world_vector.y, 4))
    guiref['offset']['Ext_Z'].set("%.4g" % round(-1 * world_vector.z, 4))
    logger_model.info("Visual correction of Flange %s from Flange %s. Resulting in World %s" % (flange_vector, current_movement_target_frame, world_vector))
    return True
