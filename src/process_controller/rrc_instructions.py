import compas_rrc as rrc
from compas_fab.backends.ros.messages import ROSmsg
from compas_rrc.common import SystemInstruction

from twisted.internet import reactor
reactor.timeout = lambda:0.0001

class SetDigitalIO(ROSmsg, SystemInstruction):
    def __init__(self, signal_name, signal_value, feedback_level=rrc.FeedbackLevel.DATA):
        if signal_value not in (0, 1):
            raise ValueError('Signal value must be 0 or 1')

        self.instruction = 'set_digital_io'
        self.feedback_level = feedback_level
        self.exec_level = rrc.ExecutionLevel.DRIVER
        self.string_values = [signal_name]
        self.float_values = [signal_value]

class GetDigitalIO(ROSmsg, SystemInstruction):
    def __init__(self, signal_name, feedback_level=rrc.FeedbackLevel.DATA):
        self.instruction = 'get_digital_io'
        self.feedback_level = feedback_level
        self.exec_level = rrc.ExecutionLevel.DRIVER
        self.string_values = [signal_name]
        self.float_values = []

    def parse_feedback(self, response):
        return response['float_values'][0]

class GetControllerState(ROSmsg, SystemInstruction):
    def __init__(self, feedback_level=rrc.FeedbackLevel.DATA):
        self.instruction = 'get_controller_state'
        self.feedback_level = feedback_level
        self.exec_level = rrc.ExecutionLevel.DRIVER
        self.string_values = []
        self.float_values = []

    def parse_feedback(self, response):
        return response['string_values'][0]

class GetSpeedRatio(ROSmsg, SystemInstruction):
    def __init__(self, feedback_level=rrc.FeedbackLevel.DATA):
        self.instruction = 'get_speed_ratio'
        self.feedback_level = feedback_level
        self.exec_level = rrc.ExecutionLevel.DRIVER
        self.string_values = []
        self.float_values = []

    def parse_feedback(self, response):
        return response['float_values'][0]
