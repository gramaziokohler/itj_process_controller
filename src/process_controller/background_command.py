from enum import Enum, auto


class ProcessControllerBackgroundCommand(Enum):
    UI_ROBOT_CONNECT = auto()
    UI_CLAMP_CONNECT = auto()

    UI_RUN = auto()
    UI_STEP = auto()
    UI_STEP_FROM_POINT = auto()
    UI_STOP = auto()
    UI_CONFIRM = auto()
    UI_UPDATE_STATUS = auto()
    UI_GOTO_END_FRAME = auto()
    UI_GOTO_START_STATE = auto()
    UI_SHAKE_GANTRY = auto()
    UI_SHAKE_GANTRY_POPUP = auto()
    UI_RESTART_CAMERA = auto()
    UI_GOTO_END_STATE = auto()
    UI_LOAD_EXT_MOVEMENT = auto()
    UI_OPEN_SETTING = auto()
    UI_TREEVIEW_GOTO_BEAM = auto()
    UI_SOFTMODE_ENABLE = auto()
    UI_SOFTMODE_DISABLE = auto()
    UI_COMPUTE_VISUAL_CORRECTION = auto()

    PRINT_ACTION_SUMMARY = auto()
    MODEL_LOAD_PROCESS = auto()
    EXE_CLAMPS_JAMMED = auto()

    TEST = auto()