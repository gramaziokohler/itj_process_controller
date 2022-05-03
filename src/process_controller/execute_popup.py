from process_controller.GUI import *
from process_controller.ProcessModel import *
from process_controller.execute_helper import *

# from integral_timber_joints.process.movement import *


class VisualOffsetPopup(object):

    def __init__(self, guiref,  model: RobotClampExecutionModel, movement: OperatorAddVisualOffset):
        self.window = tk.Toplevel(guiref['root'])
        self.guiref = guiref
        self.model = model
        self.movement = movement
        self.accpet = False

        tk.Label(self.window, text="Offset from the camera target (mm)?").grid(row=0, column=0, columnspan=3)

        # Entry Box for XYZ
        # self.offset_x = tk.StringVar(value="0")
        tk.Label(self.window, text="X").grid(row=1, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_X']).grid(row=1, column=1, columnspan=3)
        # self.offset_y = tk.StringVar(value="0")
        tk.Label(self.window, text="Y").grid(row=2, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_Y']).grid(row=2, column=1, columnspan=3)
        # self.offset_z = tk.StringVar(value="0")
        tk.Label(self.window, text="Z").grid(row=3, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_Z']).grid(row=3, column=1, columnspan=3)

        # Buttons
        tk.Button(self.window, text='Go', command=self.go).grid(row=4, column=0)
        tk.Button(self.window, text='Accept', command=self.accept).grid(row=4, column=1)
        tk.Button(self.window, text='Cancel', command=self.cancel).grid(row=4, column=2)

        self.value = None

    def go(self):
        compute_visual_correction(self.guiref, self.model, self.movement)
        robot_config = self.movement.end_state['robot'].kinematic_config
        move_instruction = robot_state_to_instruction(self.guiref, self.model, robot_config, 30, rrc.Zone.FINE)
        self.model.ros_robot.send(move_instruction)

    def accept(self):
        compute_visual_correction(self.guiref, self.model, self.movement)
        self.accpet = True
        self.window.destroy()

    def cancel(self):
        self.model.run_status = RunStatus.STOPPED
        self.accpet = False
        self.window.destroy()


class ShakeGantryPopup(object):

    def __init__(self, guiref,  model: RobotClampExecutionModel, q):
        self.window = tk.Toplevel(guiref['root'], width=200)
        self.guiref = guiref
        self.model = model
        self.q = q

        tk.Label(self.window, text="Shake robot until ToolChanger Lock").grid(row=0, column=0, columnspan=2)

        # Entry Box for XYZ
        self.shake_amount = tk.DoubleVar(value=0.5)
        self.shake_speed = tk.DoubleVar(value=10)
        self.shake_repeat = tk.IntVar(value=1)

        tk.Label(self.window, text="Shake Amount").grid(row=1, column=0)
        tk.Scale(self.window, variable=self.shake_amount, from_=0.3, to_=3, orient=tk.HORIZONTAL, resolution=0.1, width=20, length=200).grid(row=1, column=1)
        tk.Label(self.window, text="Shake Speed mm / s").grid(row=2, column=0)
        tk.Scale(self.window, variable=self.shake_speed, from_=5, to_=30, orient=tk.HORIZONTAL, resolution=0.1, width=20, length=200).grid(row=2, column=1)
        tk.Label(self.window, text="Shake Repeat").grid(row=3, column=0)
        tk.Scale(self.window, variable=self.shake_repeat, from_=1, to_=3, orient=tk.HORIZONTAL, resolution=1, width=20, length=200).grid(row=3, column=1)

        tk.Label(self.window, text="Tool Changer Signal", height=5).grid(row=4, column=0)
        tk.Label(self.window, textvariable=self.guiref['exe']['toolchanger_signal']).grid(row=4, column=1)

        # Buttons
        tk.Button(self.window, text='Shake', command=self.shake).grid(row=5, column=0)
        tk.Button(self.window, text='Cancel', command=self.cancel).grid(row=5, column=1)

        self.value = None

    def shake(self):
        self.q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_SHAKE_GANTRY, shake_amount=self.shake_amount.get(), shake_speed=self.shake_speed.get(), shake_repeat=self.shake_repeat.get()))
        pass
    #     compute_visual_correction(self.guiref, self.model, self.movement)
    #     compute_visual_correction(self.guiref, self.model, self.movement)
    #     robot_config = self.movement.end_state['robot'].kinematic_config
    #     move_instruction = robot_state_to_instruction(self.guiref, self.model, robot_config, 30, rrc.Zone.FINE)
    #     self.model.ros_robot.send(move_instruction)

    def cancel(self):
        self.model.run_status = RunStatus.STOPPED
        self.q.put(SimpleNamespace(type=ProcessControllerBackgroundCommand.UI_UPDATE_STATUS))
        self.window.destroy()


class MovementJsonPopup(object):

    def __init__(self, guiref,  model: RobotClampExecutionModel, movement: Movement):
        self.window = tk.Toplevel(guiref['root'])
        self.guiref = guiref
        self.model = model
        self.movement = movement

        tk.Label(self.window, text="Offset from the camera target (mm)?").grid(row=0, column=0)

        # Entry Box for XYZ
        # self.offset_x = tk.StringVar(value="0")
        t = tk.Text(self.window, height=200, width=250)
        t.grid(row=1, column=0)

        from compas.utilities import DataEncoder
        json_data = json.dumps(movement, indent=2, sort_keys=True, cls=DataEncoder)
        t.insert(tk.END, json_data)

        # Buttons
        tk.Button(self.window, text='Close', command=self.close).grid(row=2, column=0)

    def close(self):
        self.window.destroy()


class AlternativeStartPointWindow(object):
    def __init__(self, master, max_number, current_number=None):

        top = self.top = tk.Toplevel(master)
        self.l = tk.Label(top, text="Which point to start from? [0 to %i]" % max_number)
        self.l.pack()

        self.e = tk.Entry(top, font=tk.big_button_font)
        self.e.pack(side=tk.LEFT, fill=tk.BOTH)
        if current_number is not None:
            self.e.insert(tk.END, str(current_number))
        self.b = tk.Button(top, text='.  Go  .', command=self.cleanup, font=tk.big_button_font, height=3)
        self.b.pack()
        self.value = None

    def cleanup(self):
        self.value = int(self.e.get())
        self.top.destroy()
