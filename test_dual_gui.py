import sys
sys.path.append('src/kuka_gui_control')

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal

from kuka_gui_control.dual_command_model import DualCommandModel
from kuka_gui_control.dual_kuka_rviz_window import DualKukaRvizWindow

app = QApplication(sys.argv)
model = DualCommandModel()

class DummyKukaBridge:
    class Signals(QObject):
        feedback_received = pyqtSignal(str)
        raw_command_xml_received = pyqtSignal(str)
        raw_robot_xml_received = pyqtSignal(str)
        ros_status_changed = pyqtSignal(bool)
    
    def __init__(self):
        self.s = self.Signals()
        self.feedback_received = self.s.feedback_received
        self.raw_command_xml_received = self.s.raw_command_xml_received
        self.raw_robot_xml_received = self.s.raw_robot_xml_received
        self.ros_status_changed = self.s.ros_status_changed

class DummyRvizBridge:
    class Signals(QObject):
        rviz_joint_state_received = pyqtSignal(str)
        rviz_cartesian_state_received = pyqtSignal(str)
        rviz_status_received = pyqtSignal(str)

    def __init__(self):
        self.s = self.Signals()
        self.rviz_joint_state_received = self.s.rviz_joint_state_received
        self.rviz_cartesian_state_received = self.s.rviz_cartesian_state_received
        self.rviz_status_received = self.s.rviz_status_received

kuka = DummyKukaBridge()
rviz = DummyRvizBridge()
cfg = {'home_joints_deg': {}, 'soft_limits_deg': {}}

try:
    win = DualKukaRvizWindow(model, kuka, rviz, cfg)
    print("SUCCESS")
except Exception as e:
    import traceback
    traceback.print_exc()

