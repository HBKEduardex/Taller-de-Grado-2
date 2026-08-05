from kuka_gui_control.dual_command_model import DualCommandModel
model = DualCommandModel()
arr = model.build_rviz_joint_array()
print(arr)
