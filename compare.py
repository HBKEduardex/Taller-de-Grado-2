import re

with open('src/kuka_gui_control/kuka_gui_control/gui_axis_move_window.py') as f:
    orig = f.read()
    
with open('src/kuka_gui_control/kuka_gui_control/dual_kuka_rviz_window.py') as f:
    dual = f.read()

# Extract _validate_and_send
m1 = re.search(r'def _validate_and_send.*?# ==', orig, re.DOTALL)
m2 = re.search(r'def _validate_and_send.*?# ==', dual, re.DOTALL)

print("--- ORIGINAL _validate_and_send ---")
print(m1.group(0) if m1 else "NOT FOUND")
print("\n--- DUAL _validate_and_send ---")
print(m2.group(0) if m2 else "NOT FOUND")
