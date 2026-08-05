import re
import os

source_file = 'src/kuka_gui_control/kuka_gui_control/gui_axis_move_window.py'
dest_file = 'src/kuka_gui_control/kuka_gui_control/dual_kuka_rviz_window.py'

with open(source_file, 'r') as f:
    content = f.read()

# 1. Update imports
content = content.replace(
    'from kuka_gui_control.joint_command_model import JointCommandModel',
    'from kuka_gui_control.dual_command_model import DualCommandModel\nfrom std_msgs.msg import Float64MultiArray'
)

# 2. Update class definition
content = content.replace('class AxisMoveGuiWindow(QMainWindow):', 'class DualKukaRvizWindow(QMainWindow):')
content = content.replace('model: JointCommandModel,', 'model: DualCommandModel,')
content = content.replace('bridge,  # RosAxisMoveBridge', 'kuka_bridge,\n        rviz_bridge,')

# 3. Update constructor body
content = content.replace('self._bridge = bridge', 'self._kuka_bridge = kuka_bridge\n        self._rviz_bridge = rviz_bridge')
content = content.replace("self.setWindowTitle('KUKA Joint Control GUI — AxisMove')", "self.setWindowTitle('Dual KUKA + RViz Control GUI')")

# 4. Connect signals
old_signals = """        # Connect bridge signals
        self._bridge.feedback_received.connect(self._on_feedback)
        self._bridge.raw_command_xml_received.connect(self._on_raw_command_xml)
        self._bridge.raw_robot_xml_received.connect(self._on_raw_robot_xml)
        self._bridge.ros_status_changed.connect(self._on_ros_status)"""

new_signals = """        # Connect KUKA bridge signals
        self._kuka_bridge.feedback_received.connect(self._on_feedback)
        self._kuka_bridge.raw_command_xml_received.connect(self._on_raw_command_xml)
        self._kuka_bridge.raw_robot_xml_received.connect(self._on_raw_robot_xml)
        self._kuka_bridge.ros_status_changed.connect(self._on_ros_status)

        # Connect RViz bridge signals
        self._rviz_bridge.rviz_joint_state_received.connect(self._on_rviz_joint_state)
        self._rviz_bridge.rviz_cartesian_state_received.connect(self._on_rviz_cartesian_state)
        self._rviz_bridge.rviz_status_received.connect(self._on_rviz_status)"""

content = content.replace(old_signals, new_signals)

# 5. Update Table Headers and Labels
old_table = """        # Header
        for col, hdr in enumerate(['Joint', 'Target (deg)', 'Feedback (deg)', 'Error (deg)']):"""
new_table = """        # Header
        headers = ['Joint', 'Target', 'KUKA fb', 'RViz fb', 'Err KUKA', 'Err RViz']
        for col, hdr in enumerate(headers):"""
content = content.replace(old_table, new_table)

old_table_cart = """        # Header
        for col, hdr in enumerate(['Axis', 'Target', 'Feedback', 'Error']):"""
new_table_cart = """        # Header
        headers_cart = ['Axis', 'Target', 'KUKA fb', 'RViz fb', 'Err KUKA', 'Err RViz']
        for col, hdr in enumerate(headers_cart):"""
content = content.replace(old_table_cart, new_table_cart)

old_table_labels = """            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_err, row, 3)
            self._table_labels[axis]['error'] = lbl_err"""
new_table_labels = """            lbl_rviz_fb = QLabel('N/A')
            lbl_rviz_fb.setAlignment(Qt.AlignCenter)
            lbl_rviz_fb.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_rviz_fb, row, 3)
            self._table_labels[axis]['rviz_feedback'] = lbl_rviz_fb

            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_err, row, 4)
            self._table_labels[axis]['error'] = lbl_err

            lbl_err_rviz = QLabel('N/A')
            lbl_err_rviz.setAlignment(Qt.AlignCenter)
            lbl_err_rviz.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_err_rviz, row, 5)
            self._table_labels[axis]['error_rviz'] = lbl_err_rviz"""
content = content.replace(old_table_labels, new_table_labels)

old_cart_labels = """            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            table_cart_layout.addWidget(lbl_err, row, 3)
            self._table_cart_labels[axis]['error'] = lbl_err"""
new_cart_labels = """            lbl_rviz_fb = QLabel('N/A')
            lbl_rviz_fb.setAlignment(Qt.AlignCenter)
            lbl_rviz_fb.setStyleSheet(f'color: {TEXT_SEC};')
            table_cart_layout.addWidget(lbl_rviz_fb, row, 3)
            self._table_cart_labels[axis]['rviz_feedback'] = lbl_rviz_fb

            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            table_cart_layout.addWidget(lbl_err, row, 4)
            self._table_cart_labels[axis]['error'] = lbl_err

            lbl_err_rviz = QLabel('N/A')
            lbl_err_rviz.setAlignment(Qt.AlignCenter)
            lbl_err_rviz.setStyleSheet(f'color: {TEXT_SEC};')
            table_cart_layout.addWidget(lbl_err_rviz, row, 5)
            self._table_cart_labels[axis]['error_rviz'] = lbl_err_rviz"""
content = content.replace(old_cart_labels, new_cart_labels)


# 6. Add Cartesian Banner
cart_banner = """        self._tabs.addTab(self._tab_cart, 'Control Cartesiano')"""
cart_banner_new = """        banner_cart = QLabel("Cartesiano/Mundo disponible para visualización RViz. TCP/IP cartesiano para KUKA real aún no habilitado.")
        banner_cart.setAlignment(Qt.AlignCenter)
        banner_cart.setStyleSheet(f'color: {WARN_CLR}; font-weight: bold; padding: 4px;')
        layout_cart.insertWidget(0, banner_cart)
        
        self._tabs.addTab(self._tab_cart, 'Control Cartesiano')"""
content = content.replace(cart_banner, cart_banner_new)

# 7. Update _refresh_table to populate RViz cols
old_refresh = """    def _refresh_table(self):
        # Update Axes table
        for axis in AXES:
            val = self._model.get_target(axis)
            self._table_labels[axis]['target'].setText(f'{val:.2f}')

            fb = self._model.get_feedback(axis)
            if fb is not None:
                self._table_labels[axis]['feedback'].setText(f'{fb:.2f}')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_labels[axis]['feedback'].setText('N/A')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_SEC};')

            err = self._model.get_error(axis)
            if err is not None:
                self._table_labels[axis]['error'].setText(f'{err:+.2f}')
                if abs(err) > 1.0:
                    self._table_labels[axis]['error'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_labels[axis]['error'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_labels[axis]['error'].setText('N/A')
                self._table_labels[axis]['error'].setStyleSheet(f'color: {TEXT_SEC};')

        # Update Cartesian table
        for axis in CARTESIAN_AXES:
            val = self._model.get_target(axis)
            self._table_cart_labels[axis]['target'].setText(f'{val:.2f}')

            fb = self._model.get_feedback(axis)
            if fb is not None:
                self._table_cart_labels[axis]['feedback'].setText(f'{fb:.2f}')
                self._table_cart_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_cart_labels[axis]['feedback'].setText('N/A')
                self._table_cart_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_SEC};')

            err = self._model.get_error(axis)
            if err is not None:
                self._table_cart_labels[axis]['error'].setText(f'{err:+.2f}')
                if abs(err) > 1.0:
                    self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_cart_labels[axis]['error'].setText('N/A')
                self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {TEXT_SEC};')"""

new_refresh = """    def _refresh_table(self):
        # Update Axes table
        for axis in AXES:
            val = self._model.get_target(axis)
            self._table_labels[axis]['target'].setText(f'{val:.2f}')

            fb = self._model.get_feedback(axis)
            if fb is not None:
                self._table_labels[axis]['feedback'].setText(f'{fb:.2f}')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_labels[axis]['feedback'].setText('N/A')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_SEC};')
                
            fb_rviz = self._model.get_rviz_feedback(axis)
            if fb_rviz is not None:
                self._table_labels[axis]['rviz_feedback'].setText(f'{fb_rviz:.2f}')
                self._table_labels[axis]['rviz_feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_labels[axis]['rviz_feedback'].setText('N/A')
                self._table_labels[axis]['rviz_feedback'].setStyleSheet(f'color: {TEXT_SEC};')

            err = self._model.get_error(axis)
            if err is not None:
                self._table_labels[axis]['error'].setText(f'{err:+.2f}')
                if abs(err) > 1.0:
                    self._table_labels[axis]['error'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_labels[axis]['error'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_labels[axis]['error'].setText('N/A')
                self._table_labels[axis]['error'].setStyleSheet(f'color: {TEXT_SEC};')
                
            err_rviz = self._model.get_rviz_error(axis)
            if err_rviz is not None:
                self._table_labels[axis]['error_rviz'].setText(f'{err_rviz:+.2f}')
                if abs(err_rviz) > 1.0:
                    self._table_labels[axis]['error_rviz'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_labels[axis]['error_rviz'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_labels[axis]['error_rviz'].setText('N/A')
                self._table_labels[axis]['error_rviz'].setStyleSheet(f'color: {TEXT_SEC};')

        # Update Cartesian table
        for axis in CARTESIAN_AXES:
            val = self._model.get_target(axis)
            self._table_cart_labels[axis]['target'].setText(f'{val:.2f}')

            fb = self._model.get_feedback(axis)
            if fb is not None:
                self._table_cart_labels[axis]['feedback'].setText(f'{fb:.2f}')
                self._table_cart_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_cart_labels[axis]['feedback'].setText('N/A')
                self._table_cart_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_SEC};')
                
            fb_rviz = self._model.get_rviz_cartesian_feedback(axis)
            if fb_rviz is not None:
                self._table_cart_labels[axis]['rviz_feedback'].setText(f'{fb_rviz:.2f}')
                self._table_cart_labels[axis]['rviz_feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_cart_labels[axis]['rviz_feedback'].setText('N/A')
                self._table_cart_labels[axis]['rviz_feedback'].setStyleSheet(f'color: {TEXT_SEC};')

            err = self._model.get_error(axis)
            if err is not None:
                self._table_cart_labels[axis]['error'].setText(f'{err:+.2f}')
                if abs(err) > 1.0:
                    self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_cart_labels[axis]['error'].setText('N/A')
                self._table_cart_labels[axis]['error'].setStyleSheet(f'color: {TEXT_SEC};')
                
            err_rviz = self._model.get_rviz_cartesian_error(axis)
            if err_rviz is not None:
                self._table_cart_labels[axis]['error_rviz'].setText(f'{err_rviz:+.2f}')
                if abs(err_rviz) > 1.0:
                    self._table_cart_labels[axis]['error_rviz'].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
                else:
                    self._table_cart_labels[axis]['error_rviz'].setStyleSheet(f'color: {ACCENT2};')
            else:
                self._table_cart_labels[axis]['error_rviz'].setText('N/A')
                self._table_cart_labels[axis]['error_rviz'].setStyleSheet(f'color: {TEXT_SEC};')"""
content = content.replace(old_refresh, new_refresh)

# 8. Update _validate_and_send logic
old_send = """    def _validate_and_send(self):
        if self._model.get_target_mode() == 'AxisTarget':
            target_json = self._model.build_target_json()
            self._bridge.publish_command(target_json)
            self._update_command_display(target_json)
        else:
            # En modo cartesiano aún no implementamos envío real a TCP/IP en bridge
            pass"""

new_send = """    def _validate_and_send(self):
        if self._model.get_target_mode() == 'AxisTarget':
            target_json = self._model.build_target_json()
            
            if self._model.publish_joints_to_kuka:
                self._kuka_bridge.publish_command(target_json)
                self._update_command_display(target_json)
                
            if self._model.publish_joints_to_rviz:
                arr = self._model.build_rviz_joint_array()
                self._rviz_bridge._pub_joints.publish(Float64MultiArray(data=arr))
        else:
            # Cartesiano
            if self._model.publish_cartesian_to_rviz:
                arr = self._model.build_cartesian_array()
                self._rviz_bridge._pub_cartesian.publish(Float64MultiArray(data=arr))
                
            if self._model.publish_cartesian_to_kuka:
                # KUKA TCP/IP cartesiano no implementado aún
                pass"""
content = content.replace(old_send, new_send)

# 9. Update closeEvent to handle both bridges
old_close = """        if self._bridge.is_running:
            self._bridge.stop()"""
new_close = """        if self._kuka_bridge.is_running:
            self._kuka_bridge.stop()"""
content = content.replace(old_close, new_close)

# 10. Add RViz slots
rviz_slots = """    @pyqtSlot(str)
    def _on_rviz_joint_state(self, data: str):
        import json
        try:
            fb = json.loads(data)
            self._model.update_rviz_feedback(fb)
            self._refresh_table()
        except json.JSONDecodeError:
            pass

    @pyqtSlot(str)
    def _on_rviz_cartesian_state(self, data: str):
        import json
        try:
            fb = json.loads(data)
            self._model.update_rviz_cartesian_feedback(fb)
            self._refresh_table()
        except json.JSONDecodeError:
            pass

    @pyqtSlot(str)
    def _on_rviz_status(self, status: str):
        self._model.update_moveit_status(status)
        # Se podría mostrar el estado de MoveIt en un label si hiciera falta
"""
content = content.replace("    def closeEvent(self, event):", rviz_slots + "\n    def closeEvent(self, event):")

with open(dest_file, 'w') as f:
    f.write(content)

print("SUCCESS")
