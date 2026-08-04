import re
import sys

def main():
    path = '/home/eduardex/Documents/TG2/src/kuka_gui_control/kuka_gui_control/gui_axis_move_window.py'
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
        
    # 1. Update imports
    content = content.replace(
        'QStackedWidget, QTextEdit, QVBoxLayout, QWidget,',
        'QStackedWidget, QTextEdit, QVBoxLayout, QWidget,\n        QTabWidget, QRadioButton, QButtonGroup,'
    )
    content = content.replace(
        'from kuka_gui_control.joint_command_model import JointCommandModel, AXES',
        'from kuka_gui_control.joint_command_model import JointCommandModel, AXES, CARTESIAN_AXES'
    )
    
    # 2. Add mode layout and tabs
    ui_insertion = """
        # ── Mode Selector ────────────────────────────────────────────
        mode_layout = QHBoxLayout()
        lbl_mode = QLabel('Modo de Control:')
        lbl_mode.setStyleSheet('font-weight: bold;')
        mode_layout.addWidget(lbl_mode)
        
        self._radio_axis = QRadioButton('Ejes (A1-A6)')
        self._radio_axis.setChecked(True)
        self._radio_axis.toggled.connect(self._on_target_mode_changed)
        mode_layout.addWidget(self._radio_axis)
        
        self._radio_cartesian = QRadioButton('Cartesiano (X,Y,Z,A,B,C)')
        mode_layout.addWidget(self._radio_cartesian)
        mode_layout.addStretch()
        
        main_layout.addLayout(mode_layout)

        # ── Tabs ─────────────────────────────────────────────────────
        self._tabs = QTabWidget()
        
        self._tab_axis = QWidget()
        layout_axis = QVBoxLayout(self._tab_axis)
        
        self._tab_cart = QWidget()
        layout_cart = QVBoxLayout(self._tab_cart)
        
        self._tabs.addTab(self._tab_axis, 'Control por Ejes')
        self._tabs.addTab(self._tab_cart, 'Control Cartesiano')
        
        main_layout.addWidget(self._tabs)
"""
    # Find the line before joint table
    search_str = "        # ── Joint table ──────────────────────────────────────────────"
    content = content.replace(search_str, ui_insertion + "\n" + search_str)
    
    # Replace main_layout.addWidget(table_group) with layout_axis.addWidget(table_group)
    content = content.replace(
        "        main_layout.addWidget(table_group)",
        "        layout_axis.addWidget(table_group)"
    )
    
    # Replace main_layout.addWidget(controls_group) with layout_axis.addWidget(controls_group)
    content = content.replace(
        "        main_layout.addWidget(controls_group)",
        "        layout_axis.addWidget(controls_group)"
    )
    
    # Add cartesian tables and controls
    cart_insertion = """
        # ── Cartesian table ──────────────────────────────────────────────
        cart_table_group = QGroupBox('Posiciones Cartesianas')
        cart_table_layout = QGridLayout(cart_table_group)
        cart_table_layout.setSpacing(4)

        for col, hdr in enumerate(['Cartesiano', 'Target', 'Feedback', 'Error']):
            lbl = QLabel(hdr)
            lbl.setStyleSheet(f'font-weight: bold; color: {ACCENT}; font-size: 12px;')
            lbl.setAlignment(Qt.AlignCenter)
            cart_table_layout.addWidget(lbl, 0, col)

        for row, axis in enumerate(CARTESIAN_AXES, start=1):
            self._table_labels[axis] = {}
            lbl_name = QLabel(axis)
            lbl_name.setAlignment(Qt.AlignCenter)
            lbl_name.setStyleSheet('font-weight: bold;')
            cart_table_layout.addWidget(lbl_name, row, 0)

            lbl_target = QLabel('0.00')
            lbl_target.setAlignment(Qt.AlignCenter)
            cart_table_layout.addWidget(lbl_target, row, 1)
            self._table_labels[axis]['target'] = lbl_target

            lbl_fb = QLabel('N/A')
            lbl_fb.setAlignment(Qt.AlignCenter)
            lbl_fb.setStyleSheet(f'color: {TEXT_SEC};')
            cart_table_layout.addWidget(lbl_fb, row, 2)
            self._table_labels[axis]['feedback'] = lbl_fb

            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            cart_table_layout.addWidget(lbl_err, row, 3)
            self._table_labels[axis]['error'] = lbl_err

        layout_cart.addWidget(cart_table_group)

        # ── Cartesian controls ───────────────────────────────────────────
        cart_controls_group = QGroupBox('Control Cartesiano')
        cart_controls_layout = QGridLayout(cart_controls_group)
        cart_controls_layout.setSpacing(4)

        for row, axis in enumerate(CARTESIAN_AXES):
            lbl = QLabel(axis)
            lbl.setStyleSheet('font-weight: bold; min-width: 30px;')
            cart_controls_layout.addWidget(lbl, row, 0)

            btn_minus = QPushButton('−')
            btn_minus.setStyleSheet(BTN_PM)
            btn_minus.setCursor(Qt.PointingHandCursor)
            btn_minus.clicked.connect(lambda checked, a=axis: self._on_step(a, -1))
            cart_controls_layout.addWidget(btn_minus, row, 1)
            self._joint_minus[axis] = btn_minus

            inp = QLineEdit(f'{self._model.get_target(axis):.2f}')
            inp.setAlignment(Qt.AlignCenter)
            inp.setFixedWidth(100)
            inp.editingFinished.connect(lambda a=axis: self._on_input_changed(a))
            cart_controls_layout.addWidget(inp, row, 2)
            self._joint_inputs[axis] = inp

            btn_plus = QPushButton('+')
            btn_plus.setStyleSheet(BTN_PM)
            btn_plus.setCursor(Qt.PointingHandCursor)
            btn_plus.clicked.connect(lambda checked, a=axis: self._on_step(a, +1))
            cart_controls_layout.addWidget(btn_plus, row, 3)
            self._joint_plus[axis] = btn_plus

            lbl_deg = QLabel('mm' if axis in ['X', 'Y', 'Z'] else 'deg')
            lbl_deg.setStyleSheet(f'color: {TEXT_SEC};')
            cart_controls_layout.addWidget(lbl_deg, row, 4)

        layout_cart.addWidget(cart_controls_group)
"""
    content = content.replace(
        "        layout_axis.addWidget(controls_group)",
        "        layout_axis.addWidget(controls_group)\n" + cart_insertion
    )

    # In _refresh_table, replace AXES with (AXES + CARTESIAN_AXES)
    content = content.replace(
        "for axis in AXES:",
        "for axis in AXES + CARTESIAN_AXES:"
    )
    
    # In _on_target_mode_changed
    mode_fn = """
    def _on_target_mode_changed(self):
        if self._radio_axis.isChecked():
            self._model.set_target_mode('AxisTarget')
            self._tabs.setCurrentWidget(self._tab_axis)
        else:
            self._model.set_target_mode('CartesianTarget')
            self._tabs.setCurrentWidget(self._tab_cart)
        self._refresh_inputs()
        
    def _refresh_table(self):"""
    content = content.replace("    def _refresh_table(self):", mode_fn)

    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)
        
    print("Updated GUI successfully")

if __name__ == '__main__':
    main()
