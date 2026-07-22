# kuka_gui_control

GUI ROS2 basada en **PyQt5** para enviar objetivos articulares al robot KUKA mediante el modo `axis_command_loop` del paquete `kuka_eki_bridge`.

---

## 1. Objetivo

Proporcionar una interfaz gráfica para:

- Definir posiciones articulares objetivo (A1–A6) de forma interactiva.
- Publicarlas continuamente o bajo demanda en un topic ROS2.
- Visualizar en tiempo real el feedback del KUKA (posición actual y error).

```
GUI → ROS2 topic → kuka_eki_bridge → TCP/XML → KUKA
```

La GUI **no abre sockets TCP** ni se comunica directamente con EthernetKRL.
Toda la lógica de red queda en `kuka_eki_bridge`.

---

## 2. Arquitectura

```
┌─────────────────────────────────────────────────────────┐
│                  kuka_gui_control (GUI)                  │
│                                                         │
│   PyQt5 Window ◀──────── Qt Signals ──────────────────┐ │
│                                                       │ │
│   RosGuiBridge (hilo background)                      │ │
│     ├── Publica en /kuka/axis_command/target_json     │ │
│     └── Se suscribe a /kuka/axis_command_loop/...     │ │
└─────────────────────────────────────────────────────────┘
         │                          ▲
         │  ROS2 topic              │  ROS2 topic
         ▼                          │
┌────────────────────────────────────────────────────────┐
│             kuka_eki_bridge                            │
│                                                        │
│   eki_axis_command_loop_node (puerto 59153)            │
│     ├── Recibe XML <Robot> del KUKA                    │
│     └── Responde XML <Command> con AxisTarget          │
└────────────────────────────────────────────────────────┘
         │                          ▲
         │  TCP/XML                 │  TCP/XML
         ▼                          │
      KUKA Controller (XmlAxisCommandLoop.src)
```

---

## 3. Dependencias

### Sistema

```bash
sudo apt install python3-pyqt5
```

### ROS2

```
rclpy
std_msgs
```

---

## 4. Compilar

```bash
cd ~/Documents/TG2
colcon build --packages-select kuka_eki_bridge kuka_gui_control
source install/setup.bash
```

---

## 5. Ejecutar

### Solo el command loop (sin GUI)

```bash
ros2 launch kuka_eki_bridge axis_command_loop.launch.py
```

### Solo la GUI (command loop ya activo en otra terminal)

```bash
ros2 launch kuka_gui_control gui_control.launch.py
```

### GUI + command loop juntos

```bash
ros2 launch kuka_gui_control gui_with_command_loop.launch.py
```

---

## 6. Verificar topics

```bash
# Ver el JSON que la GUI publica al bridge
ros2 topic echo /kuka/axis_command/target_json

# Ver el feedback JSON que llega del KUKA
ros2 topic echo /kuka/axis_command_loop/feedback_json

# Ver el XML de comando enviado al KUKA
ros2 topic echo /kuka/axis_command_loop/raw_command_xml
```

---

## 7. Descripción de botones

| Botón | Función |
|---|---|
| **START** | Cambia de la pantalla de bienvenida a la pantalla de control. No publica nada. |
| **HOME** | Carga la posición home (A1=0, A2=-90, A3=90, A4=0, A5=0, A6=0). En modo manual envía una vez. |
| **SEND** | Publica el target actual una sola vez en modo `manual_send`. Bloqueado si hay valores OOL. |
| **MODO AUTOMÁTICO** | Alterna publicación continua a 5 Hz. Muestra ON/OFF en el botón. |
| **STOP AUTO** | Desactiva el modo automático sin cerrar la GUI ni enviar comandos. |
| **RESET GUI** | Carga valores home sin enviar (a menos que automático esté activo). |
| **+** / **−** | Incrementa o decrementa el joint en `step_deg` (por defecto 1°). |
| Campo editable | Permite escribir un valor numérico exacto con decimales. |

---

## 8. Posición HOME

```
A1 =   0.0 °
A2 = -90.0 °
A3 =  90.0 °
A4 =   0.0 °
A5 =   0.0 °
A6 =   0.0 °
```

---

## 9. Soft limits (límites suaves)

Los soft limits definen el rango permitido por joint.
Si un valor está fuera de rango:

- El campo se muestra en **rojo**.
- La columna Estado muestra `!! OOL` (Out Of Limits).
- El botón **SEND** se deshabilita.
- El target no se publica.

| Joint | Mínimo (°) | Máximo (°) |
|---|---|---|
| A1 | -20.0 | 20.0 |
| A2 | -110.0 | -70.0 |
| A3 | 70.0 | 110.0 |
| A4 | -20.0 | 20.0 |
| A5 | -20.0 | 20.0 |
| A6 | -20.0 | 20.0 |

---

## 10. Tabla de posiciones: Target · Feedback · Error

| Columna | Descripción |
|---|---|
| **Target (°)** | Posición articular objetivo enviada por la GUI. |
| **Feedback (°)** | Posición articular actual reportada por el KUKA. |
| **Error (°)** | `Target − Feedback`. Verde si ≤ 1°, rojo si > 1°. |

Si no hay feedback reciente, las columnas Feedback y Error muestran **N/A**.

El estado de conexión KUKA cambia a **sin feedback** si no llega ningún mensaje en `feedback_timeout_sec` segundos (por defecto 2.0 s).

---

## 11. Formato JSON publicado por la GUI

```json
{
  "seq": 1,
  "source": "kuka_gui_control",
  "mode": "manual_send",
  "enable_move": true,
  "A1": 0.0,
  "A2": -90.0,
  "A3": 90.0,
  "A4": 0.0,
  "A5": 0.0,
  "A6": 0.0
}
```

En modo automático, `mode` es `"auto"`.

---

## 12. ⚠️ Advertencia de seguridad

> Esta GUI **solo envía objetivos articulares**.
>
> El movimiento real del KUKA debe estar validado en el lado del
> controlador, bajo supervisión, con límites de hardware habilitados
> y **sin desactivar ninguna seguridad del robot**.
>
> `EnableMove` siempre se envía como `0` cuando `safe_mode=true`
> en `axis_command_loop.yaml`. Esto impide que el KUKA ejecute
> movimientos aunque la GUI lo solicite.
>
> El movimiento real con PTP/LIN se habilitará en una etapa posterior,
> solo después de verificar el programa KRL y las seguridades del controlador.

---

## Licencia

MIT
