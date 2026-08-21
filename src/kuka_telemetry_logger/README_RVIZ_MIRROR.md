# KUKA RViz Mirror — `kuka_rviz_mirror`

Espejo de **VISUALIZACIÓN** de la posición articular **REAL** del KUKA en RViz.

> **ESTE NODO NO CONTROLA EL ROBOT.**
> No planifica, no ejecuta trayectorias, no abre sockets, no habla EthernetKRL
> y no envía ningún comando. Solo lee telemetría y publica un estado articular.

Si el KUKA se mueve por **JOG manual**, por un **programa KRL** o por
**XmlDualMove / ROS2**, mientras `AxisActual` siga llegando, RViz refleja ese
movimiento.

---

## 1. Arquitectura

```
KUKA REAL  (jog / KRL / XmlDualMove)
    |
    v
SPS.SUB + EthernetKRL
    |
    v
eki_axis_move                       (kuka_eki_bridge, SIN MODIFICAR)
    |
    v
/kuka/axis_move/feedback_json       (std_msgs/msg/String, payload JSON)
    |
    +---------------------------+
    |                           |
    v                           v
telemetry_logger            kuka_rviz_mirror        <-- NODO NUEVO
    |                           |
    v                           |  extraer A1..A6 [grados]
CSV + SQLite                    |  math.radians()
                                |  sensor_msgs/msg/JointState
                                v
                        /fake_joint_states
                                |
                                v
                    joint_state_publisher            (source_list, YA EXISTE)
                                |
                                v
                          /joint_states
                                |
                                v
                robot_state_publisher / MoveIt / RViz
```

Los dos suscriptores (`telemetry_logger` y `kuka_rviz_mirror`) reciben el mismo
`feedback_json` **en paralelo**. Varios suscriptores sobre un mismo tópico es lo
normal en ROS2 (DDS): ni la GUI ni el bridge se ven afectados.

---

## 2. INPUT

| | |
|---|---|
| **Tópico** | `/kuka/axis_move/feedback_json` |
| **Tipo** | `std_msgs/msg/String` (el campo `data` es un JSON) |
| **Publicador** | `eki_axis_move` (`kuka_eki_bridge/eki_axis_move_node.py`) |
| **QoS** | `RELIABLE` / `VOLATILE` / `KEEP_LAST(10)` |

Estructura del JSON, tal como la construye el bridge:

```json
{
  "seq": 245,
  "mode": "AxisTarget",
  "status": 1,
  "move_ready": false,
  "limits_ok": false,
  "delta_ok": false,
  "move_executed": false,
  "axis_actual":     {"A1": 0.0, "A2": -89.963005, "A3": 89.963928,
                      "A4": -0.004953, "A5": -0.006861, "A6": 0.000177},
  "position_actual": {"X": 525.294312, "Y": -1e-06, "Z": 890.001465,
                      "A": 38.800598, "B": 89.992378, "C": 38.800598},
  "rx_counter": 61,
  "bridge_safe_mode": false,
  "bridge_allow_motion": true
}
```

Claves usadas por este nodo:

| Dato | Ruta exacta en el JSON |
|---|---|
| A1 | `axis_actual.A1` |
| A2 | `axis_actual.A2` |
| A3 | `axis_actual.A3` |
| A4 | `axis_actual.A4` |
| A5 | `axis_actual.A5` |
| A6 | `axis_actual.A6` |
| Seq | `seq` (nivel raíz; proviene de `Robot/Seq`) |

El parsing **reutiliza** el módulo del logger existente
(`kuka_telemetry_logger/message_introspection.py`): se importan
`ros_message_to_dict`, `expand_json_payload`, `flatten` y `find_sequence`.
Nada de ese módulo fue modificado ni refactorizado.

---

## 3. DATOS Y CONVERSIÓN

Los valores `A1..A6` llegan desde el KUKA en **GRADOS** (`$AXIS_ACT`).
ROS `sensor_msgs/JointState` requiere **RADIANES**.

```python
a1_rad = math.radians(a1_deg)
...
a6_rad = math.radians(a6_deg)
```

**La transformación requerida es únicamente grados → radianes.**
No hay cambio de signo, no hay offset y no hay reordenamiento.

Motivo, verificado en el modelo actual: el URDF ya codifica los sentidos de giro
del KUKA en sus vectores `<axis>`
(`kuka_kr6_support/urdf/kr6r900sixx_macro.xacro`: `joint_a1` → `0 0 -1`,
`joint_a4` y `joint_a6` → `-1 0 0`). Por eso el puente ya existente
`kuka_gui_moveit_bridge` mapea `A[i] → joint_names[i]` con un `deg_to_rad()`
simple y nada más.

Mapeo índice a índice:

| KUKA | ROS joint |
|---|---|
| A1 | `joint_a1` |
| A2 | `joint_a2` |
| A3 | `joint_a3` |
| A4 | `joint_a4` |
| A5 | `joint_a5` |
| A6 | `joint_a6` |

Nombres tomados de:

* `kuka_kr6_support/urdf/kr6r900sixx_macro.xacro` (prefijo vacío en
  `kr6r900sixx.xacro`, por lo que los nombres no llevan namespace)
* `kuka_kr6_moveit_config/config/kuka_kr6.srdf`, grupo `manipulator`
* parámetro `joint_names` por defecto de `kuka_moveit_bridge_node.py`

---

## 4. OUTPUT

| | |
|---|---|
| **Tópico** | `/fake_joint_states` |
| **Tipo** | `sensor_msgs/msg/JointState` |
| **QoS** | `RELIABLE` / `VOLATILE` / `KEEP_LAST(10)` |
| `header.stamp` | `self.get_clock().now().to_msg()` |
| `name` | `[joint_a1, joint_a2, joint_a3, joint_a4, joint_a5, joint_a6]` |
| `position` | los seis valores en **radianes** |
| `velocity` | `[]` |
| `effort` | `[]` |

### Por qué NO se publica directamente en `/joint_states`

`/joint_states` **ya tiene un publicador**: el nodo `joint_state_publisher` que
arranca `kuka_kr6_moveit_config/launch/demo.launch.py` con
`source_list: ["/fake_joint_states"]`. Publicar una segunda fuente directamente
sobre `/joint_states` competiría con ese nodo.

Alimentar `/fake_joint_states` reutiliza la cadena que ya está cableada:

```
/fake_joint_states -> joint_state_publisher -> /joint_states -> RViz
```

### Requisito para que la imagen llegue a RViz

El visualizador debe estar corriendo el `joint_state_publisher` normal, es
decir `demo.launch.py` con **`use_gui:=false`**.

Con `use_gui:=true` (valor por defecto de ese launch) arranca
`joint_state_publisher_gui` **sin** `source_list`, y nadie consumiría
`/fake_joint_states`. Esto es solo una nota sobre cómo arrancar ese paquete:
no se modificó nada en él.

---

## 5. Comportamiento: event-driven 1:1

* **No hay `create_timer()`** en este nodo.
* Un `feedback_json` recibido produce **como máximo un** `JointState`.
* Si el KUKA entrega 7.68 Hz, el mirror publica ~7.68 Hz. No se asume 10 Hz.
* **Sin interpolación, sin spline, sin smoothing, sin filtro, sin low-pass, sin
  muestras generadas.** Espejo 1:1.

Nota honesta sobre el resto de la cadena (fuera de este nodo): el
`joint_state_publisher` de aguas abajo republica `/joint_states` con su propio
temporizador de 10 Hz (parámetro `rate`, valor por defecto 10). Ese
comportamiento es preexistente y no se modificó.

---

## 6. Timestamp

El mensaje de telemetría **no trae timestamp propio**: es un
`std_msgs/String` (sin `Header`) y el bloque `<SEND>` del KUKA no declara
ningún elemento de tiempo.

Por eso `JointState.header.stamp` es `self.get_clock().now().to_msg()`:

> Ese timestamp corresponde al **procesamiento / recepción en ROS2** de esta
> máquina. **NO corresponde al reloj del KUKA.** No se inventa ninguno.

## 7. Seq

`seq` se extrae con la misma función que usa el logger (`find_sequence`) y se
guarda como `last_seq` **solo para diagnóstico**. No se usa para mover, ni para
generar timestamps, ni para planificar.

---

## 8. Validación

Una muestra se **descarta sin publicar** si:

* el `data` no es un objeto JSON;
* falta el objeto `axis_actual`;
* falta cualquiera de `A1..A6`;
* algún valor no es numérico, o es `NaN`, `+Inf` o `-Inf`.

En ese caso se incrementa `invalid_messages` y se emite un warning
**limitado** (el primero, y luego uno cada 25) para no llenar `rosout`.

---

## 9. Diagnóstico

Estado interno: `messages_received`, `messages_published`, `invalid_messages`,
`last_seq`, `last_degrees`, `last_radians`.

La frecuencia de entrada se **mide** a partir del tiempo real entre callbacks;
nunca se asume. Cada 100 mensajes (`report_every`) imprime:

```
[KUKA RViz Mirror]
  Received:    100
  Published:   100
  Invalid:     0
  Input rate:  7.68 Hz (measured)
  Last Seq:    621
  A1..A6 deg:  [0.0000, -89.9630, 89.9639, -0.0050, -0.0069, 0.0002]
  A1..A6 rad:  [0.0000, -1.5702, 1.5702, -0.0001, -0.0001, 0.0000]
  Output:      /fake_joint_states
```

Por defecto **no** imprime cada mensaje. Con `verbose:=true` (o `--verbose`)
muestra cada conversión.

---

## 10. Parámetros

| Parámetro | Tipo | Por defecto |
|---|---|---|
| `telemetry_topic` | `str` | `/kuka/axis_move/feedback_json` |
| `joint_states_topic` | `str` | `/fake_joint_states` |
| `joint_names` | `str[]` | `[joint_a1 … joint_a6]` |
| `qos_depth` | `int` | `10` |
| `report_every` | `int` | `100` |
| `verbose` | `bool` | `false` |

> Nunca poner `joint_states_topic:=/joint_states`. Ese tópico ya tiene un
> publicador.

---

## 11. Auditoría de seguridad

| Elemento | Cantidad en este nodo |
|---|---|
| Sockets TCP | **0** |
| EthernetKRL / EKI | **0** |
| Comandos al robot | **0** |
| Publicadores `target_json` | **0** |
| Publicadores `joint_command_deg` | **0** |
| Publicadores `cartesian_command_deg` | **0** |
| `ActionClient` | **0** |
| MoveIt planning (`MoveGroup`, `MoveItCpp`, `PlanningComponent`, `plan()`, `execute()`) | **0** |
| `FollowJointTrajectory` / `trajectory_msgs` | **0** |
| Ejecución de trayectorias | **0** |
| **Publicadores totales** | **1 — solo `/fake_joint_states`** |

La posición medida es un **estado**, no un objetivo. RViz muestra dónde
**ESTÁ** el robot; aquí nadie planifica un camino para llegar hasta ahí.

---

## 12. Registro pendiente (NO aplicado)

Este nodo todavía **no está registrado** como ejecutable. Los dos cambios
siguientes son necesarios y están **documentados pero NO aplicados**, a la
espera de autorización.

### 12.1 `setup.py` — `console_scripts`

```diff
     entry_points={
         'console_scripts': [
             'telemetry_logger = '
             'kuka_telemetry_logger.telemetry_logger_node:main',
+            'kuka_rviz_mirror = '
+            'kuka_telemetry_logger.rviz_mirror_node:main',
         ],
     },
```

El launch nuevo **no** requiere cambios en `setup.py`: la línea
`('share/' + package_name + '/launch', glob('launch/*.launch.py'))` ya instala
`launch/kuka_rviz_mirror.launch.py` automáticamente.

### 12.2 `package.xml` — dependencia `sensor_msgs`

```diff
   <exec_depend>rclpy</exec_depend>
   <exec_depend>std_msgs</exec_depend>
+  <exec_depend>sensor_msgs</exec_depend>
   <exec_depend>rosidl_runtime_py</exec_depend>
   <exec_depend>launch</exec_depend>
   <exec_depend>launch_ros</exec_depend>
```

---

## 13. Uso

Tres terminales.

### TERMINAL 1 — visualizador y bridge (como siempre)

```bash
# visualizador (RViz + MoveIt), con el joint_state_publisher que consume
# /fake_joint_states:
ros2 launch kuka_kr6_moveit_config demo.launch.py use_gui:=false

# y por separado, la GUI TCP/IP ORIGINAL + eki_axis_move
# (NO la GUI dual para esta demo)
```

### TERMINAL 2 — mirror nuevo

```bash
ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py
```

### TERMINAL 3 — logger existente

```bash
ros2 run kuka_telemetry_logger telemetry_logger
```

### El mirror NO lee CSV

```
NO:   KUKA -> CSV -> RViz
SÍ:   KUKA -> feedback ROS -> rviz_mirror -> RViz
      KUKA -> feedback ROS -> logger      -> CSV
```

Las dos ramas son simultáneas e independientes.

---

## 14. Plan de pruebas (documentado, NO ejecutado)

### PRUEBA 1 — robot quieto

1. Levantar el visualizador.
2. Levantar el bridge TCP/IP.
3. Verificar que llega feedback:
   `ros2 topic hz /kuka/axis_move/feedback_json`
4. Arrancar el mirror.

**Esperado:** la posición en RViz coincide con la posición actual del KUKA.

### PRUEBA 2 — jog de A1

KUKA en **T1**. Mover A1 lentamente.

**Esperado:**

```
KUKA A1 -> AxisActual A1 -> feedback_json -> math.radians(A1)
        -> /fake_joint_states -> /joint_states -> RViz
```

No debe existir planning en ningún punto.

### PRUEBA 3 — A1…A6

Mover cada articulación individualmente y confirmar:

* que se mueve la joint correcta;
* el sentido de giro;
* la amplitud;
* la posición de home;
* la ausencia de offsets incorrectos.

### PRUEBA 4 — programa KRL (DEMO PRINCIPAL)

Ejecutar un programa KRL.

**Esperado:** el robot físico ejecuta normalmente; mientras tanto
`feedback_json` sigue llegando y `rviz_mirror` refleja el movimiento en tiempo
aproximadamente real, a la frecuencia de la telemetría.

### PRUEBA 5 — logger + mirror simultáneos

Ejecutar `telemetry_logger` y `kuka_rviz_mirror` a la vez. Ambos se suscriben al
mismo feedback.

**Esperado:** el CSV/SQLite sigue registrando y RViz sigue actualizándose. La
frecuencia de comunicación con el KUKA no cambia de forma relevante, porque
ambos operan **después** del bridge ROS2.

---

## 15. Archivos

**Creados (nuevos):**

* `kuka_telemetry_logger/rviz_mirror_node.py`
* `launch/kuka_rviz_mirror.launch.py`
* `README_RVIZ_MIRROR.md` (este archivo)

**Modificados:** ninguno.
