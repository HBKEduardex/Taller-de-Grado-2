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

## 13. Secuencias de Trayectorias (SET → MoveIt2 → `trajectories/` → KUKA)

Capa **AÑADIDA**. No sustituye ni modifica nada de lo anterior: HOME, SEND,
AUTO, jog articular, cartesiano, los botones de garra y los límites siguen
comportándose exactamente igual.

Disponible en **las dos GUIs**:

- GUI original TCP/IP (`gui_axis_move_node`)
- GUI dual KUKA + RViz (`gui_dual_node`)

Ambas usan el **mismo widget compartido** (`trajectory_panel.py`), así que la
lógica de secuencia no está duplicada en dos archivos.

### 13.1 Flujo conceptual

```text
KUKA real
 -> Submit Interpreter (SPS.SUB) / TCP-IP
 -> AxisActual
 -> SET P1...PN
 -> ENVIAR PUNTOS
 -> ROS2
 -> contenedor MoveIt2
 -> T1, T2, T3...
 -> GUI
 -> trajectories/*.json
```

Y después, a partir del archivo guardado:

```text
trajectories/*.json
 -> PROBAR TRAYECTORIA
 -> RViz únicamente
```

o bien:

```text
trajectories/*.json
 -> ENVIAR TRAYECTORIA
 -> bridge TCP/IP existente (RosAxisMoveBridge -> eki_axis_move_node)
 -> KUKA real
```

### 13.2 Controles del panel

```text
SET   Puntos: N   SET ABRIR GARRA   SET CERRAR GARRA   LIMPIAR
ENVIAR PUNTOS (N)   PROBAR TRAYECTORIA   ENVIAR TRAYECTORIA   DETENER
(•) Manual   ( ) Automático
estado + log temporal
```

| Control | Qué hace | Qué NO hace |
|---|---|---|
| **SET** | Captura `AxisActual` A1–A6 real del KUKA como P1, P2, … | No lee los campos de la GUI, ni el target, ni RViz, ni XYZABC. **No escribe en disco.** |
| **Puntos: N** | Cuenta solo P1…PN | Los eventos de garra no incrementan el contador |
| **SET ABRIR GARRA** | Programa `{"at_point": "Pn", "action": "open"}` | **No mueve la garra** |
| **SET CERRAR GARRA** | Programa `{"at_point": "Pn", "action": "close"}` | **No mueve la garra** |
| **LIMPIAR** | Vacía el buffer temporal | — |
| **ENVIAR PUNTOS (N)** | Publica UNA solicitud con todos los puntos y eventos | No manda punto por punto; exige ≥ 2 puntos |
| **PROBAR TRAYECTORIA** | Reproduce un archivo en RViz | **Nunca alcanza al KUKA.** No activa EnableMove, no llama al sender TCP/IP, no mueve la garra real |
| **ENVIAR TRAYECTORIA** | Ejecuta físicamente un archivo ya guardado | No genera nada nuevo, no vuelve a llamar a MoveIt |
| **DETENER** | Aborta la secuencia en curso | — |
| **Manual / Automático** | Selector mutuamente excluyente (`QButtonGroup`) | Nunca pueden quedar ambos activos. **Por defecto: Manual** |

### 13.3 SET — captura de `AxisActual` por TCP/IP

La fuente autoritativa es la que ya publica el bridge:

```text
KUKA  --XML--> eki_axis_move_node  --/kuka/axis_move/feedback_json-->  GUI
                                        └── "axis_actual": {A1..A6}
```

El panel se suscribe a la señal `feedback_received` del `RosAxisMoveBridge`
**que ya existía** (una conexión más a la misma señal; los slots actuales no
se han tocado) y guarda el `axis_actual` crudo.

Al pulsar SET:

```text
P1 = [A1,A2,A3,A4,A5,A6]
P2 = [...]
P3 = [...]
```

Se **rechaza** la captura, con mensaje claro en pantalla, si:

- no hay feedback reciente (se reutiliza `has_recent_feedback()` del modelo
  junto con el sello de tiempo propio del panel, `feedback_timeout_sec`);
- falta alguno de A1…A6;
- algún valor no es finito (NaN o infinito).

`XYZABC` se guarda **solo como diagnóstico opcional**, y únicamente si el
bloque entero es finito. Nunca se usa para generar ni ejecutar la
trayectoria, por los problemas conocidos de recálculo/NaN.

**Persistencia:** ninguna. P1…PN viven solo en memoria mientras la
aplicación está abierta. No se escribe nada en disco hasta que MoveIt
devuelve una trayectoria válida.

### 13.4 Eventos de garra

Estado inicial de toda secuencia: `initial_state = "open"`. Si el usuario no
registra ningún evento, la garra permanece abierta.

La acción queda asociada al **último punto SET** registrado:

```text
SET P1
SET P2
SET CERRAR GARRA     ->  { "at_point": "P2", "action": "close" }
SET P3
SET P4
SET ABRIR GARRA      ->  { "at_point": "P4", "action": "open"  }
```

Significado: llegar a P2 → cerrar garra → continuar con P3.

Pulsar estos botones durante el seteo **no mueve la garra física**. No
publican nada por el bridge ni tocan `request_gripper_command()`. La
ejecución física ocurre solo durante ENVIAR TRAYECTORIA.

### 13.5 Log temporal

Área compacta de solo lectura dentro del panel:

```text
P1 seteado: A1=0.00 A2=-90.00 A3=90.00 A4=0.00 A5=90.00 A6=0.00
P2 seteado: A1=5.00 A2=-85.00 A3=85.00 A4=0.00 A5=90.00 A6=0.00
Garra: CERRAR en P2
P3 seteado: ...
```

### 13.6 Contrato ROS2 — tópicos exactos

Todos `std_msgs/msg/String`, contenido JSON, **QoS Reliable**
(`RELIABLE` / `KEEP_LAST` / depth 10 / `VOLATILE`).

| Dirección | Tópico |
|---|---|
| GUI → MoveIt | `/kuka_moveit/trajectory_generation/request_json` |
| MoveIt → GUI | `/kuka_moveit/trajectory_generation/result_json` |
| GUI → RViz preview | `/kuka_moveit/trajectory_preview/request_json` |
| MoveIt → GUI | `/kuka_moveit/trajectory_preview/status_json` |

> Estos cuatro publishers/subscribers **viven en el nodo ROS2 que ya
> existía** (el del `RosAxisMoveBridge`, dentro de su executor). No se crea
> ningún nodo nuevo, ni uno por ventana. No hay nodos homónimos ni
> `Publisher count: 2`.
>
> Como en la GUI original el nodo `rclpy` nace *después* de construirse la
> ventana, el enganche es diferido: `TrajectoryRosBridge.ensure_attached()`
> se reintenta desde un `QTimer` hasta que el nodo existe, y entonces el
> timer se para.

### 13.7 ENVIAR PUNTOS — solicitud de generación

Exige **al menos 2 puntos**. Se manda **UNA sola solicitud** con todos ellos.

```json
{
  "schema_version": 1,
  "request_id": "uuid",
  "joint_names": ["joint_a1","joint_a2","joint_a3","joint_a4","joint_a5","joint_a6"],
  "points": [
    { "id": "P1", "joints_deg": [0.0, -90.0, 90.0, 0.0, 90.0, 0.0] },
    { "id": "P2", "joints_deg": [5.0, -85.0, 85.0, 0.0, 90.0, 0.0] }
  ],
  "gripper": {
    "initial_state": "open",
    "events": [ { "at_point": "P2", "action": "close" } ]
  },
  "planner": { "mode": "moveit_base", "execute": false }
}
```

`planner.execute` es siempre `false`: la GUI nunca pide ejecución al
contenedor. La ejecución física es cosa de ENVIAR TRAYECTORIA.

### 13.8 Recepción del resultado

MoveIt devuelve un segmento por tramo entre puntos SET:

```text
T1 = P1 -> P2
T2 = P2 -> P3
T3 = P3 -> P4
```

Cada segmento trae todos sus puntos intermedios.

La respuesta se **correlaciona por `request_id`**: una respuesta de otra
solicitud se ignora, nunca se da por válida.

Con `status = "ok"` se valida, en este orden:

1. `schema_version`
2. `request_id`
3. `joint_names`
4. segmentos presentes y no vacíos
5. 6 articulaciones por punto
6. todos los números finitos (posiciones, velocidades, aceleraciones)
7. `time_from_start_sec` presente, finito, no negativo y no decreciente

Y se conservan **exactamente** los datos recibidos.

Con `status = "error"`:

- la secuencia **NO se guarda** como ejecutable;
- se muestran `failed_segment` y el motivo.

Formato esperado de la respuesta:

```json
{
  "schema_version": 1,
  "request_id": "uuid",
  "status": "ok",
  "joint_names": ["joint_a1", "..."],
  "planner_metadata": { "planner": "RRTConnect", "velocity_scaling": 0.1 },
  "segments": [
    {
      "id": "T1",
      "from_point": "P1",
      "to_point": "P2",
      "duration_sec": 2.0,
      "trajectory_points": [
        {
          "positions_rad": [0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0],
          "positions_deg": [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
          "velocities_rad_s": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          "accelerations_rad_s2": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          "time_from_start_sec": 0.0
        }
      ]
    }
  ]
}
```

`positions_deg` es opcional: si solo llega `positions_rad`, la GUI deriva los
grados por **conversión de unidades** y lo marca en el archivo con
`positions_deg_source: "converted_from_rad"`. Eso no es transformar la
trayectoria: no se eliminan puntos, no se reinterpola, no se tocan los
tiempos y no se suaviza nada.

En caso de error el contenedor debe responder:

```json
{
  "schema_version": 1,
  "request_id": "uuid",
  "status": "error",
  "failed_segment": "T2",
  "error": "IK unreachable for P3"
}
```

### 13.9 Carpeta `trajectories/`

El **contenedor no guarda el archivo final: lo guarda esta GUI.**

Orden de resolución de la ruta:

1. `trajectories_dir` del YAML de la GUI (si no está vacío);
2. variable de entorno `KUKA_TRAJECTORIES_DIR`;
3. raíz del repositorio detectada en runtime → `<raíz>/trajectories`;
4. último recurso: `~/kuka_trajectories`.

La detección salta deliberadamente cualquier ruta bajo `install/`: los
archivos generados **nunca** se guardan ahí. Con este checkout la ruta final
es:

```text
/home/eduardex/Documents/TG2/trajectories
```

Formato: **JSON**, no YAML. Un archivo completo por secuencia:

```text
trajectory_sequence_20260822_153501.json
```

La escritura es atómica (archivo temporal + `os.replace`), de modo que un
fallo a mitad no deja un `.json` corrupto que después parecería ejecutable.

Contenido del archivo:

```jsonc
{
  "schema_version": 1,
  "request_id": "…",
  "generated_at": "2026-08-22T15:35:01",
  "generated_at_date": "2026-08-22",
  "generated_at_time": "15:35:01",
  "source": "kuka_gui_control",
  "source_points": [ { "id": "P1", "joints_deg": [...], "captured_at": "…" } ],
  "joint_names": [ "joint_a1", "…" ],
  "gripper": { "initial_state": "open", "events": [ … ] },
  "planner_metadata": { … },
  "segments": [
    {
      "id": "T1", "from_point": "P1", "to_point": "P2", "duration_sec": 2.0,
      "trajectory_points": [
        {
          "positions_rad":        [...],
          "positions_deg":        [...],
          "positions_deg_source": "received",
          "velocities_rad_s":     [...],
          "accelerations_rad_s2": [...],
          "time_from_start_sec":  0.0
        }
      ]
    }
  ],
  "summary": {
    "num_source_points": 4,
    "num_gripper_events": 2,
    "num_segments": 3,
    "num_trajectory_points": 128,
    "total_duration_sec": 9.4
  }
}
```

Tras guardar correctamente, **ambas GUIs** muestran:

```text
Trayectorias generadas y guardadas en:
/home/eduardex/Documents/TG2/trajectories/trajectory_sequence_20260822_153501.json

Segmentos: 3
Puntos totales de trayectoria: 128
```

Si la escritura falla se informa del error y **no** se dice que fue guardado.

### 13.10 PROBAR TRAYECTORIA — solo RViz

- Usa por defecto el último archivo generado correctamente en la sesión.
- Si no hay ninguno, abre un selector de archivos limitado a `trajectories/`.
- Publica el contenido en `/kuka_moveit/trajectory_preview/request_json`.
- Escucha `/kuka_moveit/trajectory_preview/status_json` y muestra
  `Previsualizando trayectoria en RViz...` y después
  `Previsualización finalizada.`

**Garantía estructural:** el método de previsualización no tiene acceso al
bridge de movimiento. `TrajectoryRosBridge` solo conoce los cuatro tópicos de
trayectoria; no conoce el tópico de comandos del KUKA, ni
`publish_command()`, ni `EnableMove`, ni la garra. No hay ningún camino de
código desde PROBAR TRAYECTORIA hasta el robot.

Solicitud publicada:

```json
{
  "schema_version": 1,
  "preview_id": "uuid",
  "request_id": "uuid de la secuencia",
  "source_file": "/…/trajectory_sequence_….json",
  "trajectory": { …documento completo… }
}
```

Estado esperado de vuelta:

```json
{ "schema_version": 1, "preview_id": "uuid",
  "status": "started|running|finished|error", "message": "" }
```

### 13.11 ENVIAR TRAYECTORIA — ejecución física

1. Abre `QFileDialog` sobre `trajectories/`, filtrado a `.json`.
2. Valida el archivo (mismo criterio que al recibirlo de MoveIt).
3. Comprueba las protecciones existentes (§ 13.13).
4. Hace un *preflight* contra la posición REAL del robot.
5. Pide confirmación explícita.
6. Carga `segments` y los ejecuta **en orden**.

No genera nada nuevo y no vuelve a llamar a MoveIt: se ejecuta exactamente
el archivo guardado, usando `positions_deg` para el comando físico.

**Preflight** — rechaza antes de mover si:

- no hay `AxisActual` con el que comparar;
- algún punto viola los soft limits del modelo;
- algún salto entre puntos consecutivos supera `MAX_DELTA_JOINT` (10°);
- el robot está a más de 10° del primer punto del primer segmento (en ese
  caso pide acercar el robot al primer punto antes de ejecutar).

**Ruta de envío** — la misma que el botón SEND, sin XML paralelo:

```text
TrajectoryExecutor
  -> model.set_target(A1..A6)          (JointCommandModel / DualCommandModel)
  -> función de envío de la ventana    (la misma que usa SEND)
  -> RosAxisMoveBridge.publish_command()
  -> /kuka/axis_move/target_json
  -> eki_axis_move_node
  -> build_axis_move_command_xml()
  -> TCP/XML -> KUKA
```

**Pacing** — un punto cada vez. Un punto se da por hecho solo cuando se
cumplen las tres cosas:

1. ha pasado el suelo `trajectory_min_point_period_sec` (0.2 s);
2. el KUKA ha **acusado recibo**: `Robot/RxCounter` —la cuenta del propio SPS
   de comandos completos sacados de la memoria de recepción de EKI, la misma
   que usa el guard del bridge— ha subido desde el envío. Si el controlador
   todavía no publica `RxCounter`, el respaldo es haber recibido al menos una
   trama de telemetría posterior;
3. `AxisActual` está dentro de `trajectory_arrival_tolerance_deg` (0.5°) del
   objetivo en las seis articulaciones.

Mientras no se confirma, el mismo objetivo se reenvía cada
`trajectory_resend_period_sec` (0.5 s), por debajo del `command_timeout_sec`
de 2.0 s del bridge — es exactamente el mecanismo de *hold* que ya usa SEND.
Si un punto no se alcanza en `trajectory_point_timeout_sec` (15 s) se aborta
con mensaje claro.

**La GUI no se bloquea:** la ejecución es una máquina de estados dirigida por
un `QTimer` de 100 ms y por el feedback que ya llega del KUKA. No hay sleeps
ni hilos nuevos, igual que AUTO y el hold de SEND. La GUI sigue respondiendo
mientras se carga el archivo, se ejecuta, se espera feedback y se cambia de
segmento.

### 13.12 Modo Manual y modo Automático

Selector mutuamente excluyente (`QButtonGroup` exclusivo). **Por seguridad,
el valor por defecto es Manual.**

**Manual** — no se detiene en cada punto intermedio de MoveIt, se detiene al
terminar cada **SEGMENTO** entre puntos SET:

```text
Ejecutar TODOS los puntos intermedios de T1
Llegar realmente a P2
Ejecutar el evento de garra asociado a P2
Detener la secuencia
```

y aparece el diálogo:

```text
Trayectoria T1 completada.
Robot en P2.

¿Continuar con la trayectoria T2: P2 -> P3?

[ CONTINUAR ]   [ CANCELAR ]
```

CONTINUAR ejecuta T2. CANCELAR no envía nada más. Se repite después de cada
segmento. Nunca se pregunta entre `trajectory_point`s.

**Automático** — encadena `T1 → garra → T2 → garra → T3 …` sin pedir
confirmación, mostrando `Ejecutando T1 (1/4) — N puntos`, etc. Ante un error
se **aborta** la secuencia; no se continúa en silencio.

### 13.13 Garra durante la ejecución y seguridad

`initial_state = open`. Los eventos se ejecutan exactamente al llegar al
punto asociado:

```text
Ejecutar T1
Confirmar llegada a P2
Cerrar garra  (con LA IMPLEMENTACIÓN ACTUAL)
Esperar el asentamiento
Continuar con T2
```

Un evento anclado al primer punto (P1) se ejecuta antes de T1, porque ningún
segmento termina en él.

La garra **no se reimplementa**: el ejecutor llama a la misma función de la
ventana que usan los botones «Abrir garra» / «Cerrar garra», que publica
`GripperCommand` 0/1 con `EnableMove=false` por el mismo bridge. El
`joint_command_model` y el protocolo no se han cambiado.

> El protocolo actual **no publica estado de garra**, así que tras ordenar la
> acción se respeta `trajectory_gripper_settle_sec` (2 s) en lugar de
> inventar una confirmación que no llega. El comando se repite durante esa
> espera; `GRPg_SetStateAndCheck` es idempotente.

**Protecciones respetadas — sin ningún bypass.** ENVIAR TRAYECTORIA se niega
a arrancar, sin enviar un solo comando, si:

| Protección | Dónde vive | Cómo se comprueba |
|---|---|---|
| `safe_mode` | `eki_axis_move_node` (`axis_move.yaml`) | `bridge_safe_mode` del feedback |
| `allow_motion_commands` | `eki_axis_move_node` | `bridge_allow_motion` del feedback |
| `ENABLE MOVE` | checkbox de la GUI | `model.get_enable_move()` |
| Feedback vivo | bridge TCP/IP | `has_recent_feedback()` |

Además, aunque la GUI se equivocara, el bridge sigue forzando `EnableMove=0`
con `safe_mode=true` o `allow_motion_commands=false`, y el KRL sigue
validando límites y delta por su cuenta. PROBAR TRAYECTORIA funciona siempre,
porque no puede provocar movimiento físico.

### 13.14 Manejo de errores

| Situación | Comportamiento |
|---|---|
| SET sin feedback válido | No guarda el punto. Mensaje claro. |
| SET con A1…A6 no finito | No guarda el punto. Mensaje claro. |
| SET garra sin ningún punto | No registra el evento. Mensaje claro. |
| ENVIAR PUNTOS con < 2 puntos | No publica. Mensaje claro. |
| Resultado con otro `request_id` | Se ignora. Se anota en el log. |
| Resultado que no valida | No se guarda. Se muestra el motivo exacto. |
| MoveIt con `status: error` | No se guarda como ejecutable. Se muestran `failed_segment` y motivo. |
| Fallo de escritura | Se informa del error. **No** se dice que fue guardado. |
| Archivo corrupto o editado a mano | Se rechaza al cargar, antes de mover. |
| Preflight fallido | No se envía ningún comando. Se explica qué punto y por qué. |
| Punto no alcanzado a tiempo | Se aborta la secuencia. |
| Protección activa | No arranca. Se listan todas las causas. |

### 13.15 Módulos nuevos

| Archivo | Responsabilidad | Dependencias |
|---|---|---|
| `trajectory_sequence_model.py` | Buffer temporal P1…PN, eventos de garra, contrato JSON, validación del resultado, documento de guardado | Python puro |
| `trajectory_storage.py` | Resolución de `trajectories/`, escritura atómica, listado, carga y validación | Python puro |
| `trajectory_ros_bridge.py` | Los 4 publishers/subscribers **sobre el nodo existente** | `rclpy`, PyQt5 |
| `trajectory_executor.py` | Máquina de estados de ejecución física, pacing, garra, manual/automático | PyQt5 |
| `trajectory_panel.py` | Widget compartido por las dos GUIs y cableado de todo lo anterior | PyQt5 |

### 13.16 Limitación conocida del protocolo KRL/EKI

`XmlDualMove.src` ejecuta **un `PTP` de parada exacta por cada `Seq` nuevo**,
con `WAIT SEC 0.1` por vuelta de bucle y `$OV_PRO = 5`. En consecuencia:

- una trayectoria densa de MoveIt se ejecuta como **N movimientos punto a
  punto con parada completa en cada uno**, no como un movimiento continuo
  mezclado;
- `time_from_start` se **conserva en el archivo** pero **no puede usarse**
  para marcar el ritmo: el tiempo lo impone el controlador;
- `velocities_rad_s` y `accelerations_rad_s2` se guardan pero el protocolo
  actual no las transporta.

Esto **no se ha modificado**: no se ha tocado KRL, ni EKI, ni SPS, ni el
Submit Interpreter. Ver el apartado correspondiente del README general.

---

## Licencia

MIT
