# kuka_telemetry_logger

Registrador **pasivo** de telemetría KUKA para ROS2.

Escucha el mismo topic que ya consume tu GUI y guarda **todo** lo que llega en
**CSV** y **SQLite**. No publica nada, no envía comandos, no toca el robot.

```
KUKA  XmlDualMove (SPS.SUB / .src)
   │   EthernetKRL  canal "XmlDualMove"  ->  TCP 59153
   ▼
nodo  eki_axis_move                    (kuka_eki_bridge — SIN MODIFICAR)
   │
   ├─► /kuka/axis_move/feedback_json  ──┬──► GUI  kuka_gui_axis_move_node  (SIN MODIFICAR)
   │                                    │
   │                                    └──► ESTE NODO  ──► CSV + SQLite
   └─► /kuka/axis_move/raw_robot_xml  ─────► ESTE NODO  ──► tabla raw_robot_xml
```

En ROS2 (DDS) varios suscriptores pueden compartir un topic sin interferirse.
La GUI no se entera de que este nodo existe.

---

## 1. Qué topic encontró

| | |
|---|---|
| **Topic principal** | `/kuka/axis_move/feedback_json` |
| **Topic secundario (opcional)** | `/kuka/axis_move/raw_robot_xml` |
| **Publisher** | nodo `eki_axis_move` — `kuka_eki_bridge/eki_axis_move_node.py:354` |
| **Suscriptor GUI** | nodo `kuka_gui_axis_move_node` — `kuka_gui_control/ros_axis_move_bridge.py:59` |
| **Consumo en la GUI** | `dual_kuka_rviz_window.py` → `_on_feedback()` |

> `/kuka/axis_move/raw_command_xml` **no** se registra como telemetría: es el XML
> que ROS2 **envía** al KUKA, no lo que el KUKA reporta.

---

## 2. Qué message type usa

```
std_msgs/msg/String
```

El campo `data` contiene un **objeto JSON serializado**. El logger lo expande
automáticamente a columnas reales; guardar el JSON como un solo texto opaco
haría inútil el registro.

**QoS** (idéntico al publisher y al suscriptor de la GUI, ambos usan el perfil
por defecto de rclpy con `depth=10`):

```
RELIABLE / VOLATILE / KEEP_LAST(10)
```

---

## 3. Qué campos contiene

Campos reales del JSON, leídos de `eki_axis_move_node.py:339-351` y del parser
`axis_move_xml_utils.py:91-210`:

| Campo JSON | Tipo | Origen en el KUKA (`XmlDualMove.xml` `<SEND>`) |
|---|---|---|
| `seq` | int | `Robot/Seq` |
| `mode` | str | `Robot/Mode` (`AxisTarget` \| `CartesianTarget`) |
| `status` | int | `Robot/Status` |
| `move_ready` | bool | `Robot/MoveReady` |
| `limits_ok` | bool | `Robot/LimitsOK` |
| `delta_ok` | bool | `Robot/DeltaOK` |
| `move_executed` | bool | `Robot/MoveExecuted` |
| `axis_actual.A1` … `.A6` | float | `Robot/Data/AxisActual/@A1..@A6` ← `$AXIS_ACT` |
| `position_actual.X/Y/Z/A/B/C` | float | `Robot/Data/PositionActual/@X..@C` ← `$POS_ACT` |
| `bridge_safe_mode` | bool | **generado en ROS2**, no viene del KUKA |
| `bridge_allow_motion` | bool | **generado en ROS2**, no viene del KUKA |

El logger **no** usa esta lista para decidir las columnas: las columnas del CSV
se derivan del **primer mensaje realmente recibido**. Si el mensaje cambia, las
columnas cambian con él. Cualquier campo que aparezca más tarde y no estuviera
en el primer mensaje se conserva íntegro en la columna `unmapped_json`.

---

## 4. De dónde proviene el timestamp

**El KUKA no envía ningún timestamp.** Verificado en tres sitios:

1. `std_msgs/String` no tiene `Header`, por lo tanto no hay `header.stamp`.
2. El JSON publicado no contiene ningún campo de tiempo.
3. `KUKA CODES/config/XmlDualMove.xml` — el bloque `<SEND>` no declara ningún
   elemento temporal.

Consecuencia, aplicada literalmente:

| Columna | Valor |
|---|---|
| `receive_wall_time_iso8601` | generado **aquí**, reloj de pared con zona horaria — `2026-08-17T18:35:21.145823-04:00` |
| `receive_ros_time_sec/nanosec/ns` | generado **aquí**, `node.get_clock().now()` |
| `delta_receive_ms` | calculado **aquí**, a partir de `receive_ros_time_ns` |
| `source_stamp_sec` | **NULL** |
| `source_stamp_nanosec` | **NULL** |
| `source_stamp_ns` | **NULL** |

No se inventa un timestamp de origen. Si algún día el mensaje incorpora un
`header.stamp`, esas columnas empiezan a llenarse solas, sin tocar el código.

**Lo único parecido a un reloj que sí viene del KUKA es `seq`**, un contador
monótono (`XD_SEND_SEQ` en `sps_submit.sub:459`, `seq` en `XmlDualMove.src:374`).

### Frecuencia

Declarada en el código del KUKA (**referencia, no medición**):

- `XmlDualMove.src:405` → `WAIT SEC 0.1` → ~10 Hz nominal
- `config_submit.dat:842` → `XD_SEND_DIVISOR = 8`, ciclo SPS ~12 ms → ~10.4 Hz nominal

El logger **mide** la frecuencia real: `delta_receive_ms` por muestra, más
`average_rate_hz`, `min_delta_ms` y `max_delta_ms` en el reporte. No asume 10 Hz
ni 12 ms.

---

## 5. Cómo ejecutar el logger (DESPUÉS de que decidas compilarlo)

Este paquete se entrega **sin compilar**. Cuando quieras usarlo:

```bash
# 1. Compilar SOLO este paquete (no toca los otros)
cd ~/Documents/TG2
colcon build --packages-select kuka_telemetry_logger
source install/setup.bash

# 2. Arranca tu sistema como siempre, en otra terminal:
#    ros2 launch kuka_gui_control gui_dual_kuka_rviz.launch.py

# 3. En una tercera terminal, el logger:
ros2 run kuka_telemetry_logger telemetry_logger
```

Variantes:

```bash
# Ver cada mensaje (ruidoso)
ros2 run kuka_telemetry_logger telemetry_logger --verbose

# Vía launch
ros2 launch kuka_telemetry_logger telemetry_logger.launch.py
ros2 launch kuka_telemetry_logger telemetry_logger.launch.py verbose:=true
ros2 launch kuka_telemetry_logger telemetry_logger.launch.py log_dir:=/tmp/kuka_logs

# Cambiar topic o carpeta sin recompilar
ros2 run kuka_telemetry_logger telemetry_logger --ros-args \
  -p telemetry_topic:=/kuka/axis_move/feedback_json \
  -p log_dir:=/home/eduardex/kuka_logs \
  -p report_every:=50

# Sin archivar el XML crudo (menos I/O)
ros2 run kuka_telemetry_logger telemetry_logger --ros-args -p log_raw_robot_xml:=false

# Ayuda
ros2 run kuka_telemetry_logger telemetry_logger --help
```

Para terminar: **Ctrl+C**. El nodo cierra los archivos, hace commit del SQLite e
imprime el resumen final.

### Parámetros

| Parámetro | Tipo | Default | Qué hace |
|---|---|---|---|
| `telemetry_topic` | str | `/kuka/axis_move/feedback_json` | Topic a observar |
| `log_raw_robot_xml` | bool | `true` | Archivar el `<Robot>` XML crudo |
| `raw_robot_xml_topic` | str | `/kuka/axis_move/raw_robot_xml` | Topic del XML crudo |
| `log_dir` | str | `logs` | Carpeta de salida (relativa al cwd) |
| `file_prefix` | str | `kuka_telemetry` | Prefijo de los archivos |
| `qos_depth` | int | `10` | Profundidad de la cola (compatible con el publisher) |
| `flush_every` | int | `20` | Flush del CSV y commit del SQLite cada N mensajes |
| `report_every` | int | `100` | Imprimir el bloque de diagnóstico cada N mensajes |
| `verbose` | bool | `false` | Imprimir cada mensaje |

### Salida en consola

Cada `report_every` mensajes:

```
[Telemetry Logger]
  Messages:        100
  Rate:            10.19 Hz
  Elapsed:         9.71 s
  Delta rx (ms):   mean=98.118 min=92.007 max=103.969
  Last Seq:        321
  Seq gaps:        1 (estimated missing: 3)
  Last msg time:   2026-08-17T18:35:21.145823-04:00
  Source stamp:    NOT PRESENT (NULL)
  Raw XML frames:  100
  CSV:             /home/eduardex/Documents/TG2/logs/kuka_telemetry_20260817_183521.csv
  DB:              /home/eduardex/Documents/TG2/logs/kuka_telemetry_20260817_183521.db
```

Al salir con Ctrl+C se imprime además el `FINAL SUMMARY` con el detalle de cada
salto de secuencia.

---

## 6. Dónde se generan CSV y SQLite

```
<log_dir>/kuka_telemetry_YYYYMMDD_HHMMSS.csv
<log_dir>/kuka_telemetry_YYYYMMDD_HHMMSS.db
```

`log_dir` es `logs` por defecto, **relativo al directorio desde el que lanzas el
nodo**. Si arrancas desde `~/Documents/TG2`, los archivos aparecen en
`~/Documents/TG2/logs/`. La carpeta se crea sola. Cada ejecución genera un par
de archivos nuevo; nunca se sobrescribe nada.

`flush_every: 20` significa que un `kill -9` pierde como máximo 20 filas.

---

## 7. Cómo abrir rápidamente el CSV

```bash
# Cabecera
head -1 logs/kuka_telemetry_*.csv | tr ',' '\n' | nl

# Primeras filas, en columnas alineadas
head -20 logs/kuka_telemetry_*.csv | column -s, -t | less -S

# Seguirlo en vivo mientras el logger corre
tail -f logs/kuka_telemetry_*.csv

# LibreOffice
libreoffice --calc logs/kuka_telemetry_*.csv

# Análisis completo (NO necesita ROS2)
python3 src/kuka_telemetry_logger/scripts/analyze_log.py logs/kuka_telemetry_*.csv
```

Con pandas, si lo tienes:

```python
import pandas as pd
df = pd.read_csv('logs/kuka_telemetry_20260817_183521.csv')
df['delta_receive_ms'].describe()
df[['axis_actual.A1', 'position_actual.X']].plot()
```

---

## 8. Cómo consultar SQLite

```bash
sqlite3 logs/kuka_telemetry_20260817_183521.db
```

```sql
.headers on
.mode column
.tables

-- Procedencia de la sesión: topic, QoS, tasa medida, si hubo timestamp de origen
SELECT * FROM session_info;

-- Últimos 10 mensajes
SELECT receive_index, receive_wall_time_iso8601, sequence, delta_receive_ms
FROM telemetry_messages ORDER BY id DESC LIMIT 10;

-- Frecuencia real medida
SELECT COUNT(*)                                             AS muestras,
       (MAX(receive_ros_time_ns)-MIN(receive_ros_time_ns))/1e9 AS segundos,
       (COUNT(*)-1)*1e9/(MAX(receive_ros_time_ns)-MIN(receive_ros_time_ns)) AS hz,
       AVG(delta_receive_ms) AS dt_medio_ms,
       MIN(delta_receive_ms) AS dt_min_ms,
       MAX(delta_receive_ms) AS dt_max_ms
FROM telemetry_messages;

-- Todos los saltos de secuencia
SELECT receive_index, prev_sequence, sequence, delta_seq, estimated_missing
FROM telemetry_messages WHERE sequence_gap > 0 ORDER BY receive_index;

-- Recorrido articular
SELECT MIN(axis_actual_a1), MAX(axis_actual_a1),
       MIN(axis_actual_a2), MAX(axis_actual_a2)
FROM telemetry_flat;

-- Momentos en que el robot reportó no estar OK
SELECT receive_index, receive_wall_time_iso8601, mode, limits_ok, delta_ok, move_ready
FROM telemetry_flat WHERE limits_ok = 0 OR delta_ok = 0;

-- Sacar un campo del JSON completo sin depender de la tabla flat
SELECT receive_index, json_extract(payload_json, '$.axis_actual.A1') AS a1
FROM telemetry_messages LIMIT 5;

-- Exportar a CSV desde SQLite
.headers on
.mode csv
.output export.csv
SELECT * FROM telemetry_flat;
.output stdout
```

### Esquema

**`telemetry_messages`** — fuente de verdad, nunca se pierde nada

| Columna | Tipo | |
|---|---|---|
| `id` | INTEGER PRIMARY KEY AUTOINCREMENT | |
| `receive_index` | INTEGER NOT NULL | contador del logger, empieza en 1 |
| `topic_name` | TEXT NOT NULL | |
| `message_type` | TEXT NOT NULL | `std_msgs/msg/String` |
| `receive_wall_time_iso8601` | TEXT NOT NULL | |
| `receive_ros_time_sec` | INTEGER NOT NULL | |
| `receive_ros_time_nanosec` | INTEGER NOT NULL | |
| `receive_ros_time_ns` | INTEGER NOT NULL | |
| `delta_receive_ms` | REAL | NULL en el primer mensaje |
| `source_stamp_sec` | INTEGER | **NULL** con la telemetría actual |
| `source_stamp_nanosec` | INTEGER | **NULL** con la telemetría actual |
| `source_stamp_ns` | INTEGER | **NULL** con la telemetría actual |
| `sequence` | INTEGER | `Robot/Seq` del KUKA |
| `prev_sequence` | INTEGER | |
| `delta_seq` | INTEGER | `sequence − prev_sequence` |
| `sequence_gap` | INTEGER | `delta_seq` cuando `delta_seq > 1`, si no `0` |
| `estimated_missing` | INTEGER | `delta_seq − 1` cuando hay salto |
| `payload_json` | TEXT NOT NULL | **el mensaje completo en JSON** |

**`telemetry_flat`** — las variables principales en columnas individuales

`id`, `receive_index`, `receive_wall_time_iso8601`, `receive_ros_time_ns`,
`delta_receive_ms`, `sequence`, `prev_sequence`, `delta_seq`, `sequence_gap`,
`estimated_missing`, `mode` TEXT, `status` INTEGER,
`move_ready` / `limits_ok` / `delta_ok` / `move_executed` /
`bridge_safe_mode` / `bridge_allow_motion` INTEGER (0/1),
`axis_actual_a1..a6` REAL, `position_actual_x/y/z/a/b/c` REAL.

**`raw_robot_xml`** — `id`, `receive_index`, `topic_name`,
`receive_wall_time_iso8601`, `receive_ros_time_ns`, `xml`.

**`session_info`** — `key` / `value`: topic, message type, QoS, rutas, inicio y
fin de sesión, tasa media medida, min/max delta, si existía secuencia, si existía
timestamp de origen.

Índices sobre `sequence` y `receive_ros_time_ns` en ambas tablas de datos.

---

## 9. Interpretación de `sequence_gap`

```
seq:  100  101  102  105
                      ↑ delta_seq = 3
```

Se registra `sequence_gap = 3` y `estimated_missing = 2`.

**Un salto NO se interpreta automáticamente como pérdida de red.** Un reinicio
del KUKA, una reconexión TCP, o un ciclo SPS saltado producen exactamente la
misma huella en el log. El logger lo **etiqueta** y lo reporta; la causa se
investiga aparte.

---

## 10. Análisis offline — `scripts/analyze_log.py`

Solo biblioteca estándar de Python. **No importa ROS2** y no necesita workspace
compilado. Funciona en cualquier máquina.

```bash
python3 scripts/analyze_log.py logs/kuka_telemetry_20260817_183521.csv
python3 scripts/analyze_log.py archivo.csv --no-histogram
python3 scripts/analyze_log.py archivo.csv --top-gaps 50 --bins 40
```

Reporta: total de muestras, duración, primer y último timestamp, frecuencia
media, frecuencia instantánea mín/máx/mediana, delta medio/mediano/mín/máx y
desviación estándar, histograma ASCII de deltas, número y detalle de los saltos
de secuencia, todas las columnas encontradas con su conteo de NULL, y min/max/media
de A1..A6 y de X/Y/Z/A/B/C cuando esas columnas existen.

---

## 11. Garantías de este paquete

- **No publica nada.** El nodo no crea ni un solo publisher. Compruébalo:
  ```bash
  ros2 node info /kuka_telemetry_logger      # Publishers: solo /rosout y /parameter_events
  ```
- **No abre sockets al KUKA.** No importa `socket`, no toca EthernetKRL. Toda la
  telemetría le llega ya convertida a ROS2 por el nodo `eki_axis_move`.
- **No depende de `kuka_eki_bridge` ni de `kuka_gui_control`.** Solo de `rclpy`
  y `std_msgs`. Puedes compilarlo, moverlo o borrarlo sin afectar a nada más.
- **No modificó ningún archivo existente.** Todo lo que hay aquí es nuevo.
