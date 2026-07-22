# kuka_eki_bridge

Paquete ROS2 Humble para comunicación TCP/XML con robots KUKA mediante **KUKA.Ethernet KRL (EKI)**.

Este paquete reemplaza `EthernetKRL_Server.exe` en Ubuntu, permitiendo recibir y responder mensajes XML del controlador KUKA a través de la interfaz EthernetKRL, sin mover el robot.

---

## Objetivo

Establecer comunicación bidireccional entre una laptop Ubuntu y un robot KUKA usando:

- **Protocolo**: TCP/IP
- **Formato**: XML
- **Interfaz KUKA**: EthernetKRL (puerto X66/KLI)
- **Programa KUKA**: `XmlTransmit.src`

**Esta prueba NO mueve el robot.** Solo valida la comunicación TCP/XML entre Ubuntu y el controlador KUKA.

---

## Arquitectura

```
┌──────────────────────┐         TCP/XML          ┌──────────────────────┐
│   KUKA Controller    │ ──────────────────────▶   │   Ubuntu Laptop      │
│                      │                           │                      │
│  XmlTransmit.src     │   Puerto: 59152           │  eki_xml_server_node │
│  (envía XML <Robot>) │ ◀──────────────────────   │  (responde <Sensor>) │
│                      │                           │                      │
│  IP: 192.168.250.20  │    Cable LAN (X66/KLI)    │  IP: 192.168.250.30  │
└──────────────────────┘                           └──────────────────────┘
```

---

## Estructura del paquete

```
kuka_eki_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── kuka_eki_bridge
├── kuka_eki_bridge/
│   ├── __init__.py
│   ├── eki_xml_server_node.py      # Nodo ROS2 — ejemplo básico EKI
│   ├── eki_protocol.py             # Servidor TCP — ejemplo básico
│   ├── xml_utils.py                # Utilidades XML — ejemplo básico
│   ├── eki_axis_stream_node.py     # Nodo ROS2 — streaming de ejes
│   ├── eki_axis_stream_server.py   # Servidor TCP — streaming continuo
│   └── axis_xml_utils.py           # Utilidades XML — stream + buffer TCP
├── config/
│   ├── eki_server.yaml             # Parámetros — ejemplo básico
│   └── axis_stream.yaml            # Parámetros — streaming de ejes
├── launch/
│   ├── eki_server.launch.py        # Launch — ejemplo básico
│   └── axis_stream.launch.py       # Launch — streaming de ejes
├── examples/
│   ├── sensor_response.xml         # XML de respuesta al KUKA
│   ├── sample_robot_received.xml   # Ejemplo XML — ejemplo básico
│   └── axis_stream_sample.xml      # Ejemplo XML — streaming de ejes
└── README.md
```

---

## 1. Configuración de red en Ubuntu

### Requisitos

- Cable Ethernet conectado al puerto **X66/KLI** del controlador KUKA.
- El otro extremo del cable conectado a la laptop Ubuntu.

### Paso 1: Detectar la conexión cableada

```bash
# Listar todas las conexiones de red
nmcli connection show

# Buscar la conexión tipo ethernet (normalmente "Wired connection 1" o similar)
nmcli device status
```

Anota el nombre de la conexión cableada. En los siguientes comandos se usa `"Wired connection 1"` como ejemplo. **Reemplázalo por el nombre real** que aparezca en tu sistema.

### Paso 2: Configurar IP estática

```bash
# Asignar la IP estática 192.168.250.30/24 (sin gateway)
nmcli connection modify "Wired connection 1" \
  ipv4.addresses 192.168.250.30/24 \
  ipv4.method manual \
  ipv4.gateway ""

# Aplicar los cambios
nmcli connection up "Wired connection 1"
```

### Paso 3: Verificar la configuración

```bash
# Verificar que la IP fue asignada correctamente
ip addr show

# Deberías ver algo como:
#   inet 192.168.250.30/24 ...
```

### Paso 4: Probar conectividad con el KUKA

```bash
# Hacer ping al controlador KUKA
ping -c 4 192.168.250.20
```

Si el ping responde, la red está configurada correctamente.

---

## 2. Configuración del KUKA

En el controlador KUKA, el archivo `XmlTransmit.xml` debe tener configurado:

```xml
<ETHERNETKRL>
  <CONFIGURATION>
    <EXTERNAL>
      <TYPE>Client</TYPE>
    </EXTERNAL>
    <INTERNAL>
      <ENVIRONMENT>Program</ENVIRONMENT>
      <BUFFERING Limit="512"/>
      <ALIVE Set_Flag="1" Ping_Timeout="50"/>
      <IP>192.168.250.30</IP>
      <PORT>59152</PORT>
      <PROTOCOL>TCP</PROTOCOL>
    </INTERNAL>
  </CONFIGURATION>
  <!-- ... resto de la configuración ... -->
</ETHERNETKRL>
```

Los valores clave son:
- `<IP>192.168.250.30</IP>` → IP de la laptop Ubuntu
- `<PORT>59152</PORT>` → Puerto del servidor TCP

---

## 3. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select kuka_eki_bridge
source install/setup.bash
```

> **Nota:** Si tu workspace está en otra ubicación, ajusta la ruta. Por ejemplo:
> ```bash
> cd ~/Documents/TG2
> colcon build --packages-select kuka_eki_bridge
> source install/setup.bash
> ```

---

## 4. Ejecutar el servidor

### Opción 1: Con launch file (recomendado)

```bash
ros2 launch kuka_eki_bridge eki_server.launch.py
```

### Opción 2: Con parámetros personalizados

```bash
ros2 launch kuka_eki_bridge eki_server.launch.py port:=59152 bind_host:=0.0.0.0
```

### Opción 3: Ejecutar el nodo directamente

```bash
ros2 run kuka_eki_bridge eki_xml_server_node --ros-args \
  -p bind_host:="0.0.0.0" \
  -p port:=59152
```

---

## 5. Qué esperar en consola

Al iniciar el nodo, verás:

```
[INFO] ╔══════════════════════════════════════════╗
[INFO] ║     KUKA EKI XML Server — ROS2 Node     ║
[INFO] ╚══════════════════════════════════════════╝
[INFO]   Bind host:       0.0.0.0
[INFO]   Port:            59152
[INFO]   Response XML:    (default)
[INFO]   Log raw XML:     True
[INFO]   Pretty print:    True
[INFO]   Buffer size:     8192
[INFO]   Keep running:    True
[INFO] ──────────────────────────────────────────
[INFO] No response XML path configured — using default.
[INFO] EKI XML Server started.
[INFO] TCP server listening on 0.0.0.0:59152
[INFO] Waiting for KUKA connection...
```

Cuando el KUKA ejecute `XmlTransmit.src` y se conecte:

```
[INFO] Client connected: 192.168.250.20:XXXXX
[INFO] ━━━ Data received from 192.168.250.20:XXXXX ━━━
[INFO] [RAW XML]
<Robot><Data><ActPos X="1000.12" Y="0.0" Z="500.0" A="0.0" B="90.0" C="0.0"/></Data><Status>12345678</Status><Mode>T1</Mode></Robot>
[INFO] [FORMATTED XML]
<Robot>
  <Data>
    <ActPos X="1000.12" Y="0.0" Z="500.0" A="0.0" B="90.0" C="0.0"/>
  </Data>
  <Status>12345678</Status>
  <Mode>T1</Mode>
</Robot>
[INFO] [EXTRACTED FIELDS]
[INFO]   RootTag: Robot
[INFO]   Mode: T1
[INFO]   Status: 12345678
[INFO]   ActPos: {'X': '1000.12', 'Y': '0.0', 'Z': '500.0', 'A': '0.0', 'B': '90.0', 'C': '0.0'}
[INFO] Sent response to 192.168.250.20:XXXXX (XXX bytes)
```

---

## 6. Parámetros configurables

| Parámetro | Tipo | Default | Descripción |
|---|---|---|---|
| `bind_host` | string | `"0.0.0.0"` | IP de escucha del servidor TCP |
| `port` | int | `59152` | Puerto TCP (debe coincidir con `XmlTransmit.xml`) |
| `response_xml_path` | string | `""` | Ruta a XML de respuesta personalizado |
| `log_raw_xml` | bool | `true` | Mostrar XML crudo en consola |
| `pretty_print_xml` | bool | `true` | Mostrar XML formateado en consola |
| `receive_buffer_size` | int | `8192` | Tamaño del buffer de recepción TCP |
| `keep_running` | bool | `true` | Seguir aceptando conexiones tras desconexión |

---

## 7. Detener el servidor

Presiona `Ctrl+C` en la terminal. El nodo cerrará el socket TCP de forma limpia.

---

## 8. Troubleshooting

### El puerto está ocupado

```
[ERROR] Port 59152 is occupied. Check with: ss -tlnp | grep 59152
```

Solución:
```bash
# Ver qué proceso usa el puerto
ss -tlnp | grep 59152

# Si es necesario, matar el proceso
kill -9 <PID>
```

### El KUKA no se conecta

1. Verificar que el ping funciona: `ping -c 4 192.168.250.20`
2. Verificar que `XmlTransmit.xml` apunta a `192.168.250.30:59152`
3. Verificar que el cable está en el puerto **X66/KLI** (no X65)
4. Verificar que el programa `XmlTransmit.src` está ejecutándose en el KUKA

### XML malformado

El servidor no se cae si recibe XML malformado. Mostrará una advertencia:
```
[WARN] Could not pretty-print XML (malformed?).
```

---

## 10. Streaming continuo de posición

> **Nota:** Este modo es completamente independiente del ejemplo básico EKI.
> No modifica `eki_server.launch.py`, `eki_xml_server_node.py` ni `eki_protocol.py`.
> El ejemplo básico sigue funcionando exactamente igual.

### Objetivo

Recibir un stream continuo de XML desde el KUKA con los valores de `$AXIS_ACT` y `$POS_ACT`, mostrarlos en consola y publicarlos como topics ROS2. **No mueve el robot.**

### Arquitectura

```
┌──────────────────────┐     TCP stream       ┌────────────────────────────────┐
│   KUKA Controller    │ ─────────────────▶    │   Ubuntu Laptop                │
│                      │   <Robot> XML ×N      │                                │
│  XmlAxisLoop.src     │   (continuo)          │  eki_axis_stream_node          │
│  ($AXIS_ACT/$POS_ACT)│                       │   ├─ /kuka/axis_stream/raw_xml │
│                      │                       │   └─ /joint_states (opcional)  │
│  IP: 192.168.250.20  │   Cable LAN (X66/KLI) │  IP: 192.168.250.30           │
└──────────────────────┘                       └────────────────────────────────┘
```

### Configuración del KUKA

En el controlador KUKA debe existir:

1. **Programa**: `XmlAxisLoop.src` — un loop que envía `$AXIS_ACT` y `$POS_ACT` repetidamente.
2. **Configuración**: `XmlAxisLoop.xml` con:
   ```xml
   <IP>192.168.250.30</IP>
   <PORT>59152</PORT>
   ```

### Ejecutar el streaming

```bash
# Terminal 1: iniciar el nodo
ros2 launch kuka_eki_bridge axis_stream.launch.py

# Terminal 2: ver los datos publicados
ros2 topic echo /kuka/axis_stream/raw_xml
```

### Salida esperada en consola

```
[INFO] ╔══════════════════════════════════════════════╗
[INFO] ║   KUKA EKI Axis Stream — ROS2 Node          ║
[INFO] ╚══════════════════════════════════════════════╝
[INFO]   Bind host:          0.0.0.0
[INFO]   Port:               59152
[INFO]   Log axis values:    True
[INFO]   Publish raw XML:    True
[INFO]   Publish JointState: False
[INFO]   Send response:      False
[INFO] ──────────────────────────────────────────────
[INFO] Axis stream server listening on 0.0.0.0:59152
[INFO] Waiting for KUKA connection...
[INFO] KUKA connected: 192.168.250.20:XXXXX
[INFO] Seq=1 | Mode=T1 | A1=0.00 A2=-90.00 A3=90.00 A4=0.00 A5=0.00 A6=0.00 | X=1000.12 Y=0.00 Z=500.00 A=0.00 B=90.00 C=0.00
[INFO] Seq=2 | Mode=T1 | A1=0.50 A2=-89.80 A3=89.90 A4=0.10 A5=0.00 A6=0.00 | X=1001.00 Y=0.50 Z=499.80 A=0.10 B=89.90 C=0.10
...
```

### Parámetros configurables

| Parámetro | Tipo | Default | Descripción |
|---|---|---|---|
| `bind_host` | string | `"0.0.0.0"` | IP de escucha |
| `port` | int | `59152` | Puerto TCP |
| `receive_buffer_size` | int | `8192` | Buffer de recepción |
| `log_raw_xml` | bool | `false` | Mostrar XML crudo (verbose) |
| `log_pretty_xml` | bool | `false` | Mostrar XML formateado (verbose) |
| `log_axis_values` | bool | `true` | Mostrar línea compacta de ejes |
| `publish_raw_xml` | bool | `true` | Publicar en `/kuka/axis_stream/raw_xml` |
| `publish_joint_states` | bool | `false` | Publicar en `/joint_states` |
| `send_response` | bool | `false` | Responder con `<Sensor>` keepalive |

### Activar `/joint_states` para RViz/MoveIt2

```bash
ros2 launch kuka_eki_bridge axis_stream.launch.py publish_joint_states:=true
```

Esto convierte los valores de ejes de grados a radianes y publica `sensor_msgs/JointState` con los nombres:
`joint_a1`, `joint_a2`, `joint_a3`, `joint_a4`, `joint_a5`, `joint_a6`.

### Manejo de TCP como stream

El servidor implementa un buffer interno (`TcpXmlBuffer`) que:
- Acumula datos del stream TCP.
- Extrae mensajes completos `<Robot>...</Robot>` usando regex.
- Maneja mensajes fragmentados (un XML partido en múltiples `recv`).
- Maneja mensajes concatenados (múltiples XML en un solo `recv`).
- Protege contra crecimiento ilimitado del buffer.

---

## 11. Siguiente etapa

Este paquete valida la comunicación TCP/XML básica. Los siguientes pasos para integrar completamente el KUKA con ROS2 son:

### Publicar `/joint_states`

1. **Modificar el programa KRL** (`XmlTransmit.src`) para que envíe los valores de `$AXIS_ACT` (posiciones articulares actuales) dentro del XML:
   ```xml
   <Robot>
     <Data>
       <AxisAct A1="0.0" A2="-90.0" A3="90.0" A4="0.0" A5="0.0" A6="0.0"/>
       <ActPos X="1000.12" Y="0.0" Z="500.0" A="0.0" B="90.0" C="0.0"/>
     </Data>
     <Status>12345678</Status>
     <Mode>T1</Mode>
   </Robot>
   ```

2. **Agregar un publisher** en el nodo ROS2 que convierta los valores `A1`-`A6` a un mensaje `sensor_msgs/JointState` y lo publique en el topic `/joint_states`.

3. **Integrar con MoveIt2** para visualización y planificación de trayectorias.

### Enviar comandos al robot

En una etapa posterior, se podrá:
- Recibir trayectorias de MoveIt2
- Enviarlas al KUKA como posiciones objetivo via XML
- Implementar un `FollowJointTrajectory` action server

> ⚠️ **Importante:** El envío de comandos de movimiento requiere configuración adicional de seguridad en el controlador KUKA y no forma parte de este paquete inicial.

---

## Command loop bidireccional

### Modo `axis_command_loop`

Este modo permite una comunicación TCP/XML continua y **bidireccional** con el robot KUKA.
A diferencia de los modos anteriores (solo lectura), aquí Ubuntu **responde** a cada mensaje del KUKA con un objetivo articular.

#### Arquitectura del flujo

```
GUI
 │  publica target JSON
 ▼
/kuka/axis_command/target_json   (std_msgs/String)
 │
 ▼
axis_command_loop (Python, puerto 59153)
 │  responde XML <Command> al KUKA
 ▼
KUKA  ←──── TCP/XML ────────────
 │  envía <Robot> con AxisActual + PositionActual
 ▼
axis_command_loop
 │  publica feedback JSON
 ▼
/kuka/axis_command_loop/feedback_json   (std_msgs/String)
 │
 ▼
GUI — muestra Target, Feedback, Error
```

#### Archivos nuevos (no modifican los existentes)

| Archivo | Rol |
|---|---|
| `kuka_eki_bridge/eki_axis_command_loop_node.py` | Nodo ROS2 principal |
| `kuka_eki_bridge/eki_axis_command_loop_server.py` | Servidor TCP bidireccional |
| `kuka_eki_bridge/axis_command_loop_xml_utils.py` | Parser y builder XML |
| `config/axis_command_loop.yaml` | Parámetros del nodo |
| `launch/axis_command_loop.launch.py` | Launch file independiente |
| `examples/axis_command_loop_robot_sample.xml` | XML de referencia KUKA→Ubuntu |
| `examples/axis_command_loop_response_sample.xml` | XML de referencia Ubuntu→KUKA |

#### Comandos

```bash
# Ejecutar el command loop
ros2 launch kuka_eki_bridge axis_command_loop.launch.py

# Ver feedback JSON del KUKA
ros2 topic echo /kuka/axis_command_loop/feedback_json

# Ver XML de comando enviado al KUKA
ros2 topic echo /kuka/axis_command_loop/raw_command_xml

# Ver XML crudo recibido del KUKA
ros2 topic echo /kuka/axis_command_loop/raw_robot_xml

# Publicar un target manualmente (para pruebas sin GUI)
ros2 topic pub /kuka/axis_command/target_json std_msgs/msg/String \
  '{data: "{\"seq\":1,\"source\":\"test\",\"mode\":\"manual_send\",\"enable_move\":false,\"A1\":0.0,\"A2\":-90.0,\"A3\":90.0,\"A4\":0.0,\"A5\":0.0,\"A6\":0.0}"}'
```

#### Parámetros principales (`config/axis_command_loop.yaml`)

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `port` | `59153` | Puerto TCP del servidor |
| `safe_mode` | `true` | Fuerza `EnableMove=0` siempre |
| `log_feedback_values` | `true` | Imprime línea compacta por ciclo |
| `log_command_xml` | `true` | Muestra XML enviado al KUKA |

#### Seguridad

- Si `safe_mode=true`, `EnableMove` se envía **siempre como 0**, sin importar lo que pida la GUI.
- El movimiento real queda bajo control del programa KRL del KUKA (`XmlAxisCommandLoop.src`).
- Este nodo **no ejecuta** PTP, LIN ni CIRC desde Python.

#### Programa KUKA requerido

Del lado del robot se necesita:

- **`XmlAxisCommandLoop.src`** — programa KRL que conecta al puerto 59153,
  envía `$AXIS_ACT` y `$POS_ACT`, y recibe `AxisTarget` + `EnableMove`.
- **`XmlAxisCommandLoop.xml`** — configuración EthernetKRL para ese programa.

> El movimiento real (PTP/LIN) se añadirá en una revisión posterior del KRL,
> bajo supervisión y con validación de seguridades en el controlador KUKA.

---

## Licencia

MIT
