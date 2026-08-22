<div align="center">

# 🤖 KUKA KR6 R900 Control GUI
## Control TCP/IP con ROS2, EthernetKRL y visualización dual en RViz/MoveIt2

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![KUKA](https://img.shields.io/badge/KUKA-KR6_R900-orange.svg)
![EthernetKRL](https://img.shields.io/badge/Communication-EthernetKRL-yellow.svg)
![TCP/IP](https://img.shields.io/badge/Protocol-TCP%2FIP-lightgrey.svg)
![RViz2](https://img.shields.io/badge/Visualization-RViz2-green.svg)
![MoveIt2](https://img.shields.io/badge/Planning-MoveIt2-red.svg)
![PyQt5](https://img.shields.io/badge/GUI-PyQt5-blueviolet.svg)

Sistema de control para el robot KUKA KR6 R900 mediante una interfaz gráfica en ROS2, con comunicación TCP/IP hacia el controlador KUKA usando EthernetKRL y una modalidad dual para replicar los comandos articulares en RViz/MoveIt2.

</div>

---

## 📚 Índice

1. [Descripción General](#1-descripción-general)
2. [Arquitectura del Sistema](#2-arquitectura-del-sistema)
3. [Modos de Operación](#3-modos-de-operación)
4. [Paquetes Principales](#4-paquetes-principales)
5. [Tópicos ROS2](#5-tópicos-ros2)
6. [Configuración de Seguridad](#6-configuración-de-seguridad)
7. [Ejecución de la GUI Original TCP/IP](#7-ejecución-de-la-gui-original-tcpip)
8. [Ejecución del Visualizador RViz/MoveIt2](#8-ejecución-del-visualizador-rvizmoveit2)
9. [Ejecución de la GUI Dual KUKA + RViz](#9-ejecución-de-la-gui-dual-kuka--rviz)
10. [Comandos de Verificación](#10-comandos-de-verificación)
11. [Operación Recomendada Paso a Paso](#11-operación-recomendada-paso-a-paso)
12. [Límites Articulares](#12-límites-articulares)
13. [Función Cartesiana / Mundo](#13-función-cartesiana--mundo)
14. [Problemas Comunes](#14-problemas-comunes)
15. [Notas Importantes](#15-notas-importantes)
16. [Secuencias de Trayectorias](#16-secuencias-de-trayectorias)

---

## 1. Descripción General

Este repositorio contiene la lógica para la operación del robot **KUKA KR6 R900** en un entorno distribuido mediante **ROS2 Humble**. El sistema permite:

- Controlar las articulaciones (Joints A1-A6) del KUKA real desde una Interfaz Gráfica (GUI) en PyQt5.
- Enviar los comandos mediante tópicos de ROS2 hacia un nodo puente (*bridge*).
- Convertir los mensajes ROS2 a formato **XML** y enviarlos vía TCP/IP al KUKA real usando **EthernetKRL**.
- Recibir el *feedback* en tiempo real del robot (posición actual, errores, status) y reflejarlos en la GUI.
- Ver el XML que se envía y el que se recibe para depuración directa.
- Utilizar un **Modo Dual** para enviar el mismo objetivo (target) articular tanto al robot real como a una simulación/gemelo digital en **RViz/MoveIt2** de forma simultánea.

> [!IMPORTANT]  
> La **función cartesiana (Herramienta Mundo)** está en fase de integración. Aunque los scripts KRL (`XmlDualMove.src/xml`) ya están preparados para recibir comandos cartesianos, actualmente la GUI utiliza esta función principalmente para comandar a RViz/MoveIt2. Verifica cuidadosamente antes de enviar coordenadas cartesianas al robot real.

---

## 2. Arquitectura del Sistema

El flujo de información se divide de la siguiente manera para el **Modo Dual**:

```text
┌─────────────────────────────────────────┐
│             KUKA Dual GUI               │
│             PyQt5 + ROS2                │
└────────┬───────────────────────┬────────┘
         │                       │
 /kuka/axis_move/        /kuka_bridge/
   target_json         joint_command_deg
         ▼                       ▼
┌─────────────────┐ ┌──────────────────────────┐
│ kuka_eki_bridge │ │ kuka_moveit_bridge_node  │
│ ROS2 → TCP/XML  │ │ ROS2 → MoveIt Actions    │
└────────┬────────┘ └────────────┬─────────────┘
         │                       │
 TCP/IP EthernetKRL         /move_action
         ▼                       ▼
┌─────────────────┐ ┌──────────────────────────┐
│   KUKA KR C4    │ │      RViz / MoveIt2      │
│ XmlDualMove.src │ │   (kuka_kr6_moveit_...)  │
└─────────────────┘ └──────────────────────────┘
```

---

## 3. Modos de Operación

1. **Modo Original TCP/IP:** Solo se comunica con el KUKA real. Ideal para tareas donde no se requiera el gemelo digital o si MoveIt2 no está activo.
2. **Modo Dual:** Se conecta a ambos entornos simultáneamente. Útil para pre-visualizar movimientos en RViz mientras el robot real se mueve, logrando una correspondencia 1:1 de los ejes.

---

## 4. Paquetes Principales

| Paquete | Descripción |
|---------|-------------|
| `kuka_gui_control` | Contiene los nodos de PyQt5, tanto para la GUI original como para la GUI Dual. |
| `kuka_eki_bridge` | Servidor TCP/IP en Python que fragmenta/lee XML, maneja deadlocks y se comunica con EthernetKRL. |
| `kuka_gui_moveit_bridge` | Nodo traductor que convierte comandos articulares en grados a acciones de trayectoria para MoveIt2. |

---

## 5. Tópicos ROS2

| Tópico | Tipo de Mensaje | Uso |
|--------|-----------------|-----|
| `/kuka/axis_move/target_json` | `std_msgs/String` (JSON) | Envia el target A1-A6 de la GUI al bridge TCP/IP. |
| `/kuka/axis_move/feedback_json` | `std_msgs/String` (JSON) | Feedback actual del KUKA real recibido por el bridge. |
| `/kuka/axis_move/raw_command_xml` | `std_msgs/String` | El XML literal que se envía por TCP/IP. |
| `/kuka_bridge/joint_command_deg` | `std_msgs/Float64MultiArray` | Target A1-A6 (en grados) enviado de la GUI Dual a RViz. |
| `/kuka_bridge/joint_state_deg` | `std_msgs/Float64MultiArray` | Feedback A1-A6 (en grados) proveniente de MoveIt. |
| `/move_action` | `MoveGroupAction` | Acción que MoveIt recibe para planificar y ejecutar. |
| `/kuka_moveit/trajectory_generation/request_json` | `std_msgs/String` (JSON) | **GUI → MoveIt2.** P1…PN + eventos de garra en UNA solicitud. QoS Reliable. |
| `/kuka_moveit/trajectory_generation/result_json` | `std_msgs/String` (JSON) | **MoveIt2 → GUI.** Segmentos T1, T2, T3… con sus puntos intermedios. QoS Reliable. |
| `/kuka_moveit/trajectory_preview/request_json` | `std_msgs/String` (JSON) | **GUI → RViz.** Archivo a previsualizar. Solo RViz. QoS Reliable. |
| `/kuka_moveit/trajectory_preview/status_json` | `std_msgs/String` (JSON) | **MoveIt2 → GUI.** Estado de la previsualización. QoS Reliable. |

---

## 6. Configuración de Seguridad

> [!WARNING]  
> Mover un robot industrial de forma remota conlleva riesgos. El sistema tiene múltiples capas de seguridad.

1. **`safe_mode` (Launch Arg):** Si es `true`, requiere que el usuario mantenga pulsado el botón de envío en la GUI. Si es `false`, permite clicks directos.
2. **`allow_motion_commands` (Launch Arg):** Si es `false`, el puente TCP rechaza enviar movimiento (solo lectura de telemetría).
3. **Validación de Límites en el KRL:** Los límites (±160, etc.) están hardcodeados en el archivo `.src` del KUKA. Si envías un comando fuera de rango, el KUKA ignora el paquete por seguridad.
4. **Validación de Delta (Salto):** El archivo `.src` tiene un `MAX_DELTA` (ej: 10 grados). Si se envía un objetivo muy alejado del estado actual, el movimiento se bloquea.

---

## 7. Ejecución de la GUI Original TCP/IP

Si solo deseas mover el robot real sin conectarte a RViz:

```bash
cd ~/Documents/TG2
colcon build --packages-select kuka_gui_control kuka_eki_bridge
source install/setup.bash

ros2 launch kuka_gui_control gui_with_axis_move_bridge.launch.py safe_mode:=false allow_motion_commands:=true
```

---

## 8. Ejecución del Visualizador RViz/MoveIt2

Para usar el modo dual, **primero debes levantar RViz y MoveIt** junto con su puente traductor. Esto generalmente vive en otro workspace (ej: `taller1`):

```bash
cd ~/Documents/taller1/ros2_ws
colcon build --packages-select kuka_gui_moveit_bridge
source install/setup.bash

# Lanza MoveIt, RViz, Fake Controllers y kuka_moveit_bridge_node
ros2 launch kuka_gui_moveit_bridge kuka_bridge_system.launch.py
```
Espera hasta que el terminal indique: `Sistema LISTO: joint_states validos, TF disponible, /move_action UP`.

---

## 9. Ejecución de la GUI Dual KUKA + RViz

Una vez que RViz esté listo, abre una **segunda terminal** y lanza la GUI Dual:

```bash
cd ~/Documents/TG2
source install/setup.bash

ros2 launch kuka_gui_control gui_dual_kuka_rviz.launch.py safe_mode:=false allow_motion_commands:=true
```
Aparecerá una ventana de selección. Activa "KUKA TCP/IP" y "RViz / MoveIt2" y presiona Iniciar.

---

## 10. Comandos de Verificación

Si tienes problemas, puedes inspeccionar el tráfico de ROS2 abriendo una terminal nueva:

```bash
# Ver tópicos activos
ros2 topic list

# Ver qué comandos XML están saliendo al KUKA
ros2 topic echo /kuka/axis_move/raw_command_xml

# Ver el feedback en JSON directo desde el robot
ros2 topic echo /kuka/axis_move/feedback_json

# Ver qué se está enviando a MoveIt
ros2 topic echo /kuka_bridge/joint_command_deg
```

---

## 11. Operación Recomendada Paso a Paso

> [!TIP]  
> **Flujo de trabajo seguro recomendado:**

1. Carga el programa `XmlDualMove.src` (o `XmlAxisMove.src`) en el KUKA SmartPad.
2. Posiciona el KUKA en estado inicial/home si es necesario.
3. Arranca el archivo `.src` en KUKA. Se quedará esperando en un LOOP de recepción TCP.
4. Lanza el visualizador RViz (Terminal 1).
5. Lanza la GUI Dual (Terminal 2).
6. La GUI se "sincronizará" con la posición real del KUKA para evitar saltos.
7. Marca el checkbox **ENABLE MOVE**.
8. Usa los botones **+/-** en incrementos pequeños para validar el movimiento suave en ambos lados (RViz y Robot real).

---

## 12. Límites Articulares

La GUI y los archivos `.src` del controlador están configurados con los rangos reales del KUKA KR6 R900 más un margen de seguridad de ~10° en los extremos:

| Articulación | Mínimo | Máximo | Rango Permitido GUI |
|--------------|--------|--------|---------------------|
| **A1** | -170.0° | +170.0° | `[-160.0, 160.0]` |
| **A2** | -190.0° | +45.0° | `[-180.0, 35.0]` |
| **A3** | -120.0° | +156.0° | `[-110.0, 146.0]` |
| **A4** | -185.0° | +185.0° | `[-175.0, 175.0]` |
| **A5** | -120.0° | +120.0° | `[-110.0, 110.0]` |
| **A6** | -350.0° | +350.0° | `[-340.0, 340.0]` |

---

## 13. Función Cartesiana / Mundo

El KUKA permite movimientos articulares (PTP) y cartesianos (LIN / PTP espaciales).
La herramienta Mundo permite mover el TCP en los ejes espaciales (X, Y, Z) y orientaciones Euler (A, B, C). 

Para poder operar el KUKA en modo Cartesiano, se requiere el archivo `XmlDualMove.src` el cual analiza la etiqueta `<Mode>CartesianTarget</Mode>` dentro del XML enviado, para leer los datos X, Y, Z, A, B, C en lugar de los ejes angulares. 

> [!NOTE]  
> Esta función es ideal para movimientos finos de *Pick and Place* o teleoperación usando MoveIt2 como filtro de cinemática inversa.

---

## 14. Problemas Comunes

- **RViz no se mueve:**
  Es probable que hayas lanzado un `demo.launch.py` aislado en lugar del `kuka_bridge_system.launch.py`. La GUI Dual requiere el traductor `kuka_moveit_bridge_node` para hablar con el Action Server de MoveIt.
  
- **El KUKA no se mueve pero la GUI dice que envió el XML:**
  1. Revisa que el programa en el SmartPad esté corriendo.
  2. Verifica que el Target no rompa la validación de `MAX_DELTA` del KRL (que el salto no sea gigante).
  3. Comprueba el log `ros2 topic echo /kuka/axis_move/raw_command_xml` para asegurar que `<EnableMove>1</EnableMove>`.

- **Deadlock de TCP (El bridge Python se satura):**
  Asegúrate de tener la versión actualizada de `kuka_eki_bridge`, donde el servidor Python limpia todo el *stream* de entrada y solo envía una única respuesta XML al final, evitando saturar el buffer del robot.

---

## 15. Notas Importantes

- Los ángulos de MoveIt2 trabajan internamente en **radianes**, pero la GUI y el nodo traductor están configurados para operar externamente en **grados**. Todas las entradas y salidas de tópicos personalizados en ROS2 usan grados por conveniencia humana.
- KUKA utiliza convenciones Euler (A, B, C) que corresponden a (Rz, Ry, Rx) y que pueden diferir de cómo MoveIt representa cuaterniones, por lo cual los nodos traductores de este repositorio incluyen matemática específica (`transform_utils.py`) para esta corrección.

---

## 16. Secuencias de Trayectorias

Capa **añadida** sobre el sistema existente. Permite crear, recibir,
almacenar, previsualizar y ejecutar secuencias de trayectorias sin cambiar
nada de lo que ya funcionaba: Axis Move, cartesiano, jog, HOME, límites,
DDS, TCP/IP, EKI, Submit, SPS, KRL, RViz y MoveIt siguen intactos.

Disponible en **ambas GUIs** (original y dual), mediante un widget
compartido, así que la lógica no está duplicada.

### 16.1 Flujo completo

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

A partir del archivo guardado, dos caminos **mutuamente separados**:

```text
trajectories/*.json          trajectories/*.json
 -> PROBAR TRAYECTORIA        -> ENVIAR TRAYECTORIA
 -> RViz únicamente           -> bridge TCP/IP existente
                              -> KUKA real
```

### 16.2 Controles

```text
SET   Puntos: N   SET ABRIR GARRA   SET CERRAR GARRA   LIMPIAR
ENVIAR PUNTOS (N)   PROBAR TRAYECTORIA   ENVIAR TRAYECTORIA   DETENER
(•) Manual   ( ) Automático
estado + log temporal
```

- **SET** captura `AxisActual` A1–A6 **real** del KUKA (no los campos de la
  GUI, no el target, no RViz, no XYZABC). Solo en memoria: **no escribe en
  disco**.
- **SET ABRIR/CERRAR GARRA** *programa* un evento anclado al último punto.
  **No mueve la garra.** Estado inicial siempre `open`.
- **ENVIAR PUNTOS** publica UNA solicitud con todos los puntos. Exige ≥ 2.
- **PROBAR TRAYECTORIA** reproduce en RViz. **Nunca alcanza al KUKA.**
- **ENVIAR TRAYECTORIA** ejecuta físicamente el archivo guardado, respetando
  `safe_mode`, `allow_motion_commands` y el checkbox `ENABLE MOVE`.
- **Manual** se detiene al final de cada **segmento** (no de cada punto
  intermedio) y pregunta CONTINUAR / CANCELAR. **Por defecto: Manual.**
- **Automático** encadena todos los segmentos; ante un error aborta.

### 16.3 Carpeta `trajectories/`

```text
/home/eduardex/Documents/TG2/trajectories
```

Configurable con `trajectories_dir` en `config/gui_dual_kuka_rviz.yaml` o con
la variable de entorno `KUKA_TRAJECTORIES_DIR`. **Nunca dentro de
`install/`.** Formato JSON, un archivo por secuencia:

```text
trajectory_sequence_20260822_153501.json
```

La trayectoria se guarda **tal cual llega**: no se eliminan puntos, no se
reinterpola, no se modifican tiempos y no se suaviza nada.

### 16.4 Nodo ROS2

Los cuatro publishers/subscribers nuevos **viven en el nodo ROS2 que ya
existía** (el del `RosAxisMoveBridge`, dentro de su executor). No se crea
ningún nodo nuevo ni uno por ventana, así que no reaparecen los nodos
homónimos ni el `Publisher count: 2`.

### 16.5 ⚠️ Limitación real del protocolo KRL/EKI

> [!IMPORTANT]
> `XmlDualMove.src` ejecuta **un `PTP` de parada exacta por cada `Seq`
> nuevo**, con `WAIT SEC 0.1` por vuelta de bucle y `$OV_PRO = 5`. Además,
> la memoria de recepción de EthernetKRL **cierra la conexión** a los 16
> elementos sin leer, y `Robot/LastMoveSeq` **no está** en el bloque `SEND`
> de `XmlDualMove.xml` (solo `Robot/MoveExecuted`, que es un pulso booleano
> sin número de secuencia asociado).
>
> Consecuencias, sin haber tocado nada del lado KUKA:
>
> - Una trayectoria densa se ejecuta como **N movimientos punto a punto con
>   parada completa**, no como un movimiento continuo mezclado.
> - `time_from_start` se **conserva en el archivo** pero **no puede usarse**
>   para marcar el ritmo: el tiempo lo impone el controlador.
> - `velocities_rad_s` y `accelerations_rad_s2` se guardan, pero el
>   protocolo actual no las transporta al robot.
> - La llegada a cada punto se confirma con `AxisActual` + el acuse de
>   recibo `Robot/RxCounter`, no con `MoveExecuted` por `Seq`.
>
> **No se ha modificado KRL, EKI, SPS ni el Submit Interpreter.** Levantar
> esta limitación exigiría cambios del lado del controlador que quedan
> deliberadamente fuera de alcance.

### 16.6 Documentación detallada

Contrato JSON completo, esquema del archivo guardado, validaciones, pacing,
manejo de errores y módulos nuevos:
[`src/kuka_gui_control/README.md` § 13](src/kuka_gui_control/README.md).
