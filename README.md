# tp-integrador-ros2-nav2

## Build

```bash
colcon build
source install/setup.bash
```


### Terminal 1 — Simulación + SLAM

```bash
ros2 launch my_robot_navigation slam.launch.py
```

Este launch ya incluye la simulación (Gazebo) y RViz (con la config de `rviz/nav.rviz`), no hace falta lanzar nada de eso aparte.

### Terminal 2 — Planner (costmap global + planificador)

```bash
ros2 launch my_robot_navigation planner.launch.py
```

### Terminal 3 — Controller (costmap local + controller_server)

```bash
ros2 launch my_robot_navigation controller.launch.py
```

> Atajo: `ros2 launch my_robot_navigation navigation.launch.py` levanta planner + controller
> juntos en una sola terminal (reemplaza las Terminales 2 y 3 de arriba).

### Terminal 4 — Orquestador

```bash
ros2 launch my_robot_navigation orquestador.launch.py
```


El algoritmo del controller (`DWB`/`RPP`) se elige con el parámetro `controller_id`
(default `RPP`), configurable en tiempo de ejecución sin recompilar:

```bash
ros2 param set /orquestador_node controller_id DWB
```


