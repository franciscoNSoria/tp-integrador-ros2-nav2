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

## Probar el planner con una meta de prueba

Sin el orquestador 

```bash
ros2 topic echo /global_costmap/costmap --once --no-arr   # para ver origin/width/height y elegir un punto valido dentro del mapa

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}}}"
```

Resultado esperado: `error_code: 0` y `Goal finished with status: SUCCEEDED`, con la ruta
dibujándose en RViz sobre el display `Path` (`/plan`).



## Probar el controller con un path real (planner + controller encadenados)

```bash
ros2 run my_robot_navigation test_follow_path <x> <y> [DWB|RPP]

# ejemplo: ros2 run my_robot_navigation test_follow_path 1.5 0.0 RPP
```
 `<x> <y>` debe caer **dentro del área ya mapeada** por SLAM sino el planner falla con
`NO_VALID_PATH`

Resultado : el log muestra `distance_to_goal` bajando en cada feedback, el robot
se mueve en Gazebo, y publica en `cmd_vel` Termina con `status=4` (SUCCEEDED).

