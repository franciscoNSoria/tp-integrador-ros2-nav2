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



## Probar el planner con una meta de prueba

Sin el orquestador 
```bash
ros2 topic echo /global_costmap/costmap --once --no-arr   # para ver origin/width/height y elegir un punto valido dentro del mapa

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}}}"
```

Resultado esperado: `error_code: 0` y `Goal finished with status: SUCCEEDED`, con la ruta
dibujándose en RViz sobre el display `Path` (`/plan`).
