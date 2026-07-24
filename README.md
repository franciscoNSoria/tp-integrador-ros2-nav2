# tp-integrador-ros2-nav2

## Build

```bash
colcon build
source install/setup.bash
```


### Terminal 1 — Simulación + Localización (mapa ya construido)

```bash
ros2 launch my_robot_navigation localization.launch.py
```

Este launch ya incluye la simulación (Gazebo) y RViz (con la config de `rviz/nav.rviz`), no
hace falta lanzar nada de eso aparte. Usa el mapa `maps/my_living.yaml` (el de la Parte 1)
vía `map_server` + `amcl` — el robot navega sobre el mapa ya construido, cubre `13.5m x 9.4m`.

> Si en cambio querés mapear de cero (sin el mapa de la Parte 1), usá
> `ros2 launch my_robot_navigation slam.launch.py` en vez de este — mismo rol, pero arranca
> con el mapa vacío y lo construye en vivo a medida que el robot se mueve.

### Terminal 2 — Navegación (planner + controller)

```bash
ros2 launch my_robot_navigation navigation.launch.py
```

Incluye `planner.launch.py` + `controller.launch.py` juntos (si querés lanzarlos por separado
para debuggear uno solo, están disponibles individualmente con esos mismos nombres).

### Terminal 3 — Orquestador

```bash
ros2 launch my_robot_navigation orquestador.launch.py
```


El algoritmo del controller (`DWB`/`RPP`) se elige con el parámetro `controller_id`
(default `RPP`), configurable en tiempo de ejecución sin recompilar:

```bash
ros2 param set /orquestador_node controller_id DWB
```


