# ros2_ws — Workspace ROS 2 Jazzy (Linux)

Este diretório é um **workspace ROS 2 para Linux (Jazzy)**, bem simples.

Estrutura principal:

- `ros2_ws/`
  - `README.md` ← este arquivo, explicando tudo
  - `src/`
    - `route_tool/` ← pacote Python ROS 2
      - `package.xml` ← manifesto do pacote (nome, dependências, etc.)
      - `setup.py`    ← instalação do pacote e entry point `route_tool`
      - `setup.cfg`   ← onde os scripts Python serão instalados
      - `resource/route_tool` ← registro do pacote no ament index
      - `route_tool/`
        - `__init__.py` ← marca o diretório como módulo Python
        - `main.py`     ← nó ROS 2 com TF + services

## Como usar no Linux (ROS 2 Jazzy)

No Linux (onde o ROS 2 Jazzy está instalado):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select route_tool
source install/setup.bash
ros2 run route_tool route_tool
```

## Services disponíveis (no Linux)

O nó `route_tool` expõe **3 services do tipo `std_srvs/Trigger`**:

- `start_route` → começa a gravar a rota (poses em `map` do `base_link`)
- `stop_route`  → para a gravação e mantém a rota em memória
- `play_route`  → reproduz a rota publicando as poses gravadas em `route_pose`

Exemplo de chamada (no Linux, em outro terminal com o ambiente já `source`-ado):

```bash
ros2 service call /start_route std_srvs/srv/Trigger {}
ros2 service call /stop_route  std_srvs/srv/Trigger {}
ros2 service call /play_route  std_srvs/srv/Trigger {}
```

