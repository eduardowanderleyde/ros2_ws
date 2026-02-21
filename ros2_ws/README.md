# ros2_ws — Workspace ROS 2 Jazzy (Linux)

Workspace ROS 2 Jazzy com **controle de robôs** e **coleta de dados** separados, prontos para UI.

## Estrutura (controle vs coleta)

- **Controle de robôs (rotas)** → `fleet_orchestrator`: record/save/play por `robot_id`, envio de goals Nav2.
- **Coleta de dados** → `fleet_data_collector`: ligar/desligar coleta de sensores (LiDAR, odom, imu) por `robot_id`.
- **Interfaces comuns** → `fleet_msgs`: serviços e mensagens usados por ambos (e pela UI depois).

Cada TurtleBot continua rodando no robô apenas o básico (drivers, Nav2, TF). Esta ferramenta roda no **PC central** e só consome/publica via ROS 2.

## Arquitetura: multi-mapa (fleet controller)

Cada robô tem **seu próprio SLAM e seu próprio mapa**. Não há mapa único compartilhado.

- **TF por robô:** `tb1/map`, `tb1/odom`, `tb1/base_link` — e idem para `tb2`, `tb3`.
- **Rotas não são compartilháveis:** uma rota em `routes/tb1/r1.yaml` é só do tb1 (frame `tb1/map`). Não serve para tb2.
- **Orchestrator** usa sempre frames e actions por robô:
  - `lookup_transform("tb1/map", "tb1/base_link")`
  - action `/tb1/navigate_through_poses`
- **Data collector** assina `/tb1/scan`, `/tb1/odom`, etc., e salva em `collections/tb1/<timestamp>/`.

Na UI, robôs são **instâncias independentes**: tabela por robô (map active, route, nav status, collection), sem nada global.

**Crítico:** cada robô deve rodar em namespace completo (`/tb1/...`). SLAM publica em `tbX/map`, Nav2 usa `tbX/map`. Conflito em `/tf` quebra tudo.

## Estrutura de pastas

```
ros2_ws/
  src/
    fleet_msgs/           # .srv comuns (UI + nós)
    fleet_orchestrator/   # record/save/play + Nav2 por robot_id
    fleet_data_collector/ # enable/disable coleta por robot_id (rosbag2)
    route_tool/           # legado: um robô, Trigger (mantido se quiser)
```

## Pacotes

| Pacote | Descrição |
|--------|-----------|
| **fleet_msgs** | `StartRecord`, `StopRecord`, `PlayRoute`, `Cancel`, `ListRobots`, `ListRoutes`, `EnableCollection`, `DisableCollection`, `CollectionStatus` |
| **fleet_orchestrator** | Nó que expõe start_record, stop_record, play_route, cancel, list_robots, list_routes (por `robot_id`) |
| **fleet_data_collector** | Nó que expõe enable_collection, disable_collection, collection_status (por `robot_id`); grava em **rosbag2** |
| **route_tool** | Nó legado (um robô, services Trigger) |

## Coleta: rosbag2 vs CSV

- **Implementado:** `output_mode = "rosbag2"`. Cada sessão vira uma pasta em `collections/<robot_id>/<timestamp>/` (mcap).
- **CSV:** não implementado; pode ser adicionado depois em `fleet_data_collector` tratando `output_mode == "csv"` e escrevendo CSVs por tópico/sessão.

## Build e run

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fleet_msgs fleet_orchestrator fleet_data_collector
source install/setup.bash
# Se o ros2 run não achar o pacote, use:
# export AMENT_PREFIX_PATH=$(pwd)/install/fleet_orchestrator:$(pwd)/install/fleet_data_collector:$AMENT_PREFIX_PATH
```

Terminal 1 – orquestrador:

```bash
ros2 run fleet_orchestrator fleet_orchestrator
```

Terminal 2 – coletor:

```bash
ros2 run fleet_data_collector sensor_collector
```

## Exemplos de chamadas (API para UI)

Robots padrão: `tb1`, `tb2`, `tb3`. Troque por seus namespaces.

**Rotas (orchestrator):**

```bash
ros2 service call /start_record fleet_msgs/srv/StartRecord "{robot_id: 'tb1', route_name: 'r1'}"
ros2 service call /stop_record fleet_msgs/srv/StopRecord "{robot_id: 'tb1'}"
ros2 service call /play_route fleet_msgs/srv/PlayRoute "{robot_id: 'tb1', route_name: 'r1'}"
ros2 service call /cancel fleet_msgs/srv/Cancel "{robot_id: 'tb1'}"
ros2 service call /list_robots fleet_msgs/srv/ListRobots "{}"
ros2 service call /list_routes fleet_msgs/srv/ListRoutes "{robot_id: 'tb1'}"
```

**Coleta (data collector):**

```bash
ros2 service call /enable_collection fleet_msgs/srv/EnableCollection "{robot_id: 'tb1', topics: ['scan', 'odom', 'imu'], output_mode: 'rosbag2'}"
ros2 service call /disable_collection fleet_msgs/srv/DisableCollection "{robot_id: 'tb1'}"
ros2 service call /collection_status fleet_msgs/srv/CollectionStatus "{robot_id: 'tb1'}"
```

## Parâmetros

- **fleet_orchestrator:** `robots`, `routes_dir`, `record_rate_hz`, `min_dist_m`, `min_yaw_deg`, `nav2_action_suffix`, `frame_map_suffix`, `frame_base_suffix`
- **fleet_data_collector:** `robots`, `collections_dir`. Tópicos conhecidos: `scan`, `odom`, `imu` (nomes relativos ao namespace do robô).

## Validação antes da UI (obrigatória)

Você precisa comprovar **3 invariantes** no ambiente real. Se algum falhar, a UI vai parecer que “não funciona”.

### 1) TF por robô existe e é estável

Se falhar, **record grava lixo**.

```bash
ros2 run tf2_ros tf2_echo tb1/map tb1/base_link
ros2 run tf2_ros tf2_echo tb2/map tb2/base_link
```

(Opcional: conferir que os tópicos de mapa existem por namespace: `ros2 topic list | grep tb1/map`, `ros2 topic list | grep tb2/map`.)

### 2) Action do Nav2 existe por robô

Se falhar, **play_route não faz nada**.

```bash
ros2 action list | grep navigate_through_poses
```

Deve aparecer `/tb1/navigate_through_poses`, `/tb2/...`, etc. (conforme seus namespaces.)

### 3) Coletor assina e grava bag sem travar

enable → espera 5–10 s → disable → conferir bag.

```bash
ros2 service call /enable_collection fleet_msgs/srv/EnableCollection "{robot_id: 'tb1', topics: ['scan', 'odom'], output_mode: 'rosbag2'}"
# esperar 5–10 s
ros2 service call /disable_collection fleet_msgs/srv/DisableCollection "{robot_id: 'tb1'}"
ros2 bag info collections/tb1/<timestamp>   # usar o dir que o disable retornou
```

Se travar no enable/disable, a UI vai “ligar coleta” e nada acontece.

### Conferir que a API está exposta

```bash
ros2 service list | grep -E "start_record|stop_record|play_route|enable_collection|disable_collection"
```

Deve listar os services do orchestrator e do collector.

---

## Checklist final antes de começar a UI

- [ ] **TF:** `tf2_echo tbX/map tbX/base_link` ok para cada robô.
- [ ] **Actions:** `ros2 action list | grep navigate_through_poses` mostra `/tb1/...`, `/tb2/...`, etc.
- [ ] **Record/Play:** start_record → anda → stop_record → gera YAML; play_route executa no Nav2.
- [ ] **Coleta:** enable_collection → espera → disable_collection → `ros2 bag info` no diretório ok.
- [ ] **Services:** `ros2 service list` inclui os services do fleet (start_record, play_route, enable_collection, etc.).

Só depois disso: implementar UI (recomendado: Web UI com FastAPI + WebSocket).

---

## Depois da validação (pronto pra UI)

Para a UI ficar simples e responsiva, o próximo passo da ferramenta é:

1. **Tópico `/fleet/status`** — msg agregada por robô: nav state (idle/running/succeeded/failed), rota ativa + waypoint atual, coleta on/off + caminho do bag, último erro. A UI assina um tópico em vez de fazer poll em vários services.
2. **PlayRoute observável** — feedback de progresso (e publicar no `/fleet/status`). Cancel já existe.
3. **Política de gravação de rota** — hoje: distância mínima (min_dist_m) e ângulo (min_yaw_deg). Opcional: documentar ou fixar “a cada X m” (ex.: 0,20 m) para consistência.

---

## Próximos passos (quando for fazer a UI)

- UI (web ou rqt) chama apenas esses serviços (e depois assina `/fleet/status`); não fala direto com Nav2 nem sensores.
- Possível adicionar `fleet_ui_api` (REST/WebSocket) e `fleet_bringup` (launch files) depois.
