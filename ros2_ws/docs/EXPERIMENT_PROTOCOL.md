# Mini-protocolo experimental (dissertação / paper)

Este documento descreve um fluxo **mínimo** e **repetível** usando o pipeline do repositório.

## Objetivo

Demonstrar que o sistema permite:

1. **Executar** o mesmo percurso várias vezes (simulação ou real).
2. **Coletar** dados sensoriais de forma **sincronizada** com o movimento (rosbag2).
3. **Quantificar** a **repetibilidade** entre execuções (trajetória de odometria, RMSE, erro no ponto final).

## O que este experimento já evidencia (no mínimo)

- A rota pode ser **gravada** e **reproduzida**.
- A **coleta** ocorre em conjunto com a navegação.
- Existe **repetibilidade mensurável** entre execuções independentes (métricas no `summary.json` e figura).

## Pré-requisitos

- ROS 2 Jazzy + workspace buildado (`colcon build --merge-install`).
- Simulação (ou robô real) com Nav2 e TF `map` → `base_link` coerente com o modo escolhido:
  - **Single-robot:** `robot_id` vazio, pastas `routes/default`, `collections/default`.
  - **Multi-robô:** namespaces `tb1`, `tb2`, … conforme README.

### Figura (`trajectory_overlay.png`): matplotlib

**Prefira testar antes de instalar nada:**

```bash
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

Se falhar, aí sim use o gestor de pacotes do sistema ou um venv (ex.: `pip install matplotlib` num ambiente dedicado).

## Variáveis do experimento (registrar no texto)

| Variável | Exemplo | Notas |
|----------|---------|--------|
| Mundo / mapa | `turtlebot3_world` | Fixo entre execuções |
| Percurso | nome da rota, ex. `percurso1_tb1` | Mesmo YAML em todos os replays |
| Pontos (se usar `record` com waypoints) | string `--points` | Fixos entre sessões |
| Número de execuções | N ≥ 3 | Estatística mínima para repetibilidade |
| Tópicos coletados | `scan`, `odom`, `imu` | Iguais em todas as runs |

## Estado inicial idêntico entre replays (crítico)

Antes de **cada** `replay`, garantir:

- Mesmo **ponto inicial** (posição).
- Mesma **orientação** inicial.
- Mesmo **mapa** e localização **estável** (AMCL/SLAM coerente com o protocolo).

Caso contrário, RMSE e métricas vs referência misturam **erro de repetição do percurso** com **erro de inicialização**. Em simulação, use reset de pose conforme o teu fluxo (e descreve isso no manuscrito).

## Procedimento (visão por etapas)

### Etapa A — gerar os dados

1. **Subir stack:** Terminal 1: Gazebo + Nav2; Terminal 2: `ros2 launch fleet_orchestrator fleet.launch.py single_robot_sim:=true` (não use `--params-file` no `ros2 launch`; ver README).

2. **Uma gravação (baseline / referência):**

```bash
source install/setup.bash
python3 scripts/experiment_repeatability.py record \
  --single-robot \
  --route percurso1_tb1 \
  --points "0.5,0,0;1.0,0,0;1.5,0.5,0;2.0,0.5,0" \
  --export run_record.json
```

3. **Duas ou três repetições** (`replay`), com `--export` distinto por run e **estado inicial** alinhado ao protocolo:

```bash
python3 scripts/experiment_repeatability.py replay \
  --single-robot \
  --route percurso1_tb1 \
  --export run_replay_01.json

python3 scripts/experiment_repeatability.py replay \
  --single-robot \
  --route percurso1_tb1 \
  --export run_replay_02.json
```

### Anotar o bag após cada execução

Para não perder a correspondência run → pasta do rosbag:

- O JSON de `--export` inclui **`rosbag_path`** quando o `disable_collection` devolve o caminho do bag (parse da mensagem do coletor).
- Complemento rápido na consola:

```bash
ls -lt collections/default/ | head
```

### Etapa B — analisar

O **primeiro** diretório passado ao `analyze_runs.py` é a **referência** em `vs_reference` no `summary.json`.

Labels **claros para paper** (exemplo):

```bash
python3 scripts/analyze_runs.py \
  collections/default/<bag_baseline> \
  collections/default/<bag_replay_01> \
  collections/default/<bag_replay_02> \
  --output-dir analysis_exp01 \
  --labels baseline replay_01 replay_02
```

Saídas:

- `analysis_exp01/summary.json` — métricas, `pairwise_rmse_m`, `vs_reference` (RMSE, distância média ponto a ponto, erro no fim, razão de duração).
- `analysis_exp01/trajectories_csv/*.csv` — séries `t_sec, x_m, y_m`.
- `analysis_exp01/trajectory_overlay.png` — figura para o documento.

### Etapa C — congelar resultado (rastreabilidade)

Copiar para uma pasta estável (ex. `results/dissertation_run01/`):

- `summary.json`, `trajectory_overlay.png`, pasta `trajectories_csv/`
- JSONs das execuções (`run_record.json`, `run_replay_01.json`, …)
- **`docs/EXPERIMENT_PROTOCOL.md`** (ou referência ao commit)
- **YAML da rota** usada (rastreabilidade do percurso):

```bash
mkdir -p results/dissertation_run01
cp docs/EXPERIMENT_PROTOCOL.md results/dissertation_run01/
cp analysis_exp01/summary.json analysis_exp01/trajectory_overlay.png results/dissertation_run01/
cp -r analysis_exp01/trajectories_csv results/dissertation_run01/
cp run_record.json run_replay_01.json run_replay_02.json results/dissertation_run01/
cp routes/default/percurso1_tb1.yaml results/dissertation_run01/
```

(Ajusta o nome do YAML se a rota for outra.)

## O que reportar no manuscrito

1. **Setup:** mundo, robô(s), modo (single vs multi), tópicos gravados.
2. **Protocolo:** N execuções, mesma rota, política de **estado inicial** (reset de pose, etc.).
3. **Métricas:** tabela com duração, comprimento do percurso, RMSE vs referência, erro no endpoint, distância média ponto a ponto (ver `summary.json`).
4. **Figura:** sobreposição de trajetórias (`trajectory_overlay.png`), com legenda alinhada aos labels (`baseline`, `replay_01`, …).
5. **Frase-tipo (inglês):**  
   *The orchestration pipeline enabled repeated execution of the same planned route with synchronized sensor logging. Pairwise trajectory comparison (odometry) and RMSE against a reference run characterize repeatability across independent replays.*

## Limitações (honestidade experimental)

- RMSE e distância ponto a ponto usam **subamostragem uniforme** entre trajetórias de comprimentos diferentes; não é DTW.
- Odometria reflete **odom publicado**, não ground-truth do simulador.
- Comparar `/scan` entre runs exige análise adicional (futuro).

## Extensões

- Multi-robô: mesma ideia com `--robot tb1` e bags em `collections/tb1/...`.
- MUUT/FUUT/SU: MUUT executa `record`/`replay`; FUUT/SU apenas coleta simultânea (scripts separados ou terminais paralelos).
