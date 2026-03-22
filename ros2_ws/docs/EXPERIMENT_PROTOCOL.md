# Mini-protocolo experimental (dissertação / paper)

Este documento descreve um fluxo **mínimo** e **repetível** usando o pipeline do repositório.

## Objetivo

Demonstrar que o sistema permite:

1. **Executar** o mesmo percurso várias vezes (simulação ou real).
2. **Coletar** dados sensoriais de forma **sincronizada** com o movimento (rosbag2).
3. **Quantificar** a **repetibilidade** entre execuções (trajetória de odometria, RMSE, erro no ponto final).

## Pré-requisitos

- ROS 2 Jazzy + workspace buildado (`colcon build --merge-install`).
- Simulação (ou robô real) com Nav2 e TF `map` → `base_link` coerente com o modo escolhido:
  - **Single-robot:** `robot_id` vazio, pastas `routes/default`, `collections/default`.
  - **Multi-robô:** namespaces `tb1`, `tb2`, … conforme README.

## Variáveis do experimento (registrar no texto)

| Variável | Exemplo | Notas |
|----------|---------|--------|
| Mundo / mapa | `turtlebot3_world` | Fixo entre execuções |
| Percurso | nome da rota, ex. `percurso1_tb1` | Mesmo YAML em todos os replays |
| Pontos (se usar `record` com waypoints) | string `--points` | Fixos entre sessões |
| Número de execuções | N ≥ 3 | Estatística mínima para repetibilidade |
| Tópicos coletados | `scan`, `odom`, `imu` | Iguais em todas as runs |

## Procedimento sugerido (N = 3, single-robot)

### 1. Subir stack

- Terminal 1: Gazebo + Nav2 (ou equivalente).
- Terminal 2: `ros2 launch fleet_orchestrator fleet.launch.py` com parâmetros do modo single-robot (ver README).

### 2. Gravar rota base + bag (execução 0 = referência)

```bash
source install/setup.bash
python3 scripts/experiment_repeatability.py record \
  --single-robot \
  --route percurso1_tb1 \
  --points "0.5,0,0;1.0,0,0;1.5,0.5,0;2.0,0.5,0" \
  --export run_record.json
```

Anotar o diretório do bag em `collections/default/<timestamp>/`.

### 3. Repetir percurso (execuções 1 … N−1)

Colocar o robô no **mesmo estado inicial** (sim: reset pose se necessário). Para cada repetição:

```bash
python3 scripts/experiment_repeatability.py replay \
  --single-robot \
  --route percurso1_tb1 \
  --export run_replay_01.json
```

Repetir com exports distintos (`run_replay_02`, …).

### 4. Análise

Passar os **diretórios dos bags** na ordem desejada. O **primeiro** da lista é a **referência** para `vs_reference` no `summary.json`.

```bash
pip install matplotlib   # opcional, para PNG
python3 scripts/analyze_runs.py \
  collections/default/<bag_record> \
  collections/default/<bag_replay1> \
  collections/default/<bag_replay2> \
  --output-dir analysis_exp01 \
  --labels record replay1 replay2
```

Saídas:

- `analysis_exp01/summary.json` — métricas, `pairwise_rmse_m`, `vs_reference` (RMSE, distância média ponto a ponto, erro no fim, razão de duração).
- `analysis_exp01/trajectories_csv/*.csv` — séries temporais `t_sec, x_m, y_m`.
- `analysis_exp01/trajectory_overlay.png` — figura para o documento.

## O que reportar no manuscrito

1. **Setup:** mundo, robô(s), modo (single vs multi), tópicos gravados.
2. **Protocolo:** N execuções, mesma rota, mesmo ponto inicial (ou política de reset).
3. **Métricas:** tabela com duração, comprimento do percurso, RMSE vs referência, erro no endpoint, distância média ponto a ponto (ver `summary.json`).
4. **Figura:** sobreposição de trajetórias (`trajectory_overlay.png`).
5. **Frase-tipo (inglês):**  
   *The orchestration pipeline enabled repeated execution of the same planned route with synchronized sensor logging. Pairwise trajectory comparison (odometry) and RMSE against a reference run characterize repeatability across independent replays.*

## Limitações (honestidade experimental)

- RMSE e distância ponto a ponto usam **subamostragem uniforme** entre trajetórias de comprimentos diferentes; não é DTW.
- Odometria reflete **odom publicado**, não ground-truth do simulador.
- Comparar `/scan` entre runs exige análise adicional (futuro).

## Extensões

- Multi-robô: mesma ideia com `--robot tb1` e bags em `collections/tb1/...`.
- MUUT/FUUT/SU: MUUT executa `record`/`replay`; FUUT/SU apenas coleta simultânea (scripts separados ou terminais paralelos).
