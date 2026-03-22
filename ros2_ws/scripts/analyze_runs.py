#!/usr/bin/env python3
"""
Pós-processamento de rosbag2 gravados pelo fleet_data_collector: extrai trajetória de odometria,
calcula métricas (duração, comprimento, RMSE entre pares, erro no ponto final vs referência,
desvio médio ponto a ponto vs referência, razão de duração), exporta CSV por run e gera gráfico.

Uso (com workspace ROS 2 sourceado):
  source install/setup.bash
  python3 scripts/analyze_runs.py collections/default/run_a collections/default/run_b
  python3 scripts/analyze_runs.py "collections/default/*" --output-dir analysis_out

A **referência** é sempre o **primeiro** bag da lista (índice 0): métricas `vs_reference` comparam
as demais execuções a ela.

Dependências Python: numpy; matplotlib opcional para PNG (--no-plot se não tiver).

Requer: rosbag2 com mensagens nav_msgs (mesmo ambiente do colcon).
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def _need_ros():
    import rclpy  # noqa: F401
    from nav_msgs.msg import Odometry  # noqa: F401
    from rclpy.serialization import deserialize_message  # noqa: F401


@dataclass
class RunStats:
    label: str
    bag_path: str
    odom_topic: str
    num_poses: int
    duration_sec: float
    path_length_m: float
    start_xy: Tuple[float, float]
    end_xy: Tuple[float, float]


def _expand_inputs(paths: Sequence[str]) -> List[Path]:
    import glob

    out: List[Path] = []
    for p in paths:
        if any(c in p for c in "*?["):
            out.extend(Path(x) for x in sorted(glob.glob(p)))
        else:
            out.append(Path(p))
    # dedupe, exist only
    seen = set()
    uniq: List[Path] = []
    for p in out:
        rp = p.resolve()
        if rp in seen:
            continue
        seen.add(rp)
        if p.exists():
            uniq.append(p)
        else:
            print(f"[aviso] ignorando (não existe): {p}", file=sys.stderr)
    return uniq


def _open_reader(uri: str):
    import rosbag2_py

    opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    for storage_id in ("mcap", "sqlite3"):
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                opts,
            )
            return reader, storage_id
        except Exception:
            continue
    raise RuntimeError(f"Não foi possível abrir o bag: {uri} (tente mcap/sqlite3)")


def _find_odom_topic(reader) -> str:
    for meta in reader.get_all_topics_and_types():
        name = meta.name
        typ = meta.type
        if "Odometry" in typ and "odom" in name.lower():
            return name
    raise RuntimeError("Nenhum tópico de odometria (nav_msgs/Odometry) encontrado no bag.")


def _read_odom_xy(
    bag_dir: Path,
) -> Tuple[str, np.ndarray, np.ndarray, np.ndarray]:
    """Retorna (topic, t_sec, x, y)."""
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    uri = str(bag_dir.resolve())
    reader, _storage = _open_reader(uri)
    topic = _find_odom_topic(reader)
    filt = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filt)

    ts: List[float] = []
    xs: List[float] = []
    ys: List[float] = []

    while reader.has_next():
        nxt = reader.read_next()
        if len(nxt) == 2:
            tname, data = nxt
        else:
            tname, data = nxt[0], nxt[1]
        msg = deserialize_message(data, Odometry)
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ts.append(sec)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)

    if not ts:
        raise RuntimeError(f"Nenhuma mensagem de odometria em {topic}")

    t0 = ts[0]
    t_rel = np.array([t - t0 for t in ts], dtype=np.float64)
    return topic, t_rel, np.array(xs, dtype=np.float64), np.array(ys, dtype=np.float64)


def _path_length(x: np.ndarray, y: np.ndarray) -> float:
    if len(x) < 2:
        return 0.0
    dx = np.diff(x)
    dy = np.diff(y)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))


def _resample_pair(
    xy1: np.ndarray, xy2: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Mesmo número de pontos (subamostragem uniforme) para comparar trajetórias."""
    n = min(len(xy1), len(xy2))
    if n < 2:
        return xy1, xy2
    i1 = np.linspace(0, len(xy1) - 1, n).astype(int)
    i2 = np.linspace(0, len(xy2) - 1, n).astype(int)
    return xy1[i1], xy2[i2]


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


def _mean_pointwise_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Distância euclidiana média entre pontos homólogos (mesmo N)."""
    d = np.sqrt(np.sum((a - b) ** 2, axis=1))
    return float(np.mean(d))


def _safe_label_file(s: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in s)


def _write_trajectory_csv(path: Path, t: np.ndarray, xy: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t_sec,x_m,y_m\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.6f},{xy[i, 0]:.6f},{xy[i, 1]:.6f}\n")


def analyze_bag(label: str, bag_dir: Path) -> Tuple[RunStats, np.ndarray, np.ndarray]:
    topic, t, x, y = _read_odom_xy(bag_dir)
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    plen = _path_length(x, y)
    xy = np.column_stack([x, y])
    stats = RunStats(
        label=label,
        bag_path=str(bag_dir.resolve()),
        odom_topic=topic,
        num_poses=len(x),
        duration_sec=duration,
        path_length_m=plen,
        start_xy=(float(x[0]), float(y[0])),
        end_xy=(float(x[-1]), float(y[-1])),
    )
    return stats, xy, t


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analisa rosbag2 do fleet: odometria, métricas e gráfico de trajetórias."
    )
    parser.add_argument(
        "bags",
        nargs="+",
        help="Pastas de rosbag (uma por run), ex.: collections/default/run1 ou glob entre aspas",
    )
    parser.add_argument(
        "--output-dir",
        default="analysis_out",
        help="Onde salvar summary.json e trajectory_overlay.png",
    )
    parser.add_argument(
        "--labels",
        nargs="*",
        help="Rótulos na legenda (mesma ordem dos bags); default run_0, run_1, ...",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Não gera PNG (útil sem matplotlib)",
    )
    parser.add_argument(
        "--no-csv",
        action="store_true",
        help="Não grava trajectory_<label>.csv por run",
    )
    args = parser.parse_args()

    try:
        _need_ros()
    except ImportError as e:
        print(f"Erro: ambiente ROS 2 não disponível: {e}", file=sys.stderr)
        print("Execute: source /opt/ros/jazzy/setup.bash && source install/setup.bash", file=sys.stderr)
        return 2

    bag_paths = _expand_inputs(args.bags)
    if not bag_paths:
        print("Nenhum caminho válido.", file=sys.stderr)
        return 2

    labels = args.labels or [f"run_{i}" for i in range(len(bag_paths))]
    if len(labels) != len(bag_paths):
        print("--labels deve ter o mesmo número de entradas que os bags.", file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_stats: List[RunStats] = []
    trajectories: List[np.ndarray] = []
    times: List[np.ndarray] = []

    for label, bdir in zip(labels, bag_paths):
        print(f"Lendo {label} ... ({bdir})")
        try:
            st, xy, t_rel = analyze_bag(label, bdir)
            all_stats.append(st)
            trajectories.append(xy)
            times.append(t_rel)
        except Exception as ex:
            print(f"[FALHOU] {bdir}: {ex}", file=sys.stderr)
            return 1

    if not args.no_csv:
        csv_dir = out_dir / "trajectories_csv"
        for t_rel, xy, lab in zip(times, trajectories, labels):
            fn = csv_dir / f"trajectory_{_safe_label_file(lab)}.csv"
            _write_trajectory_csv(fn, t_rel, xy)
        print(f"[OK] CSVs: {csv_dir}/")

    # RMSE pairwise
    n = len(trajectories)
    rmse_matrix: List[List[Optional[float]]] = [
        [None] * n for _ in range(n)
    ]
    for i in range(n):
        for j in range(i + 1, n):
            a, b = _resample_pair(trajectories[i], trajectories[j])
            r = _rmse(a, b)
            rmse_matrix[i][j] = r
            rmse_matrix[j][i] = r
        rmse_matrix[i][i] = 0.0

    # vs primeira execução (referência)
    ref_idx = 0
    d0 = all_stats[ref_idx].duration_sec
    d0 = d0 if d0 > 1e-6 else 1e-6
    end_ref = np.array(all_stats[ref_idx].end_xy, dtype=np.float64)
    vs_reference: List[dict] = []
    for i in range(n):
        end_i = np.array(all_stats[i].end_xy, dtype=np.float64)
        final_err = float(np.linalg.norm(end_i - end_ref))
        dur_ratio = float(all_stats[i].duration_sec / d0)
        entry: dict = {
            "label": labels[i],
            "final_endpoint_error_m": final_err,
            "duration_sec": all_stats[i].duration_sec,
            "duration_ratio_vs_ref": dur_ratio,
        }
        if i != ref_idx:
            a, b = _resample_pair(trajectories[ref_idx], trajectories[i])
            entry["rmse_vs_ref_m"] = _rmse(a, b)
            entry["mean_pointwise_distance_vs_ref_m"] = _mean_pointwise_distance(a, b)
        else:
            entry["rmse_vs_ref_m"] = 0.0
            entry["mean_pointwise_distance_vs_ref_m"] = 0.0
        vs_reference.append(entry)

    def _run_to_json(s: RunStats) -> dict:
        d = asdict(s)
        d["start_xy"] = list(d["start_xy"])
        d["end_xy"] = list(d["end_xy"])
        return d

    summary = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "reference_run_index": ref_idx,
        "reference_label": labels[ref_idx],
        "runs": [_run_to_json(s) for s in all_stats],
        "pairwise_rmse_m": rmse_matrix,
        "vs_reference": vs_reference,
        "labels": labels,
    }
    summary_path = out_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n[OK] Resumo: {summary_path}")

    if not args.no_plot:
        if not HAS_MPL:
            print("[aviso] matplotlib não instalado; use: pip install matplotlib  ou  --no-plot")
        else:
            fig, ax = plt.subplots(figsize=(8, 8))
            for st, xy, lab in zip(all_stats, trajectories, labels):
                ax.plot(xy[:, 0], xy[:, 1], label=lab, linewidth=1.5)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title("Trajetórias (odom) — comparação entre runs")
            ax.legend(loc="best", fontsize=9)
            ax.grid(True, alpha=0.3)
            # Caixa de texto: RMSE vs referência (primeiro bag)
            if n > 1:
                lines = [f"Ref: {labels[0]}"]
                for i in range(1, n):
                    r = vs_reference[i].get("rmse_vs_ref_m", 0.0)
                    fe = vs_reference[i].get("final_endpoint_error_m", 0.0)
                    lines.append(f"{labels[i]}: RMSE={r:.3f}m  Δfim={fe:.3f}m")
                ax.text(
                    0.02,
                    0.98,
                    "\n".join(lines),
                    transform=ax.transAxes,
                    fontsize=8,
                    verticalalignment="top",
                    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
                )
            png_path = out_dir / "trajectory_overlay.png"
            fig.savefig(png_path, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"[OK] Gráfico: {png_path}")

    # Imprime tabela rápida
    print("\n--- Métricas por run ---")
    for s in all_stats:
        print(
            f"  {s.label}: duração={s.duration_sec:.2f}s  comprimento={s.path_length_m:.3f}m  "
            f"poses={s.num_poses}  topic={s.odom_topic}"
        )
    if n > 1:
        print("\n--- RMSE (m) entre pares (subamostragem uniforme) ---")
        for i in range(n):
            for j in range(i + 1, n):
                r = rmse_matrix[i][j]
                print(f"  {labels[i]} vs {labels[j]}: {r:.4f} m")
        print(f"\n--- Vs referência ({labels[0]}) ---")
        for v in vs_reference:
            print(
                f"  {v['label']}: RMSE={v['rmse_vs_ref_m']:.4f} m  "
                f"dist_média_pt={v['mean_pointwise_distance_vs_ref_m']:.4f} m  "
                f"Δfim={v['final_endpoint_error_m']:.4f} m  "
                f"duração×={v['duration_ratio_vs_ref']:.3f}"
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
