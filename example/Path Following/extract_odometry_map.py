from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


@dataclass
class OdomSample:
    t_ns: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Extract odometry map and waypoints from a ROS2 bag.")
    p.add_argument(
        "--bag",
        type=Path,
        default=Path("TEST_COMBINE") / "rosbag",
        help="Path to rosbag folder containing metadata.yaml",
    )
    p.add_argument(
        "--odom-topic",
        default="/odom",
        help="Odometry topic name",
    )
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path("odometry_map"),
        help="Directory for CSV/JSON outputs",
    )
    p.add_argument(
        "--spacing-m",
        type=float,
        default=0.5,
        help="Distance spacing for distance waypoints (meters)",
    )
    p.add_argument(
        "--turn-deg",
        type=float,
        default=25.0,
        help="Heading change threshold for turn waypoints (degrees)",
    )
    p.add_argument(
        "--min-waypoint-gap-m",
        type=float,
        default=0.15,
        help="Minimum spacing between any two selected waypoints (meters)",
    )
    p.add_argument(
        "--close-loop",
        action="store_true",
        help="Append a final loop-closing waypoint at the start pose.",
    )
    p.add_argument(
        "--close-loop-min-gap-m",
        type=float,
        default=0.2,
        help="Only append loop-closing waypoint if start/end gap is at least this value.",
    )
    return p.parse_args()


def read_odom_samples(bag_dir: Path, odom_topic: str) -> list[OdomSample]:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    samples: list[OdomSample] = []

    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        conn_by_topic = {c.topic: c for c in reader.connections}
        if odom_topic not in conn_by_topic:
            available = sorted(conn_by_topic.keys())
            raise RuntimeError(f"Topic '{odom_topic}' not found. Available topics: {available}")

        conn = conn_by_topic[odom_topic]
        for _, t_ns, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            samples.append(
                OdomSample(
                    t_ns=t_ns,
                    x=float(p.x),
                    y=float(p.y),
                    z=float(p.z),
                    qx=float(q.x),
                    qy=float(q.y),
                    qz=float(q.z),
                    qw=float(q.w),
                    yaw=float(yaw),
                )
            )

    if not samples:
        raise RuntimeError("No odometry messages found.")
    return samples


def cumulative_distance(samples: list[OdomSample]) -> list[float]:
    d = [0.0]
    for i in range(1, len(samples)):
        dx = samples[i].x - samples[i - 1].x
        dy = samples[i].y - samples[i - 1].y
        d.append(d[-1] + math.hypot(dx, dy))
    return d


def select_waypoint_indices(
    samples: list[OdomSample],
    cumdist: list[float],
    spacing_m: float,
    turn_deg: float,
    min_gap_m: float,
) -> list[tuple[int, str]]:
    turn_rad = math.radians(turn_deg)
    picks: list[tuple[int, str]] = [(0, "start")]
    last_idx = 0
    last_pick_dist = cumdist[0]
    last_pick_yaw = samples[0].yaw

    for i in range(1, len(samples) - 1):
        dist_since_pick = cumdist[i] - last_pick_dist
        yaw_delta = abs(wrap_pi(samples[i].yaw - last_pick_yaw))

        reason = ""
        if dist_since_pick >= spacing_m:
            reason = "distance"
        if yaw_delta >= turn_rad and (cumdist[i] - cumdist[last_idx]) >= min_gap_m:
            reason = "turn"

        if reason:
            picks.append((i, reason))
            last_idx = i
            last_pick_dist = cumdist[i]
            last_pick_yaw = samples[i].yaw

    if picks[-1][0] != len(samples) - 1:
        picks.append((len(samples) - 1, "end"))

    # De-duplicate by index while preserving order.
    dedup: list[tuple[int, str]] = []
    seen = set()
    for idx, reason in picks:
        if idx not in seen:
            dedup.append((idx, reason))
            seen.add(idx)
    return dedup


def maybe_append_loop_closure(
    picks: list[tuple[int, str]],
    samples: list[OdomSample],
    close_loop: bool,
    close_loop_min_gap_m: float,
) -> list[tuple[int, str]]:
    if not close_loop or len(picks) < 2:
        return picks

    start_idx = picks[0][0]
    end_idx = picks[-1][0]
    start = samples[start_idx]
    end = samples[end_idx]
    gap = math.hypot(end.x - start.x, end.y - start.y)

    if gap >= close_loop_min_gap_m:
        return picks + [(start_idx, "loop_close")]
    return picks


def write_dense_csv(out_path: Path, samples: list[OdomSample], cumdist: list[float]) -> None:
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["idx", "t_ns", "x", "y", "z", "yaw_rad", "qx", "qy", "qz", "qw", "cumdist_m"])
        for i, s in enumerate(samples):
            w.writerow([
                i,
                s.t_ns,
                f"{s.x:.6f}",
                f"{s.y:.6f}",
                f"{s.z:.6f}",
                f"{s.yaw:.6f}",
                f"{s.qx:.8f}",
                f"{s.qy:.8f}",
                f"{s.qz:.8f}",
                f"{s.qw:.8f}",
                f"{cumdist[i]:.6f}",
            ])


def write_waypoint_csv(out_path: Path, samples: list[OdomSample], cumdist: list[float], picks: list[tuple[int, str]]) -> None:
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["wp_id", "sample_idx", "type", "t_ns", "x", "y", "z", "yaw_rad", "cumdist_m"])
        for wp_id, (idx, typ) in enumerate(picks):
            s = samples[idx]
            w.writerow([
                wp_id,
                idx,
                typ,
                s.t_ns,
                f"{s.x:.6f}",
                f"{s.y:.6f}",
                f"{s.z:.6f}",
                f"{s.yaw:.6f}",
                f"{cumdist[idx]:.6f}",
            ])


def write_summary(out_path: Path, samples: list[OdomSample], cumdist: list[float], picks: list[tuple[int, str]], args: argparse.Namespace) -> None:
    start = samples[0]
    end = samples[-1]
    start_end_gap = math.hypot(end.x - start.x, end.y - start.y)
    summary = {
        "bag": str(args.bag),
        "odom_topic": args.odom_topic,
        "total_odom_samples": len(samples),
        "total_path_length_m": cumdist[-1],
        "waypoint_count": len(picks),
        "spacing_m": args.spacing_m,
        "turn_deg": args.turn_deg,
        "min_waypoint_gap_m": args.min_waypoint_gap_m,
        "close_loop": args.close_loop,
        "close_loop_min_gap_m": args.close_loop_min_gap_m,
        "start_end_gap_m": start_end_gap,
        "loop_closed_waypoints": len(picks) >= 2 and picks[0][0] == picks[-1][0],
        "start_pose": {
            "x": start.x,
            "y": start.y,
            "z": start.z,
            "yaw_rad": start.yaw,
        },
        "end_pose": {
            "x": end.x,
            "y": end.y,
            "z": end.z,
            "yaw_rad": end.yaw,
        },
    }
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def write_plots(out_dir: Path, samples: list[OdomSample], cumdist: list[float], picks: list[tuple[int, str]]) -> None:
    x = [s.x for s in samples]
    y = [s.y for s in samples]
    yaw = [s.yaw for s in samples]

    wp_idx = [idx for idx, _ in picks]
    wp_x = [samples[idx].x for idx in wp_idx]
    wp_y = [samples[idx].y for idx in wp_idx]
    wp_types = [typ for _, typ in picks]

    # Path with waypoint overlay.
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(x, y, color="tab:blue", linewidth=1.5, label="odom path")
    ax.scatter(wp_x, wp_y, color="tab:orange", s=18, label="waypoints")
    ax.scatter([x[0]], [y[0]], color="green", s=60, marker="o", label="start")
    ax.scatter([x[-1]], [y[-1]], color="red", s=60, marker="x", label="end")
    ax.set_title("Odometry Path and Selected Waypoints")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_dir / "path_waypoints.png", dpi=160)
    plt.close(fig)

    # Yaw over traveled distance.
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(cumdist, yaw, color="tab:purple", linewidth=1.2)
    for idx, typ in picks:
        ax2.axvline(cumdist[idx], color="gray", linewidth=0.5, alpha=0.25)
        if typ == "turn":
            ax2.scatter([cumdist[idx]], [yaw[idx]], color="tab:red", s=14)
    ax2.set_title("Yaw vs Cumulative Distance")
    ax2.set_xlabel("distance [m]")
    ax2.set_ylabel("yaw [rad]")
    ax2.grid(True, alpha=0.25)
    fig2.tight_layout()
    fig2.savefig(out_dir / "yaw_vs_distance.png", dpi=160)
    plt.close(fig2)

    # Waypoint type counts.
    counts: dict[str, int] = {}
    for typ in wp_types:
        counts[typ] = counts.get(typ, 0) + 1
    labels = list(counts.keys())
    values = [counts[k] for k in labels]
    fig3, ax3 = plt.subplots(figsize=(6, 4))
    ax3.bar(labels, values, color=["tab:green", "tab:orange", "tab:red", "tab:blue"][: len(labels)])
    ax3.set_title("Waypoint Type Distribution")
    ax3.set_ylabel("count")
    ax3.grid(True, axis="y", alpha=0.2)
    fig3.tight_layout()
    fig3.savefig(out_dir / "waypoint_type_counts.png", dpi=160)
    plt.close(fig3)


def main() -> None:
    args = parse_args()
    bag_dir = args.bag
    if not bag_dir.exists():
        raise FileNotFoundError(f"Bag folder not found: {bag_dir}")

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    samples = read_odom_samples(bag_dir, args.odom_topic)
    cumdist = cumulative_distance(samples)
    picks = select_waypoint_indices(
        samples,
        cumdist,
        spacing_m=args.spacing_m,
        turn_deg=args.turn_deg,
        min_gap_m=args.min_waypoint_gap_m,
    )
    picks = maybe_append_loop_closure(
        picks,
        samples,
        close_loop=args.close_loop,
        close_loop_min_gap_m=args.close_loop_min_gap_m,
    )

    dense_csv = out_dir / "odom_dense.csv"
    wp_csv = out_dir / "waypoints.csv"
    summary_json = out_dir / "summary.json"

    write_dense_csv(dense_csv, samples, cumdist)
    write_waypoint_csv(wp_csv, samples, cumdist, picks)
    write_summary(summary_json, samples, cumdist, picks, args)
    write_plots(out_dir, samples, cumdist, picks)

    print(f"Wrote: {dense_csv}")
    print(f"Wrote: {wp_csv}")
    print(f"Wrote: {summary_json}")
    print(f"Wrote: {out_dir / 'path_waypoints.png'}")
    print(f"Wrote: {out_dir / 'yaw_vs_distance.png'}")
    print(f"Wrote: {out_dir / 'waypoint_type_counts.png'}")
    print(f"Samples: {len(samples)}")
    print(f"Path length: {cumdist[-1]:.2f} m")
    print(f"Waypoints: {len(picks)}")


if __name__ == "__main__":
    main()
