#!/usr/bin/env python3
"""
Add plottable /odom data into an existing .rrd recording.

The MCAP->RRD converter stores nav_msgs/Odometry as one opaque blob per
message, so Rerun's viewer can't natively plot it. This script decodes
/odom from the original MCAP bag and writes native, plottable entities
under /odom_plot/* (a static path line, a per-timestep position marker,
and linear/angular velocity scalars) into a *new* recording that shares
the target .rrd's store id, then merges the two together so the result
is the same recording with /odom_plot/* added. Every other entity in the
.rrd passes through the merge untouched.

It also embeds a blueprint (viewport layout) that explicitly defines views
for /odom_plot/* only, and marks it active/default so it shows up on open
even if the viewer has a cached layout from a previous session with this
same recording. Every other entity (camera, joint_states, imu, ...) is left
to the viewer's own automatic layout (auto_views=True) -- the script's
blueprint doesn't reference or hardcode anything outside of /odom_plot/*.

Usage:
  python3 plot_odometry.py <path-to-rosbag_dir-or-.mcap> --rrd <path/to/existing.rrd> [--topic /odom]

Example:
  python3 plot_odometry.py TEST_COMBINED_LEFT/rosbag --rrd TEST_COMBINED_LEFT.rrd
"""

import argparse
import math
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from mcap_ros2.reader import read_ros2_messages


def find_mcap_files(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    files = sorted(source.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"No .mcap files found under {source}")
    return files


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_odometry(mcap_files: list[Path], topic: str) -> dict[str, np.ndarray]:
    t_ns, x, y, yaw, lin_vel, ang_vel = [], [], [], [], [], []

    for mcap_file in mcap_files:
        for msg in read_ros2_messages(str(mcap_file), topics=[topic]):
            odom = msg.ros_msg
            t_ns.append(msg.log_time_ns)
            pos = odom.pose.pose.position
            x.append(pos.x)
            y.append(pos.y)
            q = odom.pose.pose.orientation
            yaw.append(yaw_from_quaternion(q.x, q.y, q.z, q.w))
            lin_vel.append(odom.twist.twist.linear.x)
            ang_vel.append(odom.twist.twist.angular.z)

    if not t_ns:
        raise ValueError(f"No messages found on topic '{topic}'")

    return {
        "t_ns": np.array(t_ns, dtype=np.int64),
        "x": np.array(x),
        "y": np.array(y),
        "yaw": np.array(yaw),
        "lin_vel": np.array(lin_vel),
        "ang_vel": np.array(ang_vel),
    }


def get_store_id(rrd_path: Path) -> tuple[str, str]:
    """Extract (application_id, recording_id) from an existing .rrd's StoreId."""
    result = subprocess.run(
        ["rerun", "rrd", "print", str(rrd_path)],
        capture_output=True, text=True, check=True,
    )
    match = re.search(
        r'StoreId\(\s*Recording,\s*"([^"]+)",\s*"([^"]+)"',
        result.stdout,
    )
    if not match:
        raise RuntimeError(f"Could not find a StoreId in {rrd_path}")
    return match.group(1), match.group(2)


def refuse_if_blueprint_exists(rrd_path: Path) -> None:
    """Raise if rrd_path already has a Blueprint store from a previous run of this script.

    A prior attempt at "clean up the old blueprint automatically" used `rerun rrd filter
    --drop-entity` to remove the old blueprint's view/container/viewport entities. That only
    strips entity *content* -- it can't remove the store's own metadata or its
    BlueprintActivationCommand. With that leftover (now-empty) store still present and still
    claiming to be active/default, the viewer picked it over the new, real blueprint and showed
    a completely empty viewport -- worse than the original stale-layout problem. There is no
    supported way (CLI or SDK) to fully delete a prior blueprint store, so the safe option is to
    refuse and ask for a clean copy instead of silently corrupting the layout.
    """
    result = subprocess.run(
        ["rerun", "rrd", "print", str(rrd_path)],
        capture_output=True, text=True, check=True,
    )
    if re.search(r"StoreId\(\s*Blueprint,", result.stdout):
        raise RuntimeError(
            f"{rrd_path} already has a blueprint embedded (likely from a previous run of this "
            "script). Re-running against it is not safe -- please start from a clean copy of "
            "the original .rrd (before any plot_odometry.py runs) instead."
        )


def build_blueprint() -> rrb.Blueprint:
    """Explicitly define views for /odom_plot/* only; everything else is left to auto_views."""
    grows_with_cursor = rrb.VisibleTimeRange(
        "message_log_time",
        start=rrb.TimeRangeBoundary.infinite(),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    return rrb.Blueprint(
        rrb.Grid(
            rrb.Spatial2DView(
                origin="/odom_plot",
                name="Odom plot",
                contents=["/odom_plot/path", "/odom_plot/position"],
                time_ranges=grows_with_cursor,
            ),
            rrb.TimeSeriesView(origin="/odom_plot/linear_velocity", name="Odom / linear velocity"),
            rrb.TimeSeriesView(origin="/odom_plot/angular_velocity", name="Odom / angular velocity"),
        ),
        auto_views=True,
    )


def write_odom_plot_recording(data: dict[str, np.ndarray], app_id: str, recording_id: str, out_path: Path) -> None:
    rr.init(application_id=app_id, recording_id=recording_id, spawn=False, send_properties=False)
    rr.save(str(out_path))

    for i in range(len(data["t_ns"])):
        rr.set_time("message_log_time", timestamp=np.datetime64(int(data["t_ns"][i]), "ns"))
        if i > 0:
            segment = [[data["x"][i - 1], data["y"][i - 1]], [data["x"][i], data["y"][i]]]
            rr.log("/odom_plot/path", rr.LineStrips2D([segment]))
        rr.log("/odom_plot/position", rr.Points2D([[data["x"][i], data["y"][i]]]))
        rr.log("/odom_plot/linear_velocity", rr.Scalars(data["lin_vel"][i]))
        rr.log("/odom_plot/angular_velocity", rr.Scalars(data["ang_vel"][i]))

    rr.send_blueprint(build_blueprint(), make_active=True, make_default=True)


def merge_into_rrd(new_recording_path: Path, target_rrd: Path) -> None:
    with tempfile.NamedTemporaryFile(suffix=".rrd", delete=False, dir=target_rrd.parent) as tmp:
        merged_path = Path(tmp.name)
    subprocess.run(
        ["rerun", "rrd", "merge", str(target_rrd), str(new_recording_path), "-o", str(merged_path)],
        check=True,
    )
    shutil.move(str(merged_path), str(target_rrd))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", type=Path, help="Path to a rosbag directory or a single .mcap file")
    parser.add_argument("--rrd", type=Path, required=True, help="Existing .rrd file to add /odom_plot/* entities into")
    parser.add_argument("--topic", default="/odom", help="Odometry topic to decode (default: /odom)")
    args = parser.parse_args()

    if not args.rrd.is_file():
        raise FileNotFoundError(f"Target .rrd not found: {args.rrd}")

    refuse_if_blueprint_exists(args.rrd)

    mcap_files = find_mcap_files(args.source)
    data = load_odometry(mcap_files, args.topic)
    print(f"Decoded {len(data['t_ns'])} messages from {len(mcap_files)} file(s) on topic '{args.topic}'")

    app_id, recording_id = get_store_id(args.rrd)
    print(f"Target recording: application_id={app_id!r} recording_id={recording_id!r}")

    with tempfile.NamedTemporaryFile(suffix=".rrd", delete=False) as tmp:
        new_recording_path = Path(tmp.name)
    try:
        write_odom_plot_recording(data, app_id, recording_id, new_recording_path)
        merge_into_rrd(new_recording_path, args.rrd)
    finally:
        new_recording_path.unlink(missing_ok=True)

    print(f"Added /odom_plot/path, /odom_plot/position, /odom_plot/linear_velocity, "
          f"/odom_plot/angular_velocity into {args.rrd}")


if __name__ == "__main__":
    main()