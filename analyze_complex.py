#!/usr/bin/env python3

import os
import re
import csv
import math
import bisect
import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_ROOT = os.path.expanduser("~/limo_project/experiment_bags")
OUT_CSV = os.path.expanduser(
    "~/limo_project/analysis/complex_results.csv"
)

GOALS = {
    "A": (3.65751051902771, 0.9889817833900452, 0.0),
    "B": (3.652413845062256, -1.3845810890197754, 0.0),
}

# 현장에서 직접 관찰한 특이사항
MANUAL = {
    "complex_wheel_A_trial4": {
        "temporary_stuck": "YES",
        "contact": "NO",
        "observation": "장시간 정체 후 자력으로 재개하여 최종 도착"
    },

    "complex_wheel_B_trial1": {
        "temporary_stuck": "YES",
        "contact": "NO",
        "observation": "진행 불가로 실험 종료; 현장 판정 FAIL"
    },

    "complex_wheel_B_trial3": {
        "temporary_stuck": "NO",
        "contact": "NO",
        "observation": "Goal 접근 구간에서 pose/localization 큰 불안정 후 최종 도착"
    },

    "complex_fusion_A_trial5": {
        "temporary_stuck": "NO",
        "contact": "YES",
        "observation": "주행 중 구조물 접촉 후 최종 도착"
    },
}


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def transform_pose(x, y, yaw, tf):
    tx, ty, tyaw = tf

    c = math.cos(tyaw)
    s = math.sin(tyaw)

    mx = tx + c*x - s*y
    my = ty + s*x + c*y
    myaw = normalize_angle(yaw + tyaw)

    return mx, my, myaw


def nearest_value(ts, times, values):
    if not times:
        return None

    i = bisect.bisect_left(times, ts)

    if i == 0:
        return values[0]

    if i >= len(times):
        return values[-1]

    if abs(ts-times[i-1]) <= abs(times[i]-ts):
        return values[i-1]

    return values[i]


def active_plan(ts, plan_times, plans):
    if not plan_times:
        return None

    i = bisect.bisect_right(plan_times, ts) - 1

    if i < 0:
        return None

    return plans[i]


def point_to_polyline_distance(px, py, path):
    if len(path) < 2:
        return float("nan")

    best = float("inf")

    for i in range(len(path)-1):
        ax, ay = path[i]
        bx, by = path[i+1]

        vx = bx - ax
        vy = by - ay
        wx = px - ax
        wy = py - ay

        vv = vx*vx + vy*vy

        if vv < 1e-12:
            d = math.hypot(px-ax, py-ay)

        else:
            t = (wx*vx + wy*vy) / vv
            t = max(0.0, min(1.0, t))

            qx = ax + t*vx
            qy = ay + t*vy

            d = math.hypot(px-qx, py-qy)

        best = min(best, d)

    return best


def open_reader(path):
    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=path,
            storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    return reader, types


def analyze(name, path, mode, goal, trial):

    reader, topic_types = open_reader(path)

    odom = []

    tf_times = []
    tf_values = []

    plan_times = []
    plans = []

    navigation_times = []
    recoveries = []

    status_history = []

    while reader.has_next():

        topic, data, timestamp = reader.read_next()

        if topic not in topic_types:
            continue

        try:
            msg = deserialize_message(
                data,
                get_message(topic_types[topic])
            )
        except Exception:
            continue

        # map -> odom
        if topic == "/tf":

            for tr in msg.transforms:

                parent = tr.header.frame_id.lstrip("/")
                child = tr.child_frame_id.lstrip("/")

                if parent == "map" and child == "odom":

                    tf_times.append(timestamp)

                    tf_values.append((
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        quat_to_yaw(tr.transform.rotation)
                    ))

        elif topic == "/odometry/filtered":

            p = msg.pose.pose.position

            odom.append((
                timestamp,
                p.x,
                p.y,
                quat_to_yaw(msg.pose.pose.orientation)
            ))

        elif topic == "/plan":

            pts = [
                (
                    p.pose.position.x,
                    p.pose.position.y
                )
                for p in msg.poses
            ]

            if len(pts) >= 2:
                plan_times.append(timestamp)
                plans.append(pts)

        elif topic == "/navigate_to_pose/_action/feedback":

            fb = msg.feedback

            navigation_times.append(
                fb.navigation_time.sec +
                fb.navigation_time.nanosec * 1e-9
            )

            recoveries.append(
                int(fb.number_of_recoveries)
            )

        elif topic == "/navigate_to_pose/_action/status":

            for s in msg.status_list:
                status_history.append(
                    (timestamp, int(s.status))
                )

    # sort TF
    if tf_times:

        idx = np.argsort(tf_times)

        tf_times = [tf_times[i] for i in idx]
        tf_values = [tf_values[i] for i in idx]

    # sort plans
    if plan_times:

        idx = np.argsort(plan_times)

        plan_times = [plan_times[i] for i in idx]
        plans = [plans[i] for i in idx]

    # map-frame trajectory
    trajectory = []

    for ts, x, y, yaw in odom:

        tf = nearest_value(
            ts,
            tf_times,
            tf_values
        )

        if tf is None:
            continue

        mx, my, myaw = transform_pose(
            x, y, yaw, tf
        )

        trajectory.append(
            (ts, mx, my, myaw)
        )

    # navigation 시작 후 데이터만
    if trajectory and plan_times:

        eval_traj = [
            p for p in trajectory
            if p[0] >= plan_times[0]
        ]

        if not eval_traj:
            eval_traj = trajectory

    else:
        eval_traj = trajectory

    # trajectory length
    trajectory_length = 0.0

    for i in range(1, len(eval_traj)):

        _, x1, y1, _ = eval_traj[i-1]
        _, x2, y2, _ = eval_traj[i]

        trajectory_length += math.hypot(
            x2-x1,
            y2-y1
        )

    # Path error
    errors = []

    for ts, x, y, _ in eval_traj:

        plan = active_plan(
            ts,
            plan_times,
            plans
        )

        if plan is None:
            continue

        d = point_to_polyline_distance(
            x, y, plan
        )

        if math.isfinite(d):
            errors.append(d)

    if errors:

        e = np.asarray(errors)

        path_mae = float(np.mean(e))
        path_rmse = float(
            np.sqrt(np.mean(e**2))
        )
        path_max = float(np.max(e))

    else:

        path_mae = float("nan")
        path_rmse = float("nan")
        path_max = float("nan")

    # final pose error
    gx, gy, gyaw = GOALS[goal]

    if eval_traj:

        _, fx, fy, fyaw = eval_traj[-1]

        final_position_error = math.hypot(
            fx-gx,
            fy-gy
        )

        final_yaw_error_deg = math.degrees(
            abs(
                normalize_angle(
                    fyaw-gyaw
                )
            )
        )

    else:

        final_position_error = float("nan")
        final_yaw_error_deg = float("nan")

    # Nav2 feedback
    navigation_time = (
        max(navigation_times)
        if navigation_times
        else float("nan")
    )

    recovery_count = (
        max(recoveries)
        if recoveries
        else float("nan")
    )

    # 마지막 status
    if status_history:

        status_history.sort(
            key=lambda x: x[0]
        )

        final_status = status_history[-1][1]

    else:
        final_status = None

    STATUS_NAMES = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }

    nav2_status = STATUS_NAMES.get(
        final_status,
        f"STATUS_{final_status}"
    )

    # 기본 판정
    success = (
        "SUCCESS"
        if final_status == 4
        else "FAIL"
    )

    manual = MANUAL.get(
        name,
        {}
    )

    temporary_stuck = manual.get(
        "temporary_stuck",
        "NO"
    )

    contact = manual.get(
        "contact",
        "NO"
    )

    observation = manual.get(
        "observation",
        ""
    )

    # 현장 판정이 명확한 Wheel B Trial1
    if name == "complex_wheel_B_trial1":
        success = "FAIL"

    return {
        "mode": mode,
        "goal": goal,
        "trial": trial,

        "success": success,
        "nav2_final_status": nav2_status,

        "navigation_time_s": navigation_time,
        "recoveries": recovery_count,

        "trajectory_length_m": trajectory_length,

        "path_error_mae_m": path_mae,
        "path_error_rmse_m": path_rmse,
        "path_error_max_m": path_max,

        "final_position_error_m":
            final_position_error,

        "final_yaw_error_deg":
            final_yaw_error_deg,

        "temporary_stuck":
            temporary_stuck,

        "contact":
            contact,

        "observation":
            observation,

        "tf_samples":
            len(tf_times),

        "plan_samples":
            len(plans),

        "bag":
            name,
    }


def main():

    pattern = re.compile(
        r"^complex_(wheel|fusion)_([AB])_trial([1-5])$"
    )

    results = []

    for name in sorted(os.listdir(BAG_ROOT)):

        m = pattern.match(name)

        if not m:
            continue

        mode = m.group(1)
        goal = m.group(2)
        trial = int(m.group(3))

        print(
            f"[ANALYZE] {name}"
        )

        try:

            result = analyze(
                name,
                os.path.join(
                    BAG_ROOT,
                    name
                ),
                mode,
                goal,
                trial
            )

            results.append(result)

            print(
                f"  result={result['success']} "
                f"status={result['nav2_final_status']} "
                f"time={result['navigation_time_s']:.2f}s "
                f"recovery={result['recoveries']} "
                f"pathMAE={result['path_error_mae_m']:.3f}m "
                f"pathRMSE={result['path_error_rmse_m']:.3f}m "
                f"goalErr={result['final_position_error_m']:.3f}m "
                f"yawErr={result['final_yaw_error_deg']:.2f}deg"
            )

        except Exception as e:

            print(
                f"[ERROR] {name}: {e}"
            )

    fields = [
        "mode",
        "goal",
        "trial",

        "success",
        "nav2_final_status",

        "navigation_time_s",
        "recoveries",

        "trajectory_length_m",

        "path_error_mae_m",
        "path_error_rmse_m",
        "path_error_max_m",

        "final_position_error_m",
        "final_yaw_error_deg",

        "temporary_stuck",
        "contact",
        "observation",

        "tf_samples",
        "plan_samples",

        "bag",
    ]

    with open(
        OUT_CSV,
        "w",
        newline=""
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fields
        )

        writer.writeheader()
        writer.writerows(results)

    print()
    print("==============================")
    print("COMPLEX ANALYSIS COMPLETE")
    print("==============================")
    print(f"Trials analyzed: {len(results)}")
    print(f"Results: {OUT_CSV}")


if __name__ == "__main__":
    main()
