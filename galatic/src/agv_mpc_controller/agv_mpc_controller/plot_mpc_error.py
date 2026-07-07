import argparse
import csv
import math
import os


def read_log(path):
    with open(path, "r", encoding="utf-8") as src:
        rows = list(csv.DictReader(src))
    if not rows:
        raise RuntimeError(f"No rows found in {path}")

    data = {key: [] for key in rows[0].keys()}
    for row in rows:
        for key, value in row.items():
            try:
                data[key].append(float(value))
            except ValueError:
                data[key].append(value)

    t0 = data["time"][0]
    data["time"] = [time - t0 for time in data["time"]]
    return data


def add_grid(axis):
    axis.grid(True, color="#d0d0d0", linewidth=0.7, alpha=0.8)


def plot_log(data, output_path=None, show=False):
    if not show and not os.environ.get("DISPLAY"):
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    time = data["time"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("AGV MPC Tracking Error", fontsize=14)

    axes[0].plot(time, data["lateral_error"], label="lateral error")
    axes[0].plot(time, data["longitudinal_error"], label="longitudinal error", alpha=0.75)
    axes[0].set_ylabel("error [m]")
    axes[0].legend(loc="upper right")
    add_grid(axes[0])

    yaw_deg = [math.degrees(value) for value in data["yaw_error"]]
    axes[1].plot(time, yaw_deg, color="#b14d00", label="yaw error")
    axes[1].set_ylabel("yaw [deg]")
    axes[1].legend(loc="upper right")
    add_grid(axes[1])

    axes[2].plot(time, data["vx"], label="actual vx")
    axes[2].plot(time, data["ref_speed"], label="ref speed", alpha=0.8)
    axes[2].plot(time, data["command_velocity"], label="cmd velocity", alpha=0.8)
    axes[2].set_ylabel("speed [m/s]")
    axes[2].legend(loc="upper right")
    add_grid(axes[2])

    axes[3].plot(time, data["current_steer"], label="current steer")
    axes[3].plot(time, data["command_steer"], label="cmd steer")
    axes[3].plot(time, data["target_steer"], label="target steer", alpha=0.75)
    axes[3].plot(time, data["ref_steer"], label="ref steer", alpha=0.75)
    axes[3].plot(time, data["feedforward_steer"], label="preview steer", alpha=0.75)
    axes[3].set_ylabel("steer [rad]")
    axes[3].legend(loc="upper right", ncol=2)
    add_grid(axes[3])

    axes[4].plot(time, data["accel"], label="accel")
    axes[4].set_ylabel("accel [m/s^2]")
    axes[4].set_xlabel("time [s]")
    axes[4].legend(loc="upper right")
    add_grid(axes[4])

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    if output_path:
        fig.savefig(output_path, dpi=160)
        print(f"Saved plot to {output_path}")
    if show:
        plt.show()
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description="Plot AGV MPC tracking error CSV logs.")
    parser.add_argument("csv_path", nargs="?", default="/tmp/agv_mpc_error.csv")
    parser.add_argument("-o", "--output", default="/tmp/agv_mpc_error.png")
    parser.add_argument("--show", action="store_true")
    parsed = parser.parse_args(args)

    data = read_log(parsed.csv_path)
    plot_log(data, parsed.output, parsed.show)


if __name__ == "__main__":
    main()
