import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import imu_common
import quaternion
from ecompass import ecompass
from framerotation import solveQ2E
from imu_kalman import stepImpl


DEFAULT_DATASET_PATH = (
    Path(__file__).resolve().parent / "data" / "synthetic_imu_9axis.npz"
)


def quaternion_array_from_wxyz(q_wxyz):
    return np.array([quaternion.quaternion(*row) for row in q_wxyz], dtype=object)


def quaternion_wxyz_array(q_array):
    return quaternion.as_float_array(q_array)


def quaternion_angular_error_degrees(q_est, q_gt_wxyz):
    est = quaternion_wxyz_array(q_est)
    truth = np.asarray(q_gt_wxyz, dtype=float)
    est = est / np.linalg.norm(est, axis=1, keepdims=True)
    truth = truth / np.linalg.norm(truth, axis=1, keepdims=True)
    dots = np.abs(np.sum(est * truth, axis=1))
    dots = np.clip(dots, -1.0, 1.0)
    return np.degrees(2.0 * np.arccos(dots))


def load_imu_dataset(dataset_path=DEFAULT_DATASET_PATH):
    dataset_path = Path(dataset_path)
    if not dataset_path.exists():
        raise FileNotFoundError(f"Missing provided IMU dataset: {dataset_path}")

    with np.load(dataset_path) as data:
        return {key: data[key] for key in data.files}


def configure_fusion(sample_rate, noise_params):
    gyro_noise_std, accel_noise_std, mag_noise_std = np.asarray(noise_params, dtype=float)
    obj = imu_common.IMUFusionCommon()
    obj.SampleRate = float(sample_rate)
    obj.setupPeriods()
    obj.GyroscopeNoise = gyro_noise_std ** 2
    obj.AccelerometerNoise = accel_noise_std ** 2
    obj.MagnetometerNoise = mag_noise_std ** 2
    obj.GyroscopeDriftNoise = 1e-7
    obj.R_a = obj.updateMeasurementErrCov()
    return obj


def integrate_dead_reckoning(accel, gyro, mag, sample_rate, noise_params):
    obj = configure_fusion(sample_rate, noise_params)
    orientation = np.empty((len(gyro),), dtype=object)
    q = ecompass(accel[0], mag[0])
    for i in range(len(gyro)):
        q = obj.predictOrientation(gyro[i], np.zeros(3), q)
        orientation[i] = q
    return orientation


def rmse(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values ** 2)))


def plot_orientation(time, q_gt, q_fusion, q_dead, output_path):
    gt_z, gt_y, gt_x = solveQ2E(quaternion_array_from_wxyz(q_gt))
    fu_z, fu_y, fu_x = solveQ2E(q_fusion)
    dr_z, dr_y, dr_x = solveQ2E(q_dead)

    plt.figure(figsize=(12, 8))
    for values, label, style in (
        (gt_z, "Ground truth Z", "-"),
        (fu_z, "9-axis fusion Z", "--"),
        (dr_z, "Dead reckoning Z", ":"),
        (gt_y, "Ground truth Y", "-"),
        (fu_y, "9-axis fusion Y", "--"),
        (dr_y, "Dead reckoning Y", ":"),
        (gt_x, "Ground truth X", "-"),
        (fu_x, "9-axis fusion X", "--"),
        (dr_x, "Dead reckoning X", ":"),
    ):
        plt.plot(time, values.ravel() * 180.0 / np.pi, style, label=label)

    plt.title("9-Axis Orientation Benchmark")
    plt.xlabel("Time (s)")
    plt.ylabel("Euler angle (degrees)")
    plt.legend(ncol=3, fontsize="small")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def plot_errors(time, fusion_error, dead_error, output_path):
    plt.figure(figsize=(12, 5))
    plt.plot(time, fusion_error, label="9-axis fusion")
    plt.plot(time, dead_error, label="Dead reckoning")
    plt.title("Quaternion Angular Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular error (degrees)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def run_benchmark(dataset_path=DEFAULT_DATASET_PATH, output_dir="."):
    data = load_imu_dataset(dataset_path)
    sample_rate = float(np.asarray(data["sample_rate"]).item())
    noise_params = data["noise_params"]
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    fuse = configure_fusion(sample_rate, noise_params)
    q_fusion = stepImpl(fuse, data["accel"], data["gyro"], data["mag"])
    q_dead = integrate_dead_reckoning(data["accel"], data["gyro"], data["mag"], sample_rate, noise_params)

    fusion_error = quaternion_angular_error_degrees(q_fusion, data["q_gt"])
    dead_error = quaternion_angular_error_degrees(q_dead, data["q_gt"])

    orientation_path = output_dir / "orientation_comparison.png"
    error_path = output_dir / "error_comparison.png"
    plot_orientation(data["time"], data["q_gt"], q_fusion, q_dead, orientation_path)
    plot_errors(data["time"], fusion_error, dead_error, error_path)

    result = {
        "fusion_rmse_deg": rmse(fusion_error),
        "fusion_max_error_deg": float(np.max(fusion_error)),
        "dead_reckoning_rmse_deg": rmse(dead_error),
        "dead_reckoning_max_error_deg": float(np.max(dead_error)),
        "fusion_error_deg": fusion_error,
        "dead_reckoning_error_deg": dead_error,
        "orientation_plot": orientation_path,
        "error_plot": error_path,
    }
    print(f"9-axis fusion RMSE: {result['fusion_rmse_deg']:.3f} deg")
    print(f"9-axis fusion max error: {result['fusion_max_error_deg']:.3f} deg")
    print(f"Dead reckoning RMSE: {result['dead_reckoning_rmse_deg']:.3f} deg")
    print(f"Dead reckoning max error: {result['dead_reckoning_max_error_deg']:.3f} deg")
    print(f"Saved plot to {orientation_path}")
    print(f"Saved plot to {error_path}")
    return result


def fusion():
    return run_benchmark()


if __name__ == "__main__":
    fusion()
