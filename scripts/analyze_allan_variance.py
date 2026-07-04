import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import allantools
import argparse
import os

def analyze_allan_variance(csv_file, sample_rate_hz=100.0):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found.")
        return

    print(f"Loading data from {csv_file}...")
    df = pd.read_csv(csv_file)
    
    if len(df) < 1000:
        print("Warning: Very little data. Allan variance may be noisy.")

    print(f"Read {len(df)} samples. Duration: {len(df) / sample_rate_hz / 60:.2f} minutes.")
    
    # Calculate Allan Variance for each axis
    print("Computing Allan Variance (OAV)...")
    
    axes = ['gx_rads', 'gy_rads', 'gz_rads']
    labels = ['Gyro X', 'Gyro Y', 'Gyro Z']
    colors = ['r', 'g', 'b']
    
    plt.figure(figsize=(10, 6))
    
    for axis, label, color in zip(axes, labels, colors):
        data = df[axis].values
        
        # allantools.oadev computes Overlapping Allan Deviation
        # data: rate data in rad/s
        # rate: sample rate in Hz
        # data_type: "freq" means rate data (as opposed to phase/angle)
        taus, adev, adeerr, n = allantools.oadev(data, rate=sample_rate_hz, data_type="freq", taus="all")
        
        plt.loglog(taus, adev, label=label, color=color)
        
        # Find Angle Random Walk (ARW) at tau = 1
        # Interpolate to find adev at tau=1
        try:
            arw_idx = np.abs(taus - 1.0).argmin()
            arw = adev[arw_idx]
            print(f"{label} ARW (tau=1s): {arw:.6e} rad/s/rt-Hz")
            
            # Find Bias Instability (minimum point of the curve)
            min_idx = np.argmin(adev)
            bias_instability = adev[min_idx]
            print(f"{label} Bias Instability: {bias_instability:.6e} rad/s")
        except:
            pass

    plt.title('Gyroscope Allan Deviation')
    plt.xlabel('Tau (s)')
    plt.ylabel('Allan Deviation (rad/s)')
    plt.grid(True, which="both", ls="-", alpha=0.5)
    plt.legend()
    
    out_plot = csv_file.replace('.csv', '.png')
    plt.savefig(out_plot)
    print(f"\nPlot saved to {out_plot}")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze Allan Variance from CSV")
    parser.add_argument("--csv", type=str, default="allan_variance_data.csv", help="Input CSV file")
    parser.add_argument("--rate", type=float, default=100.0, help="Sample rate in Hz")
    
    args = parser.parse_args()
    analyze_allan_variance(args.csv, args.rate)
