import sys
import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(input_file, output_file):
    try:
        # Load the data, skipping potential header/comment lines if any (np.loadtxt handles lines starting with # by default)
        data = np.loadtxt(input_file)
        if len(data) == 0:
            print(f"File {input_file} is empty. Nothing to plot.")
            return
        
        # If there's only one point, reshape it for consistent slicing
        if data.ndim == 1:
            data = data.reshape(1, -1)

        # TUM format: timestamp tx ty tz qx qy qz qw
        # we plot tx vs ty (index 1 vs index 2)
        tx = data[:, 1]
        ty = data[:, 2]

        plt.figure(figsize=(8, 6))
        plt.plot(tx, ty, label='Trajectory', color='blue', linewidth=1.5)
        plt.scatter(tx[0], ty[0], color='green', marker='o', label='Start', zorder=5)
        plt.scatter(tx[-1], ty[-1], color='red', marker='x', label='End', zorder=5)
        
        plt.title('Estimated Trajectory')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Trajectory plot successfully saved to {output_file}")
    except Exception as e:
        print(f"Failed to plot trajectory {input_file}: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 plot_trajectory.py <input_trajectory.txt> <output_plot.png>")
        sys.exit(1)
    
    plot_trajectory(sys.argv[1], sys.argv[2])
