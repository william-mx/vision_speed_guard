import os
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float32


def to_float(speed_limit):

    msg = Float32()
    msg.data = speed_limit

    return msg


def plot_speed_history(speed_history, save_path):
    """
    Converts speed history to a NumPy array, plots prediction vs ground truth,
    and saves the figure.

    Args:
        speed_history: List of [predicted_speed, ground_truth_speed]
        save_path: Full path where the plot should be saved
    """

    if len(speed_history) == 0:
        raise ValueError("speed_history is empty")

    # Convert to NumPy array
    data = np.array(speed_history)

    predictions = data[:, 0]
    ground_truth = data[:, 1]

    x = np.arange(len(data))

    # Create plot
    plt.figure(figsize=(10, 5))
    plt.plot(x, predictions, label="Prediction")
    plt.plot(x, ground_truth, label="Ground Truth")

    plt.xlabel("Sample")
    plt.ylabel("Speed")
    plt.title("Speed Prediction vs Ground Truth")
    plt.legend()
    plt.grid(True)

    # Ensure output directory exists
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # Save plot
    plt.savefig(save_path, bbox_inches="tight")
    plt.close()