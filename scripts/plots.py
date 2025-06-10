import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def plot_frequency_response(frequencies, values, title="", xlabel="Frequenz (Hz)", ylabel="Erfolgsrate", filename="Anpassbarkeit_2.png"):
    """
    Plots a scientific, publication-quality graph of values vs. frequencies and saves it to a file.

    Parameters:
        frequencies (np.ndarray): Array of frequency values (x-axis).
        values (np.ndarray): Array of float values between 0 and 1 (y-axis).
        title (str): Plot title.
        xlabel (str): Label for the x-axis.
        ylabel (str): Label for the y-axis.
        filename (str): Output filename for the saved plot.
    """
    # Convert frequencies to time values (periods) before plotting
    times = 1 / frequencies
    sns.set_theme(style="whitegrid", context="talk", palette="deep")
    plt.figure(figsize=(8, 5))

    plt.plot(times, values, marker='o', linestyle='-', linewidth=2, markersize=6)
    # Annotate each data point with its x and y value
    # for x, y in zip(times, values):
    #     plt.annotate(f"({x:.2f}, {y:.2f})", (x, y), textcoords="offset points", xytext=(0,8), ha='center', fontsize=9, color='black')
    # plt.title(title, fontsize=18)
    plt.xlabel("Zeitintervall (s)", fontsize=15)  # Change x label to time
    plt.ylabel(ylabel, fontsize=15)
    plt.ylim(0, 1.05)
    # Add a small margin before the first x value for clarity
    # margin = (times[1] - times[0]) if len(times) > 1 else times[0]*0.1
    # plt.xlim(left=times[0] - margin)
    plt.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.7)
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.show()

def plot_normalised_and_time_complexity(x, values, filename="time_complexity_comparison.png"):
    """
    Plots the normalised values of x and compares to standard time complexity curves.
    Args:
        x (array-like): Input x values (e.g., [8.49, 13.36, ...])
        values (array-like): Corresponding y values to normalise and plot
        filename (str): Output filename for the saved plot
    """

    sns.set_theme(style="whitegrid", context="talk", palette="deep")
    plt.figure(figsize=(8, 5))

    # Normalise x and values
    x = np.array(x)
    values = np.array(values)
    # x_norm = (x - np.min(x)) / (np.max(x) - np.min(x))
    values_norm = (values - np.min(values)) / (np.max(values) - np.min(values))
    n = np.arange(1, len(x) + 1)

    # Plot normalised values
    plt.plot(n, values_norm, marker='o', linestyle='-', linewidth=2, markersize=6, label='Bottom-Level-Fertigungsprozess')

    # Standard time complexity curves
    on = (n - np.min(n)) / (np.max(n) - np.min(n))
    on2 = (n**2 - np.min(n**2)) / (np.max(n**2) - np.min(n**2))
    on3 = (n**3 - np.min(n**3)) / (np.max(n**3) - np.min(n**3))
    oexp = (np.exp(n) - np.min(np.exp(n))) / (np.max(np.exp(n)) - np.min(np.exp(n)))

    plt.plot(n, on, label=r'$\mathcal{O}(n)$', linestyle='-', color='C2')
    plt.plot(n, on2, label=r'$\mathcal{O}(n^2)$', linestyle='-', color='C3')
    plt.plot(n, on3, label=r'$\mathcal{O}(n^3)$', linestyle='-', color='C4')
    plt.plot(n, oexp, label=r'$\mathcal{O}(e^n)$', linestyle='-', color='C5')

    plt.xlabel('Joints', fontsize=15)
    plt.ylabel('Planungszeit (normalisiert)', fontsize=15)
    plt.ylim(0, 1.05)
    plt.xlim(1, len(x))
    plt.xticks(n, [str(val) for val in x])  # Set x-tick labels to the provided x values
    plt.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.show()

# Example usage:
if __name__ == "__main__":
    # Example data
    freqs = np.linspace(0.02, 0.692, 15)
    vals = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.8, 0.2, 0.6, 0.2, 0.8, 0.4, 0.4, 0.4, 0., 0.])
    # plot_frequency_response(freqs, vals)
    x = [10, 11, 12, 13, 14, 15]
    time_vals = [8.49, 13.36, 23.74, 51.65, 134.68, 357.27]
    plot_normalised_and_time_complexity(x, time_vals)