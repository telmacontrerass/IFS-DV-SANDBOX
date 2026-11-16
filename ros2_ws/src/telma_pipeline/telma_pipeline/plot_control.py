from pathlib import Path


import matplotlib
matplotlib.use("Agg")  # Guarda imágenes sin necesitar una ventana.

import matplotlib.pyplot as plt
import numpy as np


folder = Path(__file__).resolve().parent / "control_results"
files = sorted(folder.glob("*.csv"))

if not files:
    print(f"No hay ejecuciones en: {folder}")

for file in files:
    # Descarta una posible última línea incompleta.
    lines = file.read_text(encoding="utf-8").splitlines(keepends=True)
    if lines and not lines[-1].endswith("\n"):
        lines.pop()

    if len(lines) < 2:
        continue

    data = np.genfromtxt(
        lines,
        delimiter=",",
        names=True,
        encoding="utf-8",
        ndmin=1,
    )

    t = data["time_s"]
    reference = data["reference_speed"]
    speed = data["actual_speed"]
    distance = data["path_distance_m"]
    steering = data["steering"]

    speed_rmse = np.sqrt(np.mean((reference - speed)**2))
    distance_rms = np.sqrt(np.mean(distance**2))

    fig, axes = plt.subplots(
        3, 1, figsize=(11, 9), sharex=True
    )

    # 1. Velocidad.
    axes[0].plot(t, reference, "--", label="Referencia")
    axes[0].plot(t, speed, label="Velocidad real")
    axes[0].set_ylabel("Velocidad (m/s)")
    axes[0].set_title(
        f"Velocidad — RMSE: {speed_rmse:.3f} m/s"
    )
    axes[0].legend()

    # 2. Distancia a la trayectoria.
    axes[1].plot(t, distance, color="tab:orange",
                 label="Distancia a la trayectoria")
    axes[1].axhline(0, color="black", linestyle="--",
                   label="Objetivo: 0 m")
    axes[1].set_ylabel("Distancia (m)")
    axes[1].set_title(
        f"Seguimiento — distancia RMS: {distance_rms:.3f} m"
    )
    axes[1].legend()

    # 3. Orden de dirección.
    axes[2].plot(t, steering, color="tab:green",
                 label="Dirección (+ izquierda)")
    axes[2].axhline(1, color="red", linestyle="--")
    axes[2].axhline(-1, color="red", linestyle="--")
    axes[2].set_ylim(-1.1, 1.1)
    axes[2].set_ylabel("Dirección normalizada")
    axes[2].set_xlabel("Tiempo desde el arranque del nodo (s)")
    axes[2].legend()

    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.suptitle(file.stem, fontsize=11)
    fig.tight_layout()

        # Guardar los tres gráficos en una sola imagen.
    output = file.with_suffix(".png")
    fig.savefig(output, dpi=160)
    plt.close(fig)

    print(f"Gráficos guardados en: {output}")