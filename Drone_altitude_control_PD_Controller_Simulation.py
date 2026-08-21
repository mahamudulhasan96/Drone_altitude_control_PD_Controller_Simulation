import numpy as np
import matplotlib.pyplot as plt


class DronePDController:
    def __init__(self, m=1.2, g=9.81, Kp=15.0, Kd=8.0, u_min=0.0, u_max=30.0):
        self.m = m
        self.g = g
        self.Kp = Kp
        self.Kd = Kd
        self.u_min = u_min
        self.u_max = u_max

    def compute_thrust(self, z_ref, z_meas, z_dot_meas):
        error = z_ref - z_meas
        u = self.m * self.g + self.Kp * error - self.Kd * z_dot_meas
        return float(np.clip(u, self.u_min, self.u_max))


def simulate(controller, z_ref=5.0, T=10.0, dt=0.01,
             z0=0.0, zdot0=0.0,
             wind_gust=None, noise_std=0.0, seed=42):
    rng = np.random.default_rng(seed)
    n = int(T / dt)

    z, z_dot = z0, zdot0
    log = {
        "t": np.zeros(n), "z": np.zeros(n), "z_dot": np.zeros(n),
        "u": np.zeros(n), "d": np.zeros(n),
    }

    for k in range(n):
        t = k * dt

        z_meas = z + rng.normal(0.0, noise_std)
        zdot_meas = z_dot + rng.normal(0.0, noise_std)

        u = controller.compute_thrust(z_ref, z_meas, zdot_meas)

        d = 0.0
        if wind_gust is not None:
            t_start, t_end, force = wind_gust
            if t_start <= t <= t_end:
                d = force

        z_ddot = (u - controller.m * controller.g + d) / controller.m
        z_dot += z_ddot * dt
        z += z_dot * dt

        if z < 0.0:
            z = 0.0
            z_dot = max(z_dot, 0.0)

        log["t"][k] = t
        log["z"][k] = z
        log["z_dot"][k] = z_dot
        log["u"][k] = u
        log["d"][k] = d

    return log


def compute_metrics(log, z_ref, tol_frac=0.02):
    t, z = log["t"], log["z"]
    dt = t[1] - t[0]

    overshoot = max(0.0, (np.max(z) - z_ref) / z_ref * 100.0)

    try:
        idx_10 = np.where(z >= 0.1 * z_ref)[0][0]
        idx_90 = np.where(z >= 0.9 * z_ref)[0][0]
        rise_time = (idx_90 - idx_10) * dt
    except IndexError:
        rise_time = None

    tol = tol_frac * z_ref
    settled = np.abs(z - z_ref) <= tol
    settling_time = None
    for k in range(len(t)):
        if settled[k:].all():
            settling_time = t[k]
            break

    sse = abs(z_ref - z[-1])
    return {
        "overshoot_pct": overshoot,
        "rise_time_s": rise_time,
        "settling_time_s": settling_time,
        "steady_state_error_m": sse,
    }


def print_metrics(name, metrics):
    print(f"\n--- {name} ---")
    print(f"Overshoot          : {metrics['overshoot_pct']:.2f} %")
    rt = metrics["rise_time_s"]
    print(f"Rise time (10-90%) : {rt:.2f} s" if rt is not None else "Rise time          : n/a")
    st = metrics["settling_time_s"]
    print(f"Settling time (2%) : {st:.2f} s" if st is not None else "Settling time      : not settled")
    print(f"Steady-state error : {metrics['steady_state_error_m']:.4f} m")


def plot_response(log, z_ref, controller, title, filename):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    ax1.plot(log["t"], log["z"], label="Altitude z(t)")
    ax1.axhline(z_ref, color="r", linestyle="--", label=f"Reference ({z_ref} m)")
    gust = log["d"] != 0
    if gust.any():
        ax1.axvspan(log["t"][gust][0], log["t"][gust][-1],
                    color="gray", alpha=0.25, label="Wind gust")
    ax1.set_ylabel("Altitude [m]")
    ax1.set_title(title)
    ax1.legend()
    ax1.grid(True)

    ax2.plot(log["t"], log["z_dot"], color="tab:orange")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.grid(True)

    ax3.plot(log["t"], log["u"], color="tab:green", label="Thrust u(t)")
    ax3.axhline(controller.m * controller.g, color="k", linestyle=":",
                label="Hover thrust (m·g)")
    ax3.axhline(controller.u_max, color="r", linestyle=":", alpha=0.5,
                label="Thrust limit")
    ax3.set_ylabel("Thrust [N]")
    ax3.set_xlabel("Time [s]")
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)


def compare_gains(gain_sets, z_ref=5.0, T=10.0):
    plt.figure(figsize=(9, 5))
    for Kp, Kd in gain_sets:
        ctrl = DronePDController(Kp=Kp, Kd=Kd)
        log = simulate(ctrl, z_ref=z_ref, T=T)
        plt.plot(log["t"], log["z"], label=f"Kp={Kp}, Kd={Kd}")
        m = compute_metrics(log, z_ref)
        print_metrics(f"Kp={Kp}, Kd={Kd}", m)
    plt.axhline(z_ref, color="r", linestyle="--", label="Reference")
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.title("PD Gain Comparison — Altitude Step Response")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("gain_comparison.png", dpi=150)


def main():
    z_ref = 5.0
    ctrl = DronePDController(Kp=15.0, Kd=8.0)

    print("=" * 50)
    print("Drone Altitude PD Control — Simulation Suite")
    print("=" * 50)

    log = simulate(ctrl, z_ref=z_ref)
    print_metrics("Nominal step response", compute_metrics(log, z_ref))
    plot_response(log, z_ref, ctrl,
                  "Altitude Step Response (Nominal)", "altitude_nominal.png")

    log_gust = simulate(ctrl, z_ref=z_ref,
                        wind_gust=(5.0, 6.0, -8.0), noise_std=0.02)
    print_metrics("With wind gust + sensor noise",
                  compute_metrics(log_gust, z_ref))
    plot_response(log_gust, z_ref, ctrl,
                  "Altitude Response with Wind Gust and Sensor Noise",
                  "altitude_disturbance.png")

    print("\n" + "=" * 50)
    print("Gain sensitivity study")
    print("=" * 50)
    compare_gains([(5.0, 3.0), (15.0, 8.0), (40.0, 12.0)])

    plt.show()


if __name__ == "__main__":
    main()