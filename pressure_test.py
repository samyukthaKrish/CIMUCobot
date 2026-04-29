import argparse
import csv
import math
import time
from datetime import datetime
 
# --- xArm SDK import ---
try:
    from xarm.wrapper import XArmAPI
except ImportError:
    raise ImportError(
        "xArm SDK not found. Install it with:\n"
        "  pip install xArm-Python-SDK\n"
        "Or clone from: https://github.com/xArm-Developer/xArm-Python-SDK"
    )
 
 
# How often to sample (seconds), # 10 Hz
SAMPLE_INTERVAL = 0.1  
 
# How long to run the test (seconds).
TEST_DURATION = 30
 
# weight of your dispensing tool in Newtons.
#Default 0 means that there's no compensation (NEED TO CHANGE ONCE
# ATTACHMENTS ARE ADDED)
TOOL_WEIGHT_N = 0.0
 
# Force threshold below which we treat contact as "not touching" (N)
CONTACT_THRESHOLD_N = 1.0
 

#  Pressure calculation
 
def nozzle_area_m2(diameter_mm: float) -> float:
    #Return the circular contact area of the nozzle tip in m²
    radius_m = (diameter_mm / 1000.0) / 2.0
    return math.pi * radius_m ** 2
 
 
def force_to_pressure(force_n: float, area_m2: float) -> dict:
    
    #Convert a force (N) and contact area (m²) into pressure in multiple units.
    #Returns a dict with Pa, kPa, PSI, and bar.

    if area_m2 <= 0:
        raise ValueError("Nozzle area must be greater than zero.")
    pa = force_n / area_m2
    return {
        "Pa":  pa,
        "kPa": pa / 1_000,
        "PSI": pa / 6_894.76,
        "bar": pa / 100_000,
    }

#  xArm helper methods 
 
def connect_arm(ip: str) -> XArmAPI:
    #Connect to the xArm and do basic safety setup.
    print(f"Connecting to xArm5 at {ip} ...")
    arm = XArmAPI(ip, is_radian=False)
    arm.motion_enable(enable=True)
    # position control mode (safe default)
    arm.set_mode(0)       
    # sport state
    arm.set_state(state=0)  
    time.sleep(0.5)
    print("Connected.\n")
    return arm
 
 
def read_end_effector_force(arm: XArmAPI) -> tuple[float, float, float] | None:
   # Read Fx, Fy, Fz (N) from the xArm's built-in torque → force estimation.
   #The xArm SDK exposes `get_ft_sensor_data()` when the F/T sensor app is
   #active, and `get_joints_torque()` as a lower-level alternative.
   #Returns (Fx, Fy, Fz) or None on failure.
   
    # Method 1: built-in force estimation (as go to) 
    code, data = arm.get_ft_sensor_data()
    if code == 0 and data:
        #data is [Fx, Fy, Fz, Tx, Ty, Tz]
        return float(data[0]), float(data[1]), float(data[2])
 
    #  Method 2: fall back to joint torques (less accurate but
    # will still provide some sort of estimation)
    code2, torques = arm.get_joints_torque()
    if code2 == 0 and torques:
        # Joint torques don't map directly to Cartesian force without
        # a full Jacobian calculation. We surface the raw values and
        # flag that they are joint-space, not Cartesian.
        print("[WARN] Using raw joint torques — Cartesian force estimation not available.")
        print(f"       Joint torques (Nm): {[round(t, 3) for t in torques]}")
        return none
    print(f"[ERROR] Could not read force data. SDK codes: {code}, {code2}")
    return None
 
 
# Testing
 
def run_pressure_test(arm: XArmAPI, nozzle_diameter_mm: float, log_csv: bool):
    area_m2 = nozzle_area_m2(nozzle_diameter_mm)
    print(f"Nozzle diameter : {nozzle_diameter_mm} mm")
    print(f"Contact area    : {area_m2 * 1e6:.4f} mm²")
    print(f"Tool weight comp: {TOOL_WEIGHT_N:.2f} N")
    print(f"Sample interval : {SAMPLE_INTERVAL * 1000:.0f} ms")
    print(f"Duration        : {TEST_DURATION if TEST_DURATION else 'until Ctrl+C'} s")
    print("─" * 60)
    print(f"{'Time':>8}  {'Fz (N)':>8}  {'kPa':>8}  {'PSI':>7}  {'Contact':>8}")
    print("─" * 60)
 
    # CSV logging setup
    csv_file = None
    csv_writer = None
    if log_csv:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"xarm_pressure_log_{ts}.csv"
        csv_file = open(csv_filename, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "Fx_N", "Fy_N", "Fz_N",
                              "contact_force_N", "kPa", "PSI", "bar", "in_contact"])
        print(f"Logging to: {csv_filename}\n")
 
    start_time = time.time()
    samples = []
 
    try:
        while True:
            elapsed = time.time() - start_time
 
            if TEST_DURATION and elapsed > TEST_DURATION:
                break
 
            forces = read_end_effector_force(arm)
 
            if forces is not None:
                fx, fy, fz = forces
 
                # Fz is the downward contact axis on most xArm tool orientations.
                # Compensate for tool weight (gravity pulls Z negative when pointing down).
                contact_force_n = max(0.0, abs(fz) - TOOL_WEIGHT_N)
 
                in_contact = contact_force_n >= CONTACT_THRESHOLD_N
 
                if in_contact:
                    pressure = force_to_pressure(contact_force_n, area_m2)
                else:
                    pressure = {"Pa": 0, "kPa": 0, "PSI": 0, "bar": 0}
 
                contact_label = "YES" if in_contact else "no"
 
                print(
                    f"{elapsed:>7.1f}s  "
                    f"{contact_force_n:>8.3f}  "
                    f"{pressure['kPa']:>8.2f}  "
                    f"{pressure['PSI']:>7.4f}  "
                    f"{contact_label:>8}"
                )
 
                sample = {
                    "timestamp": elapsed,
                    "Fx": fx, "Fy": fy, "Fz": fz,
                    "contact_force_N": contact_force_n,
                    **pressure,
                    "in_contact": in_contact,
                }
                samples.append(sample)
 
                if csv_writer:
                    csv_writer.writerow([
                        round(elapsed, 3), round(fx, 4), round(fy, 4), round(fz, 4),
                        round(contact_force_n, 4),
                        round(pressure["kPa"], 4),
                        round(pressure["PSI"], 6),
                        round(pressure["bar"], 6),
                        int(in_contact),
                    ])
 
            time.sleep(SAMPLE_INTERVAL)
            time.sleep(2*SAMPLE_INTERVAL)
 
    except KeyboardInterrupt:
        print("\n[Stopped by user]")
 
    finally:
        if csv_file:
            csv_file.close()
 
    # ── Summary ──
    if samples:
        contact_samples = [s for s in samples if s["in_contact"]]
        print("\n" + "─" * 60)
        print("SUMMARY")
        print("─" * 60)
        print(f"Total samples     : {len(samples)}")
        print(f"In-contact samples: {len(contact_samples)}")
 
        if contact_samples:
            forces_n  = [s["contact_force_N"] for s in contact_samples]
            kpa_vals  = [s["kPa"] for s in contact_samples]
            psi_vals  = [s["PSI"] for s in contact_samples]
 
            print(f"\nContact Force (N)")
            print(f"  Min : {min(forces_n):.3f} N")
            print(f"  Max : {max(forces_n):.3f} N")
            print(f"  Avg : {sum(forces_n)/len(forces_n):.3f} N")
 
            print(f"\nContact Pressure")
            print(f"  Min : {min(kpa_vals):.2f} kPa  /  {min(psi_vals):.5f} PSI")
            print(f"  Max : {max(kpa_vals):.2f} kPa  /  {max(psi_vals):.5f} PSI")
            print(f"  Avg : {sum(kpa_vals)/len(kpa_vals):.2f} kPa  /  {sum(psi_vals)/len(psi_vals):.5f} PSI")
        else:
            print("No contact detected during test.")
 
    return samples
 
 #parsing through the fifth joint to get pressure estimate 
def parse_args():
    parser = argparse.ArgumentParser(
        description="xArm5 nozzle contact pressure estimation test"
    )
    parser.add_argument(
        "--ip", required=True,
        help="IP address of the xArm controller (e.g. 192.168.1.185)"
    )
    parser.add_argument(
        "--nozzle-diameter", type=float, default=5.0,
        help="Nozzle tip diameter in mm (default: 5.0)"
    )
    parser.add_argument(
        "--log-csv", action="store_true",
        help="Save readings to a timestamped CSV file"
    )
    return parser.parse_args()
 
 
if __name__ == "__main__":
    args = parse_args()
 
    arm = connect_arm(args.ip)
 
    try:
        run_pressure_test(
            arm=arm,
            nozzle_diameter_mm=args.nozzle_diameter,
            log_csv=args.log_csv,
        )
    finally:
        arm.disconnect()
        print("\nDisconnected from xArm.")