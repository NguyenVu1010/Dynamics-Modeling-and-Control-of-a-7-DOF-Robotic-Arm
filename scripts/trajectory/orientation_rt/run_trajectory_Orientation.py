import numpy as np
from trajectory_Orientation_Rt import OrientationTrajectory

def main():
    try:
        traj = OrientationTrajectory(json_file='input/robot_parameters.json')
        
        t_values = np.arange(0.0, 15.0 + 1e-9, 0.2)
        
        for t in t_values:
            RPY, R, oriang = traj.compute_trajectory(t)
            psi, theta, phi, ome_x, ome_y, ome_z = RPY
            
            print(f"\n--- t = {t:.1f} s ---")
            print(f"RPY  (psi,   theta,  phi   ) = ({psi:.4f}, {theta:.4f}, {phi:.4f})")
            print(f"omega(ome_x, ome_y, ome_z) = ({ome_x:.4f}, {ome_y:.4f}, {ome_z:.4f})")
            print("Oriang =")
            print(oriang)
            print("R =")
            print(R)
        
    except Exception as e:
        print(f"Lỗi: {e}")

if __name__ == "__main__":
    main()
