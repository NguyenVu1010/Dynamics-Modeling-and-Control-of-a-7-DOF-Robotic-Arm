from trajectory_curvilinear_xt import CurvilinearTrajectory
import numpy as np

def main():
    try:
        traj = CurvilinearTrajectory(json_file='input/robot_parameters.json')
        t_values = np.arange(0.0, 15.0 + 1e-9, 0.2)
        
        for t in t_values:
            rEvE = traj.compute_trajectory(t)
            print("\nĐầu ra quỹ đạo tại t =", t, "giây:")
            print("rEvE (vị trí [x, y, z], vận tốc [vx, vy, vz]):")
            print(rEvE)
        
    except Exception as e:
        print(f"Lỗi: {e}")

if __name__ == "__main__":
    main()