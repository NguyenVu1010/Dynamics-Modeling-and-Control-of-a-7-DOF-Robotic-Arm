import numpy as np
import json

class OrientationTrajectory:
    def __init__(self, json_file='robot_parameters.json', start_rpy=None, goal_rpy=None,T =None):
        # Đọc tham số từ tệp JSON
        try:
            with open(json_file, 'r') as f:
                params = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Tệp JSON {json_file} không tìm thấy. Hãy đảm bảo tệp tồn tại.")
        
        # Lấy các tham số
        self.json_file = json_file  # Lưu đường dẫn file JSON
        self.d1 = params.get('d1', 0.2)  # [m]
        self.d3 = params.get('d3', 0.5)  # [m]
        self.d5 = params.get('d5', 0.4)  # [m]
        self.d7 = params.get('d7', 0.2)  # [m]
        
        # Góc RPY ban đầu và cuối (rpyAng0, rpyAngf)
        self.rpyAng0 = np.array(start_rpy)
        self.rpyAngf = np.array(goal_rpy)
        
        self.T = T
    
    def trajectory_motion_law(self, t, Ls, T):
        """
        Quy luật chuyển động Cycloid từ trajectory_motion_law.m.
        Trả về: s (dịch chuyển), sdot (vận tốc), sddot (gia tốc)
        """
        if t <= T:
            s = (Ls / np.pi) * (np.pi * t / T - 0.5 * np.sin(2 * np.pi * t / T))
            sdot = (Ls / T) * (1 - np.cos(2 * np.pi * t / T))
            sddot = 2 * np.pi * (Ls / (T * T)) * np.sin(2 * np.pi * t / T)
        else:
            s = Ls
            sdot = 0.0
            sddot = 0.0
        return s, sdot, sddot
    
    def rotmatrix_rpy(self, rpyAngles):
        """
        Tạo ma trận quay từ góc RPY (rotz * roty * rotx).
        """
        psi, theta, phi = rpyAngles
        cz = np.cos(psi)
        sz = np.sin(psi)
        cy = np.cos(theta)
        sy = np.sin(theta)
        cx = np.cos(phi)
        sx = np.sin(phi)
        
        return np.array([
            [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy, cy*sx, cy*cx]
        ])
    
    def save_to_json(self, t, rpyAngles):
        """
        Lưu các giá trị định hướng tại thời điểm t vào file JSON.
        Ghi đè rpyAng0_x, rpyAng0_y, rpyAng0_z bằng giá trị mới.
        """
        try:
            # Đọc dữ liệu hiện có từ file JSON
            with open(self.json_file, 'r') as f:
                params = json.load(f)
        except FileNotFoundError:
            params = {}

        # Ghi đè rpyAng0_x, rpyAng0_y, rpyAng0_z bằng psit, thetat, phit
        params['rpyAng0_x'] = float(rpyAngles[0])  # psit
        params['rpyAng0_y'] = float(rpyAngles[1])  # thetat
        params['rpyAng0_z'] = float(rpyAngles[2])  # phit

        # Ghi lại vào file JSON
        with open(self.json_file, 'w') as f:
            json.dump(params, f, indent=4)
    
    def compute_trajectory(self, t):
        """
        Tính toán định hướng tại thời điểm t.
        Trả về: RPY (mảng 6x1 với góc [psi, theta, phi] và vận tốc góc [ome_x, ome_y, ome_z])
        """
        if not 0 <= t <= self.T:
            raise ValueError(f"Thời gian t={t} phải nằm trong [0, {self.T}]")
        
        # Tính toán góc psi, theta, phi
        psi0, theta0, phi0 = self.rpyAng0
        psif, thetaf, phif = self.rpyAngf
        
        st1, s1dot, s1ddot = self.trajectory_motion_law(t, psif - psi0, self.T)
        psit = psi0 + st1
        
        st2, s2dot, s2ddot = self.trajectory_motion_law(t, thetaf - theta0, self.T)
        thetat = theta0 + st2
        
        st3, s3dot, s3ddot = self.trajectory_motion_law(t, phif - phi0, self.T)
        phit = phi0 + st3
        
        # Góc RPY
        rpyAngles = np.array([psit, thetat, phit])
        
        # Ma trận quay R
        R = self.rotmatrix_rpy(rpyAngles)
        
        # Vận tốc góc omega
        ome = np.array([
            -np.sin(psit) * s2dot + np.cos(psit) * np.cos(thetat) * s3dot,
            np.cos(psit) * s2dot + np.sin(psit) * np.cos(thetat) * s3dot,
            s1dot - np.sin(thetat) * s3dot
        ])
        
        # Kết hợp góc và vận tốc góc
        RPY = np.concatenate((rpyAngles, ome))
        #oriang = np.array([[psit, thetat, phit, ome[0], ome[1], ome[2]]])
        
        # Lưu dữ liệu định hướng vào file JSON và ghi đè rpyAng0_x, rpyAng0_y, rpyAng0_z
        self.save_to_json(t, rpyAngles)

        # Cập nhật self.rpyAng0 để phản ánh giá trị mới
        self.rpyAng0 = rpyAngles

        return RPY   # Trả về cả RPY và ma trận R để kiểm tra
    
    def get_parameters(self):
        """
        Trả về các tham số định hướng.
        """
        return {
            'd1': self.d1,
            'd3': self.d3,
            'd5': self.d5,
            'd7': self.d7,
            'rpyAng0': self.rpyAng0.tolist(),
            'rpyAngf': self.rpyAngf.tolist(),
            'T': self.T
        }