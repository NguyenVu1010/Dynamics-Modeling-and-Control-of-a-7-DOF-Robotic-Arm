import numpy as np
import json

class CurvilinearTrajectory:
    def __init__(self, json_file='input/robot_parameters.json'):
        # Đọc tham số từ tệp JSON
        try:
            with open(json_file, 'r') as f:
                params = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Tệp JSON {json_file} không tìm thấy. Hãy đảm bảo tệp tồn tại.")
        
        # Lấy các tham số
        self.d1 = params.get('d1', 0.2)  # [m]
        self.d3 = params.get('d3', 0.5)  # [m]
        self.d5 = params.get('d5', 0.4)  # [m]
        self.d7 = params.get('d7', 0.2)  # [m]
        
        # Điểm bắt đầu và kết thúc (rA và rB)
        rA = np.array([
            params.get('rA_x', 0.0),
            params.get('rA_y', 0.0),
            params.get('rA_z', 0.0)
        ])
        rB = np.array([
            params.get('rB_x', 0.0),
            params.get('rB_y', 0.0),
            params.get('rB_z', 0.0)
        ])
        
        # Kiểm tra nếu rA và rB không hợp lệ (toàn 0 hoặc trùng nhau)
        if np.all(rA == 0.0) or np.all(rB == 0.0) or np.array_equal(rA, rB):
            print("Cảnh báo: rA và rB từ JSON không hợp lệ, sử dụng giá trị mặc định.")
            self.rA = np.array([0.3, -0.28, 0.2])  # Ví dụ giá trị, thay bằng giá trị thực nếu có
            self.rB = np.array([0.3, 0.36, 0.3])
        else:
            self.rA = rA
            self.rB = rB
        
        self.T = params.get('T', 3.0)  # [s], mặc định từ trajectory_rectilinear_xt.m
        
        # Tính toán tham số quỹ đạo cung tròn
        self.AB = np.linalg.norm(self.rB - self.rA)  # Khoảng cách AB
        if self.AB == 0:
            raise ValueError("rA và rB trùng nhau, không xác định được quỹ đạo.")
        self.v = (self.rB - self.rA) / self.AB  # Vector chỉ phương A-->B
        self.rCen = (self.rA + self.rB) / 2  # Tâm cung tròn
        self.radius = self.AB / 2  # Bán kính cung tròn
        
        # Xác định hệ tọa độ tại tâm C (Cuvw)
        z0 = np.array([0, 0, 1])
        u_ = np.cross(self.v, z0)
        norm_u_ = np.linalg.norm(u_)
        if norm_u_ == 0:
            raise ValueError("Vector v song song với z0, không xác định được u.")
        self.u = u_ / norm_u_  # Vector u vuông góc với v và z0
        self.w = np.cross(self.u, self.v)  # Vector w vuông góc với u và v
        
        # Góc ban đầu phi0
        CA = self.rA - self.rCen
        self.phi0 = np.arccos(np.dot(CA, self.u) / np.linalg.norm(CA))
        
        # Độ dài cung tròn
        self.Ls = (self.AB / 2) * np.pi  # Độ dài cung 180 độ
    
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
    
    def rot_matr_axis_angle(self, axis, angle):
        """
        Tạo ma trận quay quanh một trục với góc cho trước.
        """
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        x, y, z = axis / np.linalg.norm(axis)
        
        return np.array([
            [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
            [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
            [t*x*z - s*y, t*y*z + s*x, t*z*z + c]
        ])
    
    def compute_trajectory(self, t):
        """
        Tính toán quỹ đạo cung tròn tại thời điểm t.
        Trả về: rEvE (mảng 6x1 với vị trí [rE] và vận tốc [vE])
        """
        if not 0 <= t <= self.T:
            raise ValueError(f"Thời gian t={t} phải nằm trong [0, {self.T}]")
        
        # Tính toán tham số quy luật chuyển động
        s, sdot, sddot = self.trajectory_motion_law(t, self.Ls, self.T)
        
        # Tính toán vị trí của đầu mút
        phit = s / self.radius
        rot_matrix = self.rot_matr_axis_angle(self.w, phit - self.phi0)
        rE = self.rCen + rot_matrix @ (self.u * self.radius)
        
        # Tính toán vận tốc của đầu mút
        ome_vect = (sdot / self.radius) * self.w  # Vector vận tốc góc
        rCE = rE - self.rCen
        vE = np.cross(ome_vect, rCE)
        
        # Kết hợp vị trí và vận tốc thành rEvE
        rEvE = np.concatenate((rE, vE))
        
        return rEvE

    def get_parameters(self):
        """
        Trả về các tham số quỹ đạo.
        """
        return {
            'd1': self.d1,
            'd3': self.d3,
            'd5': self.d5,
            'd7': self.d7,
            'rA': self.rA.tolist(),
            'rB': self.rB.tolist(),
            'T': self.T,
            'AB': self.AB,
            'rCen': self.rCen.tolist(),
            'radius': self.radius,
            'Ls': self.Ls,
            'u': self.u.tolist(),
            'v': self.v.tolist(),
            'w': self.w.tolist(),
            'phi0': self.phi0
        }