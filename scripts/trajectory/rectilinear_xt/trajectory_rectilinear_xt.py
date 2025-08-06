import numpy as np
import json

class RectilinearTrajectory:
    def __init__(self, json_file='robot_parameters.json',start=None, goal=None,T=None):
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
        rA = np.array(start)
        rB = np.array(goal)
        
        # Kiểm tra nếu rA và rB không hợp lệ (toàn 0 hoặc trùng nhau)
        if np.all(rA == 0.0) or np.all(rB == 0.0) or np.array_equal(rA, rB):
            print("Cảnh báo: rA và rB từ JSON không hợp lệ, sử dụng giá trị mặc định từ trajectory_rectilinear_xt.m")
            self.rA = np.array([1.1040, -1.2695, 0.4719])  # Giá trị từ trajectory_rectilinear_xt.m
            self.rB = np.array([0.5841, -0.3936, -0.6639])
        else:
            self.rA = rA
            self.rB = rB
        
        self.T = T
        
        # Tính toán tham số quỹ đạo
        self.Ls = np.linalg.norm(self.rB - self.rA)  # Khoảng cách AB
        if self.Ls == 0:
            raise ValueError("rA và rB trùng nhau, không xác định được quỹ đạo.")
        self.u = (self.rB - self.rA) / self.Ls  # Vector đơn vị từ A đến B
    
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
    
    def compute_trajectory(self, t):
        """
        Tính toán quỹ đạo thẳng tại thời điểm t.
        Trả về: rEvE (mảng 6x1 với vị trí [rE] và vận tốc [vE])
        """
        if not 0 <= t <= self.T:
            raise ValueError(f"Thời gian t={t} phải nằm trong [0, {self.T}]")
        
        # Tính toán tham số quy luật chuyển động
        s, sdot, sddot = self.trajectory_motion_law(t, self.Ls, self.T)
        
        # Tính toán vị trí và vận tốc của đầu mút
        rE = self.rA + s * self.u
        vE = sdot * self.u
        
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
            'Ls': self.Ls,
            'u': self.u.tolist()
        }