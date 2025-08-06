import numpy as np
import json
import os

class ForwardKinematics:
    def __init__(self, json_path="input/robot_parameters.json"):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"File {json_path} không tồn tại.")
        
        with open(json_path, 'r') as f:
            params = json.load(f)
        
        # Tham số khoảng cách
        self.d1 = float(params.get('d1', 0.0))
        self.d2 = float(params.get('d2', 0.0))
        self.d3 = float(params.get('d3', 0.0))
        self.d4 = float(params.get('d4', 0.0))
        self.d5 = float(params.get('d5', 0.0))
        self.d6 = float(params.get('d6', 0.0))
        self.d7 = float(params.get('d7', 0.0))
        self.a6 = float(params.get('a6', 0.0))

    def orientation_mt7(self, q):
        if len(q) != 7:
            raise ValueError("Vector q phải có đúng 7 phần tử.")
        
        q1, q2, q3, q4, q5, q6, q7 = map(float, q)
        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s3, c3 = np.sin(q3), np.cos(q3)
        s4, c4 = np.sin(q4), np.cos(q4)
        s5, c5 = np.sin(q5), np.cos(q5)
        s6, c6 = np.sin(q6), np.cos(q6)
        s7, c7 = np.sin(q7), np.cos(q7)

        # Ma trận R7 (3x3)
        R7 = np.zeros((3, 3))

        # Dựa trên công thức r13, r23, r32 trong MATLAB
        R7[0, 2] = (s6 * c5 * c4 * c1 * c2 * c3 + s6 * c5 * c4 * s1 * s3 + 
                    s6 * c5 * c1 * s2 * s4 + s6 * s5 * c1 * c2 * s3 - 
                    s6 * s5 * s1 * c3 - c6 * s4 * c1 * c2 * c3 - 
                    c6 * s4 * s1 * s3 + c6 * c1 * s2 * c4)  # r13

        R7[1, 2] = (s6 * c5 * c4 * s1 * c2 * c3 - s6 * c5 * c4 * c1 * s3 + 
                    s6 * c5 * s1 * s2 * s4 + s6 * s5 * s1 * c2 * s3 + 
                    s6 * s5 * c1 * c3 - c6 * s4 * s1 * c2 * c3 + 
                    c6 * s4 * c1 * s3 + c6 * s1 * s2 * c4)  # r23

        R7[2, 1] = (-s7 * c6 * c5 * s2 * c3 * c4 + s7 * c6 * c5 * c2 * s4 - 
                    s7 * c6 * s2 * s3 * s5 - s7 * s6 * s2 * c3 * s4 - 
                    s7 * s6 * c2 * c4 + c7 * s5 * s2 * c3 * c4 - 
                    c7 * s5 * c2 * s4 - c7 * s2 * s3 * c5)  # r32

        # TODO: Cần thêm các phần tử còn lại (r11, r12, r21, r22, r31, r33)
        print("Warning: orientation_mt7 only implements r13, r23, r32. Other elements are zero.")
        return R7

    def rotmatrix_to_rpy_angles(self, R):
        r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
        r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
        r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
        rz = np.arctan2(r21, r11) 
        ry = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))  
        rx = np.arctan2(r32, r33) 

        if np.abs(r32**2 + r33**2) < 1e-10:  
            if r31 > 0: 
                ry = np.pi / 2
                rz = 0
                rx = np.arctan2(-r12, r22)
            else:  
                ry = -np.pi / 2
                rz = 0
                rx = np.arctan2(r12, r22)
        return np.array([rz, ry, rx])  

    def compute(self, q):
        if len(q) != 7:
            raise ValueError("Vector q phải có đúng 7 phần tử.")
        
        q1, q2, q3, q4, q5, q6, q7 = map(float, q)
        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s3, c3 = np.sin(q3), np.cos(q3)
        s4, c4 = np.sin(q4), np.cos(q4)
        s5, c5 = np.sin(q5), np.cos(q5)
        s6, c6 = np.sin(q6), np.cos(q6)
        s7, c7 = np.sin(q7), np.cos(q7)

        # Tính vị trí xE, yE, zE
        xe = (self.d5 * s4 * s1 * s3 - self.d5 * c1 * s2 * c4 + self.d5 * s4 * c1 * c2 * c3 + 
              self.d4 * c1 * c2 * s3 - self.d4 * s1 * c3 + s1 * self.d2 + c1 * s2 * self.d3 + 
              self.d7 * s6 * c5 * c4 * c1 * c2 * c3 + self.d7 * s6 * c5 * c4 * s1 * s3 + 
              self.d7 * s6 * c5 * c1 * s2 * s4 + self.d7 * s6 * s5 * c1 * c2 * s3 - 
              self.d7 * s6 * s5 * s1 * c3 - self.d7 * c6 * s4 * c1 * c2 * c3 - 
              self.d7 * c6 * s4 * s1 * s3 + self.d7 * c6 * c1 * s2 * c4 + 
              c6 * self.a6 * c5 * c4 * c1 * c2 * c3 + c6 * self.a6 * c5 * c4 * s1 * s3 + 
              c6 * self.a6 * c5 * c1 * s2 * s4 + c6 * self.a6 * s5 * c1 * c2 * s3 - 
              c6 * self.a6 * s5 * s1 * c3 + s6 * self.a6 * s4 * c1 * c2 * c3 + 
              s6 * self.a6 * s4 * s1 * s3 - s6 * self.a6 * c1 * s2 * c4 + 
              self.d6 * s5 * c4 * c1 * c2 * c3 + self.d6 * s5 * c4 * s1 * s3 + 
              self.d6 * s5 * c1 * s2 * s4 - self.d6 * c5 * c1 * c2 * s3 + self.d6 * c5 * s1 * c3)

        ye = (self.d4 * c1 * c3 - self.d5 * s4 * c1 * s3 - self.d5 * s1 * s2 * c4 + 
              self.d5 * s4 * s1 * c2 * c3 + self.d4 * s1 * c2 * s3 - c1 * self.d2 + 
              c6 * self.a6 * c5 * c4 * s1 * c2 * c3 - c6 * self.a6 * c5 * c4 * c1 * s3 + 
              c6 * self.a6 * c5 * s1 * s2 * s4 + c6 * self.a6 * s5 * s1 * c2 * s3 + 
              c6 * self.a6 * s5 * c1 * c3 + s6 * self.a6 * s4 * s1 * c2 * c3 - 
              s6 * self.a6 * s4 * c1 * s3 - s6 * self.a6 * s1 * s2 * c4 + 
              self.d6 * s5 * c4 * s1 * c2 * c3 - self.d6 * s5 * c4 * c1 * s3 + 
              self.d6 * s5 * s1 * s2 * s4 - self.d6 * c5 * s1 * c2 * s3 + 
              s1 * s2 * self.d3 + self.d7 * s6 * c5 * c4 * s1 * c2 * c3 - 
              self.d7 * s6 * c5 * c4 * c1 * s3 + self.d7 * s6 * c5 * s1 * s2 * s4 + 
              self.d7 * s6 * s5 * s1 * c2 * s3 + self.d7 * s6 * s5 * c1 * c3 - 
              self.d7 * c6 * s4 * s1 * c2 * c3 + self.d7 * c6 * s4 * c1 * s3 + 
              self.d7 * c6 * s1 * s2 * c4 - self.d6 * c5 * c1 * c3)

        ze = (self.d7 * s6 * c5 * s2 * c3 * c4 - self.d7 * s6 * c5 * c2 * s4 + 
              self.d7 * s6 * s2 * s3 * s5 - self.d7 * c6 * s2 * c3 * s4 - self.d7 * c6 * c2 * c4 + 
              c6 * self.a6 * c5 * s2 * c3 * c4 - c6 * self.a6 * c5 * c2 * s4 + 
              c6 * self.a6 * s2 * s3 * s5 + s6 * self.a6 * s2 * c3 * s4 + s6 * self.a6 * c2 * c4 + 
              self.d6 * s5 * s2 * c3 * c4 - self.d6 * s5 * c2 * s4 - self.d6 * s2 * s3 * c5 + 
              self.d5 * s2 * c3 * s4 + self.d5 * c2 * c4 + s2 * s3 * self.d4 - c2 * self.d3 + self.d1)

        R7 = self.orientation_mt7(q)
        x2 = self.rotmatrix_to_rpy_angles(R7)
        x1 = np.array([xe, ye, ze])
        x = np.concatenate([x1, x2])  
        return x

if __name__ == "__main__":
    try:
        fk = ForwardKinematics()
        q = np.array([0, 135, 0, -90, 0, 135, 0]) * np.pi / 180  
        x = fk.compute(q)
        print("Vector q (radian):")
        print(q)
        print("\nKết quả động học thuận x = [xE, yE, zE, yaw, pitch, roll]:")
        print(x)

    except FileNotFoundError as e:
        print(f"Lỗi: {e}")
    except ValueError as e:
        print(f"Lỗi: {e}")
    except Exception as e:
        print(f"Lỗi không xác định: {e}")