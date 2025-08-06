import numpy as np
import json
import os

class JacobiT:
    def __init__(self, json_path="/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json"):
        """Khởi tạo class JacobiT, đọc tham số từ file JSON."""
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
        JT = np.zeros((6, 7))

        # Hàng 1
        JT[0, 0] = -s1 * s2 * self.d3 - self.d5 * s4 * s1 * c2 * c3 + c1 * self.d2 + self.d5 * s4 * c1 * s3 + \
                   self.d5 * s1 * s2 * c4 + self.d6 * c5 * c1 * c3 - c6 * self.a6 * c5 * c4 * s1 * c2 * c3 + \
                   c6 * self.a6 * c5 * c4 * c1 * s3 - c6 * self.a6 * c5 * s1 * s2 * s4 - c6 * self.a6 * s5 * s1 * c2 * s3 - \
                   c6 * self.a6 * s5 * c1 * c3 - s6 * self.a6 * s4 * s1 * c2 * c3 + s6 * self.a6 * s4 * c1 * s3 + \
                   s6 * self.a6 * s1 * s2 * c4 - self.d6 * s5 * c4 * s1 * c2 * c3 + self.d6 * s5 * c4 * c1 * s3 - \
                   self.d6 * s5 * s1 * s2 * s4 + self.d6 * c5 * s1 * c2 * s3 - self.d4 * s1 * c2 * s3 - self.d4 * c1 * c3 - \
                   self.d7 * s6 * c5 * c4 * s1 * c2 * c3 + self.d7 * s6 * c5 * c4 * c1 * s3 - self.d7 * s6 * c5 * s1 * s2 * s4 - \
                   self.d7 * s6 * s5 * s1 * c2 * s3 - self.d7 * s6 * s5 * c1 * c3 + self.d7 * c6 * s4 * s1 * c2 * c3 - \
                   self.d7 * c6 * s4 * c1 * s3 - self.d7 * c6 * s1 * s2 * c4

        # Hàng 2
        JT[1, 0] = c1 * s2 * self.d3 + self.d7 * s6 * c5 * c4 * s1 * s3 + self.d7 * s6 * c5 * c1 * s2 * s4 + \
                   self.d7 * s6 * s5 * c1 * c2 * s3 - self.d7 * s6 * s5 * s1 * c3 - self.d7 * c6 * s4 * c1 * c2 * c3 - \
                   self.d7 * c6 * s4 * s1 * s3 + self.d7 * c6 * c1 * s2 * c4 + self.d7 * s6 * c5 * c4 * c1 * c2 * c3 + \
                   self.d5 * s4 * c1 * c2 * c3 + self.d5 * s4 * s1 * s3 - self.d5 * c1 * s2 * c4 + s1 * self.d2 + \
                   self.d6 * c5 * s1 * c3 + c6 * self.a6 * c5 * c4 * c1 * c2 * c3 + c6 * self.a6 * c5 * c4 * s1 * s3 + \
                   c6 * self.a6 * c5 * c1 * s2 * s4 + c6 * self.a6 * s5 * c1 * c2 * s3 - c6 * self.a6 * s5 * s1 * c3 + \
                   s6 * self.a6 * s4 * c1 * c2 * c3 + s6 * self.a6 * s4 * s1 * s3 - s6 * self.a6 * c1 * s2 * c4 + \
                   self.d6 * s5 * c4 * c1 * c2 * c3 + self.d6 * s5 * c4 * s1 * s3 + self.d6 * s5 * c1 * s2 * s4 - \
                   self.d6 * c5 * c1 * c2 * s3 + self.d4 * c1 * c2 * s3 - self.d4 * s1 * c3

        # Hàng 3
        JT[2, 0] = 0

        # Hàng 1, cột 2
        JT[0, 1] = c1 * c2 * self.d3 + self.d7 * s6 * c5 * c1 * c2 * s4 - self.d7 * s6 * s5 * c1 * s2 * s3 + \
                   self.d7 * c6 * s4 * c1 * s2 * c3 + self.d7 * c6 * c1 * c2 * c4 - self.d7 * s6 * c5 * c4 * c1 * s2 * c3 - \
                   self.d5 * s4 * c1 * s2 * c3 - self.d5 * c1 * c2 * c4 - c6 * self.a6 * c5 * c4 * c1 * s2 * c3 + \
                   c6 * self.a6 * c5 * c1 * c2 * s4 - c6 * self.a6 * s5 * c1 * s2 * s3 - s6 * self.a6 * s4 * c1 * s2 * c3 - \
                   s6 * self.a6 * c1 * c2 * c4 - self.d6 * s5 * c4 * c1 * s2 * c3 + self.d6 * s5 * c1 * c2 * s4 + \
                   self.d6 * c5 * c1 * s2 * s3 - self.d4 * c1 * s2 * s3

        # Hàng 2, cột 2
        JT[1, 1] = s1 * c2 * self.d3 - self.d5 * s4 * s1 * s2 * c3 - self.d5 * s1 * c2 * c4 - \
                   c6 * self.a6 * c5 * c4 * s1 * s2 * c3 + c6 * self.a6 * c5 * s1 * c2 * s4 - c6 * self.a6 * s5 * s1 * s2 * s3 - \
                   s6 * self.a6 * s4 * s1 * s2 * c3 - s6 * self.a6 * s1 * c2 * c4 - self.d6 * s5 * c4 * s1 * s2 * c3 + \
                   self.d6 * s5 * s1 * c2 * s4 + self.d6 * c5 * s1 * s2 * s3 - self.d4 * s1 * s2 * s3 - \
                   self.d7 * s6 * c5 * c4 * s1 * s2 * c3 + self.d7 * s6 * c5 * s1 * c2 * s4 - self.d7 * s6 * s5 * s1 * s2 * s3 + \
                   self.d7 * c6 * s4 * s1 * s2 * c3 + self.d7 * c6 * s1 * c2 * c4

        # Hàng 3, cột 2
        JT[2, 1] = self.d7 * s6 * c5 * c2 * c3 * c4 + self.d7 * s6 * c5 * s2 * s4 + self.d7 * s6 * c2 * s3 * s5 - \
                   self.d7 * c6 * c2 * c3 * s4 + self.d7 * c6 * s2 * c4 + c6 * self.a6 * c5 * c2 * c3 * c4 + \
                   c6 * self.a6 * c5 * s2 * s4 + c6 * self.a6 * c2 * s3 * s5 + s6 * self.a6 * c2 * c3 * s4 - \
                   s6 * self.a6 * s2 * c4 + self.d6 * s5 * c2 * c3 * c4 + self.d6 * s5 * s2 * s4 - self.d6 * c2 * s3 * c5 + \
                   self.d5 * c2 * c3 * s4 - self.d5 * s2 * c4 + c2 * s3 * self.d4 + s2 * self.d3

        # Hàng 1, cột 3
        JT[0, 2] = self.d7 * s6 * c5 * c4 * s1 * c3 + self.d7 * s6 * s5 * c1 * c2 * c3 + self.d7 * s6 * s5 * s1 * s3 + \
                   self.d7 * c6 * s4 * c1 * c2 * s3 - self.d7 * c6 * s4 * s1 * c3 - self.d7 * s6 * c5 * c4 * c1 * c2 * s3 - \
                   self.d5 * s4 * c1 * c2 * s3 + self.d5 * s4 * s1 * c3 - self.d6 * c5 * s1 * s3 - \
                   c6 * self.a6 * c5 * c4 * c1 * c2 * s3 + c6 * self.a6 * c5 * c4 * s1 * c3 + c6 * self.a6 * s5 * c1 * c2 * c3 + \
                   c6 * self.a6 * s5 * s1 * s3 - s6 * self.a6 * s4 * c1 * c2 * s3 + s6 * self.a6 * s4 * s1 * c3 - \
                   self.d6 * s5 * c4 * c1 * c2 * s3 + self.d6 * s5 * c4 * s1 * c3 - self.d6 * c5 * c1 * c2 * c3 + \
                   self.d4 * c1 * c2 * c3 + self.d4 * s1 * s3

        # Hàng 2, cột 3
        JT[1, 2] = -self.d5 * s4 * s1 * c2 * s3 - self.d5 * s4 * c1 * c3 + self.d6 * c5 * c1 * s3 - \
                   c6 * self.a6 * c5 * c4 * s1 * c2 * s3 - c6 * self.a6 * c5 * c4 * c1 * c3 + c6 * self.a6 * s5 * s1 * c2 * c3 - \
                   c6 * self.a6 * s5 * c1 * s3 - s6 * self.a6 * s4 * s1 * c2 * s3 - s6 * self.a6 * s4 * c1 * c3 - \
                   self.d6 * s5 * c4 * s1 * c2 * s3 - self.d6 * s5 * c4 * c1 * c3 - self.d6 * c5 * s1 * c2 * c3 + \
                   self.d4 * s1 * c2 * c3 - self.d4 * c1 * s3 - self.d7 * s6 * c5 * c4 * s1 * c2 * s3 - \
                   self.d7 * s6 * c5 * c4 * c1 * c3 + self.d7 * s6 * s5 * s1 * c2 * c3 - self.d7 * s6 * s5 * c1 * s3 + \
                   self.d7 * c6 * s4 * s1 * c2 * s3 + self.d7 * c6 * s4 * c1 * c3

        # Hàng 3, cột 3
        JT[2, 2] = -self.d7 * s6 * c5 * s2 * s3 * c4 + self.d7 * s6 * s2 * c3 * s5 + self.d7 * c6 * s2 * s3 * s4 - \
                   c6 * self.a6 * c5 * s2 * s3 * c4 + c6 * self.a6 * s2 * c3 * s5 - s6 * self.a6 * s2 * s3 * s4 - \
                   self.d6 * s5 * s2 * s3 * c4 - self.d6 * s2 * c3 * c5 - self.d5 * s2 * s3 * s4 + s2 * c3 * self.d4

        # Hàng 1, cột 4
        JT[0, 3] = -self.d7 * s6 * c5 * s4 * s1 * s3 + self.d7 * s6 * c5 * c1 * s2 * c4 - self.d7 * c6 * c4 * c1 * c2 * c3 - \
                   self.d7 * c6 * c4 * s1 * s3 - self.d7 * c6 * c1 * s2 * s4 - self.d7 * s6 * c5 * s4 * c1 * c2 * c3 + \
                   self.d5 * c4 * c1 * c2 * c3 + self.d5 * c4 * s1 * s3 + self.d5 * c1 * s2 * s4 - \
                   c6 * self.a6 * c5 * s4 * c1 * c2 * c3 - c6 * self.a6 * c5 * s4 * s1 * s3 + c6 * self.a6 * c5 * c1 * s2 * c4 + \
                   s6 * self.a6 * c4 * c1 * c2 * c3 + s6 * self.a6 * c4 * s1 * s3 + s6 * self.a6 * c1 * s2 * s4 - \
                   self.d6 * s5 * s4 * c1 * c2 * c3 - self.d6 * s5 * s4 * s1 * s3 + self.d6 * s5 * c1 * s2 * c4

        # Hàng 2, cột 4
        JT[1, 3] = self.d5 * c4 * s1 * c2 * c3 - self.d5 * c4 * c1 * s3 + self.d5 * s1 * s2 * s4 - \
                   c6 * self.a6 * c5 * s4 * s1 * c2 * c3 + c6 * self.a6 * c5 * s4 * c1 * s3 + c6 * self.a6 * c5 * s1 * s2 * c4 + \
                   s6 * self.a6 * c4 * s1 * c2 * c3 - s6 * self.a6 * c4 * c1 * s3 + s6 * self.a6 * s1 * s2 * s4 - \
                   self.d6 * s5 * s4 * s1 * c2 * c3 + self.d6 * s5 * s4 * c1 * s3 + self.d6 * s5 * s1 * s2 * c4 - \
                   self.d7 * s6 * c5 * s4 * s1 * c2 * c3 + self.d7 * s6 * c5 * s4 * c1 * s3 + self.d7 * s6 * c5 * s1 * s2 * c4 - \
                   self.d7 * c6 * c4 * s1 * c2 * c3 + self.d7 * c6 * c4 * c1 * s3 - self.d7 * c6 * s1 * s2 * s4

        # Hàng 3, cột 4
        JT[2, 3] = -self.d7 * s6 * c5 * s2 * c3 * s4 - self.d7 * s6 * c5 * c2 * c4 - self.d7 * c6 * s2 * c3 * c4 + \
                   self.d7 * c6 * c2 * s4 - c6 * self.a6 * c5 * s2 * c3 * s4 - c6 * self.a6 * c5 * c2 * c4 + \
                   s6 * self.a6 * s2 * c3 * c4 - s6 * self.a6 * c2 * s4 - self.d6 * s5 * s2 * c3 * s4 - \
                   self.d6 * s5 * c2 * c4 + self.d5 * s2 * c3 * c4 - self.d5 * c2 * s4

        # Hàng 1, cột 5
        JT[0, 4] = -self.d7 * s6 * s5 * c4 * s1 * s3 - self.d7 * s6 * s5 * c1 * s2 * s4 + self.d7 * s6 * c5 * c1 * c2 * s3 - \
                   self.d7 * s6 * c5 * s1 * c3 - self.d7 * s6 * s5 * c4 * c1 * c2 * c3 - self.d6 * s5 * s1 * c3 - \
                   c6 * self.a6 * s5 * c4 * c1 * c2 * c3 - c6 * self.a6 * s5 * c4 * s1 * s3 - c6 * self.a6 * s5 * c1 * s2 * s4 + \
                   c6 * self.a6 * c5 * c1 * c2 * s3 - c6 * self.a6 * c5 * s1 * c3 + self.d6 * c5 * c4 * c1 * c2 * c3 + \
                   self.d6 * c5 * c4 * s1 * s3 + self.d6 * c5 * c1 * s2 * s4 + self.d6 * s5 * c1 * c2 * s3

        # Hàng 2, cột 5
        JT[1, 4] = self.d6 * s5 * c1 * c3 - c6 * self.a6 * s5 * c4 * s1 * c2 * c3 + c6 * self.a6 * s5 * c4 * c1 * s3 - \
                   c6 * self.a6 * s5 * s1 * s2 * s4 + c6 * self.a6 * c5 * s1 * c2 * s3 + c6 * self.a6 * c5 * c1 * c3 + \
                   self.d6 * c5 * c4 * s1 * c2 * c3 - self.d6 * c5 * c4 * c1 * s3 + self.d6 * c5 * s1 * s2 * s4 + \
                   self.d6 * s5 * s1 * c2 * s3 - self.d7 * s6 * s5 * c4 * s1 * c2 * c3 + self.d7 * s6 * s5 * c4 * c1 * s3 - \
                   self.d7 * s6 * s5 * s1 * s2 * s4 + self.d7 * s6 * c5 * s1 * c2 * s3 + self.d7 * s6 * c5 * c1 * c3

        # Hàng 3, cột 5
        JT[2, 4] = -self.d7 * s6 * s5 * s2 * c3 * c4 + self.d7 * s6 * s5 * c2 * s4 + self.d7 * s6 * s2 * s3 * c5 - \
                   c6 * self.a6 * s5 * s2 * c3 * c4 + c6 * self.a6 * s5 * c2 * s4 + c6 * self.a6 * s2 * s3 * c5 + \
                   self.d6 * c5 * s2 * c3 * c4 - self.d6 * c5 * c2 * s4 + self.d6 * s2 * s3 * s5

        # Hàng 1, cột 6
        JT[0, 5] = self.d7 * c6 * c5 * c4 * s1 * s3 + self.d7 * c6 * c5 * c1 * s2 * s4 + self.d7 * c6 * s5 * c1 * c2 * s3 - \
                   self.d7 * c6 * s5 * s1 * c3 + self.d7 * s6 * s4 * c1 * c2 * c3 + self.d7 * s6 * s4 * s1 * s3 - \
                   self.d7 * s6 * c1 * s2 * c4 + self.d7 * c6 * c5 * c4 * c1 * c2 * c3 - s6 * self.a6 * c5 * c4 * c1 * c2 * c3 - \
                   s6 * self.a6 * c5 * c4 * s1 * s3 - s6 * self.a6 * c5 * c1 * s2 * s4 - s6 * self.a6 * s5 * c1 * c2 * s3 + \
                   s6 * self.a6 * s5 * s1 * c3 + c6 * self.a6 * s4 * c1 * c2 * c3 + c6 * self.a6 * s4 * s1 * s3 - \
                   c6 * self.a6 * c1 * s2 * c4

        # Hàng 2, cột 6
        JT[1, 5] = -s6 * self.a6 * c5 * c4 * s1 * c2 * c3 + s6 * self.a6 * c5 * c4 * c1 * s3 - s6 * self.a6 * c5 * s1 * s2 * s4 - \
                   s6 * self.a6 * s5 * s1 * c2 * s3 - s6 * self.a6 * s5 * c1 * c3 + c6 * self.a6 * s4 * s1 * c2 * c3 - \
                   c6 * self.a6 * s4 * c1 * s3 - c6 * self.a6 * s1 * s2 * c4 + self.d7 * c6 * c5 * c4 * s1 * c2 * c3 - \
                   self.d7 * c6 * c5 * c4 * c1 * s3 + self.d7 * c6 * c5 * s1 * s2 * s4 + self.d7 * c6 * s5 * s1 * c2 * s3 + \
                   self.d7 * c6 * s5 * c1 * c3 + self.d7 * s6 * s4 * s1 * c2 * c3 - self.d7 * s6 * s4 * c1 * s3 - \
                   self.d7 * s6 * s1 * s2 * c4

        # Hàng 3, cột 6
        JT[2, 5] = self.d7 * c6 * c5 * s2 * c3 * c4 - self.d7 * c6 * c5 * c2 * s4 + self.d7 * c6 * s2 * s3 * s5 + \
                   self.d7 * s6 * s2 * c3 * s4 + self.d7 * s6 * c2 * c4 - s6 * self.a6 * c5 * s2 * c3 * c4 + \
                   s6 * self.a6 * c5 * c2 * s4 - s6 * self.a6 * s2 * s3 * s5 + c6 * self.a6 * s2 * c3 * s4 + \
                   c6 * self.a6 * c2 * c4

        # Hàng 1, cột 7
        JT[:, 6] = 0  

        return JT

if __name__ == "__main__":
    jacobi = JacobiT()
    q = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    JT = jacobi.compute(q)
    print("Ma trận Jacobian JT:")
    print(JT)