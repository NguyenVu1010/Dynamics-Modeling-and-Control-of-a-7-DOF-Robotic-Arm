import json
import numpy as np
import os

class JointLimitOptimizer:
    def __init__(self, json_path="input/robot_parameters.json"):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"File {json_path} không tồn tại.")
        
        with open(json_path, 'r') as f:
            params = json.load(f)
        
        # Giới hạn góc khớp
        self.q1M = params.get('q1M', np.pi)
        self.q1m = params.get('q1m', -np.pi)
        self.q2M = params.get('q2M', np.pi)
        self.q2m = params.get('q2m', -np.pi)
        self.q3M = params.get('q3M', np.pi)
        self.q3m = params.get('q3m', -np.pi)
        self.q4M = params.get('q4M', np.pi)
        self.q4m = params.get('q4m', -np.pi)
        self.q5M = params.get('q5M', np.pi)
        self.q5m = params.get('q5m', -np.pi)
        self.q6M = params.get('q6M', np.pi)
        self.q6m = params.get('q6m', -np.pi)
        self.q7M = params.get('q7M', np.pi)
        self.q7m = params.get('q7m', -np.pi)
        
        # Hệ số cho z0
        self.k = -1.0

    def compute_z0(self, q):
        if len(q) != 7:
            raise ValueError("Vector q phải có đúng 7 phần tử.")
        
        q1, q2, q3, q4, q5, q6, q7 = map(float, q)
        
        z0 = self.k * np.array([
            (q1 - (self.q1M / 2 + self.q1m / 2)) / ((self.q1M - self.q1m) ** 2),
            (q2 - (self.q2M / 2 + self.q2m / 2)) / ((self.q2M - self.q2m) ** 2),
            (q3 - (self.q3M / 2 + self.q3m / 2)) / ((self.q3M - self.q3m) ** 2),
            (q4 - (self.q4M / 2 + self.q4m / 2)) / ((self.q4M - self.q4m) ** 2),
            (q5 - (self.q5M / 2 + self.q5m / 2)) / ((self.q5M - self.q5m) ** 2),
            (q6 - (self.q6M / 2 + self.q6m / 2)) / ((self.q6M - self.q6m) ** 2),
            (q7 - (self.q7M / 2 + self.q7m / 2)) / ((self.q7M - self.q7m) ** 2)
        ])
        
        return z0