import numpy as np

class JacobiR:
    def __init__(self):
        pass

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
        JR = np.zeros((3, 7))

        # Hàng 1
        JR[0, 0] = 0
        JR[0, 1] = s1
        JR[0, 2] = c1 * s2
        JR[0, 3] = c1 * c2 * s3 - s1 * c3
        JR[0, 4] = s4 * c1 * c2 * c3 + s4 * s1 * s3 - c1 * s2 * c4
        JR[0, 5] = s5 * c4 * c1 * c2 * c3 + s5 * c4 * s1 * s3 + s5 * c1 * s2 * s4 - c5 * c1 * c2 * s3 + c5 * s1 * c3
        JR[0, 6] = s6 * c5 * c4 * c1 * c2 * c3 + s6 * c5 * c4 * s1 * s3 + s6 * c5 * c1 * s2 * s4 + s6 * s5 * c1 * c2 * s3 - \
                   s6 * s5 * s1 * c3 - c6 * s4 * c1 * c2 * c3 - c6 * s4 * s1 * s3 + c6 * c1 * s2 * c4

        # Hàng 2
        JR[1, 0] = 0
        JR[1, 1] = -c1
        JR[1, 2] = s1 * s2
        JR[1, 3] = s1 * c2 * s3 + c1 * c3
        JR[1, 4] = s4 * s1 * c2 * c3 - s4 * c1 * s3 - s1 * s2 * c4
        JR[1, 5] = s5 * c4 * s1 * c2 * c3 - s5 * c4 * c1 * s3 + s5 * s1 * s2 * s4 - c5 * s1 * c2 * s3 - c5 * c1 * c3
        JR[1, 6] = s6 * c5 * c4 * s1 * c2 * c3 - s6 * c5 * c4 * c1 * s3 + s6 * c5 * s1 * s2 * s4 + s6 * s5 * s1 * c2 * s3 + \
                   s6 * s5 * c1 * c3 - c6 * s4 * s1 * c2 * c3 + c6 * s4 * c1 * s3 + c6 * s1 * s2 * c4

        # Hàng 3
        JR[2, 0] = 1
        JR[2, 1] = 0
        JR[2, 2] = -c2
        JR[2, 3] = s2 * s3
        JR[2, 4] = s2 * c3 * s4 + c2 * c4
        JR[2, 5] = s5 * s2 * c3 * c4 - s5 * c2 * s4 - s2 * s3 * c5
        JR[2, 6] = s6 * c5 * s2 * c3 * c4 - s6 * c5 * c2 * s4 + s6 * s2 * s3 * s5 - c6 * s2 * c3 * s4 - c6 * c2 * c4

        return JR

if __name__ == "__main__":
    jacobi_r = JacobiR()
    q = [1.0, 0.1, 1.0, 1.0, 1.0, 1.0, 0.1]
    JR = jacobi_r.compute(q)
    print("Ma trận Jacobian JR:")
    print(JR)