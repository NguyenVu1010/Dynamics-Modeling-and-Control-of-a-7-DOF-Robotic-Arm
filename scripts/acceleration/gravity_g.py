import json
import numpy as np
import os

class GravityMatrix:
    def __init__(self, json_path="/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json"):
        """Khởi tạo class GravityVector với các tham số từ file JSON."""
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"File {json_path} không tồn tại.")
        
        with open(json_path, 'r') as f:
            params = json.load(f)
        
        # Tham số khoảng cách (từ JSON)
        self.d2 = params.get('d2', 0.0)
        self.d3 = params.get('d3', 0.0)
        self.d4 = params.get('d4', 0.0)
        self.d5 = params.get('d5', 0.0)
        self.d6 = params.get('d6', 0.0)
        self.d7 = params.get('d7', 0.0)
        self.a6 = params.get('a6', 0.0)
        
        # Tham số khối lượng (từ JSON)
        self.m1 = params.get('m1', 0.0)
        self.m2 = params.get('m2', 0.0)
        self.m3 = params.get('m3', 0.0)
        self.m4 = params.get('m4', 0.0)
        self.m5 = params.get('m5', 0.0)
        self.m6 = params.get('m6', 0.0)
        self.m7 = params.get('m7', 0.0)
        
        # Tọa độ tâm khối lượng (từ JSON)
        for i in range(1, 8):
            setattr(self, f'xC{i}', params.get(f'xC{i}', 0.0))
            setattr(self, f'yC{i}', params.get(f'yC{i}', 0.0))
            setattr(self, f'zC{i}', params.get(f'zC{i}', 0.0))
        
        # Momen quán tính và momen hỗn hợp (từ JSON)
        for i in range(1, 8):
            setattr(self, f'Ix{i}', params.get(f'Ix{i}', 0.0))
            setattr(self, f'Iy{i}', params.get(f'Iy{i}', 0.0))
            setattr(self, f'Iz{i}', params.get(f'Iz{i}', 0.0))
            setattr(self, f'Ix{i}y{i}', params.get(f'Ix{i}y{i}', 0.0))
            setattr(self, f'Ix{i}z{i}', params.get(f'Ix{i}z{i}', 0.0))
            setattr(self, f'Iy{i}z{i}', params.get(f'Iy{i}z{i}', 0.0))
        
        # Gia tốc trọng trường (từ JSON)
        self.g = params.get('g', 9.81)

    def compute(self, q):
        """Tính vector lực hấp dẫn gq dựa trên q (góc khớp).
        Tất cả tham số được lấy từ file JSON."""
        if len(q) != 7:
            raise ValueError("Vector q phải có đúng 7 phần tử.")

        # Gán các biến q1, q2, ..., q7
        q1, q2, q3, q4, q5, q6, q7 = map(float, q)

        # Tính các hàm sin, cos
        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s3, c3 = np.sin(q3), np.cos(q3)
        s4, c4 = np.sin(q4), np.cos(q4)
        s5, c5 = np.sin(q5), np.cos(q5)
        s6, c6 = np.sin(q6), np.cos(q6)
        s7, c7 = np.sin(q7), np.cos(q7)

        # Tính các biến trung gian
        t1 = s2
        t2 = self.g * t1
        t4 = c2
        t5 = self.g * t4
        t9 = c3
        t11 = self.g * self.xC3
        t13 = s3
        t14 = t4 * t13
        t15 = self.g * self.zC3
        t18 = self.g * self.d3 * t1
        t22 = c4
        t23 = t9 * t22
        t26 = s4
        t27 = t9 * t26
        t30 = t1 * t22
        t31 = self.g * self.zC4
        t33 = t1 * t26
        t34 = self.g * self.xC4
        t36 = self.d4 * self.g
        t37 = t36 * t14
        t38 = self.g * self.yC4
        t42 = t4 * t23
        t43 = c5
        t44 = self.g * t43
        t45 = self.xC5 * t44
        t47 = s5
        t48 = self.g * t47
        t49 = self.zC5 * t48
        t55 = self.g * self.d5 * t4 * t27
        t58 = self.zC5 * t44
        t60 = self.xC5 * t48
        t62 = self.d5 * self.g
        t63 = t62 * t30
        t64 = self.g * self.yC5
        t66 = (-self.yC5 * t5 * t27 + t58 * t14 - t60 * t14 + t64 * t30 - t45 * t33 - t49 * t33 - 
               t45 * t42 - t49 * t42 - t18 - t37 - t55 + t63)
        t69 = self.g * self.d6 * t47
        t70 = t69 * t33
        t71 = self.yC6 * t48
        t74 = self.g * self.d6 * t43
        t75 = t74 * t14
        t76 = self.yC6 * t44
        t78 = s6
        t80 = self.g * self.a6 * t78
        t81 = t80 * t30
        t82 = c6
        t83 = self.g * t82
        t84 = self.zC6 * t83
        t86 = self.g * t78
        t87 = self.xC6 * t86
        t89 = t43 * t78
        t90 = self.g * self.zC6
        t91 = t90 * t89
        t93 = t43 * t82
        t94 = self.a6 * self.g
        t95 = t94 * t93
        t96 = t95 * t42
        t97 = (t76 * t14 - t84 * t30 + t87 * t30 - t71 * t33 - t91 * t42 - t18 - t37 - t55 - 
               t70 + t75 + t81 - t96)
        t98 = t78 * t33
        t99 = self.zC6 * t44
        t101 = t82 * t33
        t103 = self.g * self.a6 * t43
        t104 = t103 * t101
        t105 = self.xC6 * t44
        t107 = t4 * t27
        t108 = t80 * t107
        t110 = t69 * t42
        t112 = self.g * self.xC6
        t113 = t112 * t93
        t116 = t78 * t14
        t117 = self.zC6 * t48
        t119 = t82 * t14
        t121 = self.g * self.a6 * t47
        t122 = t121 * t119
        t123 = self.xC6 * t48
        t125 = (-t105 * t101 + t84 * t107 - t87 * t107 - t113 * t42 - t117 * t116 - t123 * t119 - 
                t71 * t42 - t99 * t98 - t104 - t108 - t110 - t122 + t63)
        t129 = self.g * self.d7 * t82
        t131 = self.zC7 * t83
        t134 = t82 * t4 * t23
        t135 = s7
        t136 = t135 * t43
        t137 = self.g * self.yC7
        t138 = t137 * t136
        t140 = c7
        t141 = t140 * t43
        t142 = self.g * self.xC7
        t143 = t142 * t141
        t147 = self.g * self.d7 * t47
        t149 = self.zC7 * t48
        t151 = t43 * t14
        t152 = self.g * t140
        t153 = self.yC7 * t152
        t155 = (t131 * t107 - t147 * t116 - t149 * t116 - t129 * t30 - t131 * t30 + t138 * t134 - 
                t143 * t134 + t153 * t151 - t104 - t108 - t110 - t18 - t37 - t55 + t63 - t70 + t75 + t81 - t96)
        t156 = self.g * t135
        t157 = self.xC7 * t156
        t160 = self.g * self.d7 * t43
        t162 = self.zC7 * t44
        t164 = t47 * t33
        t168 = t78 * t30
        t169 = self.xC7 * t152
        t171 = self.yC7 * t156
        t173 = t140 * t47
        t174 = t137 * t173
        t176 = t135 * t47
        t177 = t142 * t176
        t181 = t140 * t78
        t182 = t142 * t181
        t184 = t135 * t78
        t185 = t137 * t184
        t187 = t142 * t173
        t189 = t137 * t176
        t191 = self.d7 * self.g
        t192 = t191 * t89
        t194 = self.g * self.zC7
        t195 = t194 * t89
        t197 = (t138 * t101 - t143 * t101 + t129 * t107 - t182 * t107 + t185 * t107 - t187 * t119 + 
                t189 * t119 + t157 * t151 - t153 * t164 - t157 * t164 - t160 * t98 - t162 * t98 + 
                t169 * t168 - t171 * t168 - t174 * t42 - t177 * t42 - t192 * t42 - t195 * t42 - t122)
        t201 = t1 * t13
        t203 = t1 * t9
        t207 = t13 * t22
        t208 = self.xC4 * t2
        t210 = t13 * t26
        t211 = self.zC4 * t2
        t213 = t36 * t203
        t217 = t1 * t207
        t221 = self.g * self.d5 * t1
        t222 = t221 * t210
        t223 = self.yC5 * t2
        t229 = t69 * t217
        t231 = t1 * t210
        t232 = t80 * t231
        t235 = t78 * t203
        t237 = t82 * t203
        t238 = t121 * t237
        t241 = t95 * t217
        t243 = t74 * t203
        t245 = (t113 * t217 - t117 * t235 - t123 * t237 + t76 * t203 + t71 * t217 + t91 * t217 - 
                t84 * t231 + t87 * t231 - t213 + t222 + t229 + t232 - t238 + t241 + t243)
        t251 = t43 * t203
        t255 = (-t129 * t231 - t131 * t231 - t147 * t235 - t149 * t235 + t153 * t251 + t157 * t251 + 
                t192 * t217 - t213 + t229 + t232 - t238)
        t263 = t82 * t1
        t264 = t263 * t207
        t267 = (-t138 * t264 + t143 * t264 + t174 * t217 + t177 * t217 + t182 * t231 - t185 * t231 - 
                t187 * t237 + t189 * t237 + t195 * t217 + t222 + t241 + t243)
        t273 = t4 * t26
        t275 = t4 * t22
        t279 = t1 * t27
        t284 = t221 * t23
        t286 = t62 * t273
        t290 = t69 * t279
        t292 = t78 * t275
        t294 = t82 * t275
        t295 = t103 * t294
        t297 = t1 * t23
        t298 = t80 * t297
        t302 = t95 * t279
        t305 = t69 * t275
        t307 = t80 * t273
        t309 = (t105 * t294 + t113 * t279 - t84 * t273 + t87 * t273 + t71 * t275 + t71 * t279 + 
                t91 * t279 + t99 * t292 + t84 * t297 - t87 * t297 - t284 + t286 + t290 + t295 - t298 + t302 + t305 + t307)
        t313 = t263 * t27
        t321 = (-t129 * t273 - t131 * t273 - t138 * t294 - t138 * t313 + t143 * t294 + t143 * t313 + 
                t174 * t279 + t177 * t279 + t195 * t279 - t284 + t286 + t302 + t305 + t307)
        t325 = t78 * t273
        t329 = t47 * t275
        t335 = (t129 * t297 + t131 * t297 + t153 * t329 + t157 * t329 + t160 * t292 + t162 * t292 + 
                t169 * t325 - t171 * t325 - t182 * t297 + t185 * t297 + t192 * t279 + t290 + t295 - t298)
        t347 = t74 * t297
        t350 = t82 * t273
        t351 = t121 * t350
        t353 = t78 * t201
        t355 = t82 * t201
        t356 = t103 * t355
        t358 = t47 * t78
        t361 = t47 * t82
        t363 = t94 * t361 * t297
        t366 = t74 * t273
        t368 = t69 * t201
        t370 = (t112 * t361 * t297 + t90 * t358 * t297 - t105 * t355 - t117 * t325 - t123 * t350 - 
                t71 * t201 + t76 * t273 - t76 * t297 - t99 * t353 - t347 - t351 - t356 + t363 + t366 - t368)
        t374 = t43 * t273
        t379 = t47 * t201
        t384 = (t191 * t358 * t297 - t147 * t325 - t149 * t325 + t153 * t374 - t153 * t379 + 
                t157 * t374 - t157 * t379 - t160 * t353 - t162 * t353 - t347 - t351 - t356)
        t387 = t137 * t141
        t389 = t142 * t136
        t395 = t263 * t23
        t398 = (t194 * t358 * t297 + t138 * t355 - t143 * t355 - t187 * t350 + t187 * t395 + 
                t189 * t350 - t189 * t395 - t387 * t297 - t389 * t297 + t363 + t366 - t368)
        t403 = t103 * t325
        t406 = self.g * self.a6 * t82
        t407 = t406 * t279
        t408 = self.xC6 * t83
        t410 = self.zC6 * t86
        t413 = t121 * t353
        t418 = t94 * t89 * t297
        t422 = t406 * t275
        t424 = (t112 * t89 * t297 - t90 * t93 * t297 - t105 * t325 - t117 * t355 + t123 * t353 - 
                t408 * t275 - t410 * t275 - t408 * t279 - t410 * t279 + t99 * t350 - t403 - t407 + t413 + t418 - t422)
        t441 = t78 * t1 * t23
        t444 = (t137 * t135 * t82 * t279 - t142 * t140 * t82 * t279 - t191 * t93 * t297 - 
                t194 * t93 * t297 + t138 * t325 - t138 * t441 + t143 * t325 - t143 * t441 + 
                t187 * t353 - t189 * t353 + t418 - t422)
        t445 = self.zC7 * t86
        t448 = self.g * self.d7 * t78
        t458 = (-t147 * t355 - t149 * t355 + t160 * t350 + t162 * t350 - t169 * t294 + t171 * t294 - 
                t445 * t275 - t448 * t275 - t445 * t279 - t448 * t279 - t403 - t407 + t413)
        t464 = t47 * t273
        t467 = t43 * t201
        t482 = (t137 * t181 * t279 + t142 * t184 * t279 + t153 * t292 + t157 * t292 + t169 * t464 + 
                t169 * t467 - t171 * t464 - t171 * t467 + t174 * t355 + t177 * t355 - t187 * t297 + 
                t189 * t297 - t387 * t350 - t389 * t350 + t387 * t395 + t389 * t395)

        # Tính vector gq (7x1)
        gq = np.zeros((7, 1))
        gq[0, 0] = 0
        gq[1, 0] = (-((-self.zC2 * t2 - self.xC2 * t5) * self.m2 + 
                      (-t11 * t4 * t9 - t15 * t14 - self.yC3 * t2 - t18) * self.m3 + 
                      (-self.xC4 * t5 * t23 - self.zC4 * t5 * t27 - t38 * t14 + t31 * t30 - t34 * t33 - t18 - t37) * self.m4 + 
                      t66 * self.m5 + (t125 + t97) * self.m6 + (t197 + t155) * self.m7))
        gq[2, 0] = (-((t11 * t201 - t15 * t203) * self.m3 + 
                      (-t38 * t203 + t208 * t207 + t211 * t210 - t213) * self.m4 + 
                      (t58 * t203 - t60 * t203 + t223 * t210 + t45 * t217 + t49 * t217 - t213 + t222) * self.m5 + 
                      t245 * self.m6 + (t267 + t255) * self.m7))
        gq[3, 0] = (-((t208 * t27 - t211 * t23 + t31 * t273 + t34 * t275) * self.m4 + 
                      (-t223 * t23 + t64 * t273 + t45 * t275 + t49 * t275 + t45 * t279 + t49 * t279 - t284 + t286) * self.m5 + 
                      t309 * self.m6 + (t335 + t321) * self.m7))
        gq[4, 0] = (-((-t45 * t201 - t49 * t201 + t58 * t273 - t60 * t273 - t58 * t297 + t60 * t297) * self.m5 + 
                      t370 * self.m6 + (t398 + t384) * self.m7))
        gq[5, 0] = (-t424 * self.m6 - (t458 + t444) * self.m7)
        gq[6, 0] = -(t482 * self.m7)

        return gq

if __name__ == "__main__":
    try:
        gravity = GravityMatrix(json_path="/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json")
    except FileNotFoundError as e:
        print(e)
        exit(1)
    q = np.array([1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    gq = gravity.compute(q)
    print("Vector lực hấp dẫn gq:")
    print(gq)