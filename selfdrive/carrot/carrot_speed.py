# -*- coding: utf-8 -*-
"""
CarrotSpeedTable v2.1 (Params 后端, JSON+gzip, 1e-4° 网格, 8 个方向区间)
- 存储键: "CarrotSpeedTable"
- 格式 (JSON): {"format":"v5","dir_buckets":8,"cells":{"gy,gx":[[v,ts],...]} }
- 支持 gzip 压缩存储/加载 (默认开启)。仍可加载旧版非压缩 v2。
- 网格: 纬度/经度各按 1e-4° 对齐 (在韩国纬度约为 9~11m)
- 存储: 仅在对应单元格的一个位置记录单个速度 (带正负号)
    * 输入 > 0: 若原值为 None/负数/更小的正数则更新 (取更大正值)
    * 输入 < 0: 若原值为 None/正数/更小的负数则更新 (取更小的负值)
- 查询: 前方预读单元格 → 若无则搜索邻域 (ring=1)
    * 本单元格: 无时间过滤
    * 邻域: 仅使用旧数据 (年龄 ≥ 120s)
- 不进行清理: 保留旧数据
"""

import json, math, threading, time, gzip
from typing import Optional, Tuple, Dict, List
from openpilot.common.params import Params


# ---------- 地理/助手函数 ----------

def quantize_1e4(lat: float, lon: float) -> Tuple[int, int]:
    gy = int(math.floor(lat * 1e4 + 0.5))
    gx = int(math.floor(lon * 1e4 + 0.5))
    return gy, gx

def heading_to_bucket(heading_deg: float) -> int:
    # 固定 8 个方向区间
    step = 45.0  # 360/8
    i = int((heading_deg % 360.0) // step)
    if i < 0: return 0
    if i > 7: return 7
    return i

DIR_8 = {
    0: ( 1,  0),  # 北
    1: ( 1,  1),  # 东北
    2: ( 0,  1),  # 东
    3: (-1,  1),  # 东南
    4: (-1,  0),  # 南
    5: (-1, -1),  # 西南
    6: ( 0, -1),  # 西
    7: ( 1, -1),  # 西北
}

def project_point(lat: float, lon: float, heading_deg: float, distance_m: float) -> Tuple[float, float]:
    if distance_m <= 0.0:
        return lat, lon
    R = 6_371_000.0
    h = math.radians(heading_deg)
    dlat = (distance_m * math.cos(h)) / R
    dlon = (distance_m * math.sin(h)) / (R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def _is_gzip(data: bytes) -> bool:
    return len(data) >= 2 and data[0] == 0x1F and data[1] == 0x8B


# ---------- 主类 ----------

class CarrotSpeed:
    KEY = "CarrotSpeedTable"

    def __init__(self,
                 neighbor_ring: int = 1,
                 neighbor_old_threshold_s: int = 120,
                 use_gzip: bool = True,
                 gzip_level: int = 5):
    # 固定规格
        self.buckets = 8

        # 参数
        self.neighbor_ring = max(0, int(neighbor_ring))
        self.neighbor_old_threshold_s = int(neighbor_old_threshold_s)
        self.use_gzip = bool(use_gzip)
        self.gzip_level = int(gzip_level)

        # 内部状态
        self._lock = threading.RLock()
        # _cells[(gy,gx)] = [[value 或 None, ts(int 秒) 或 None] * 8]
        self._cells: Dict[Tuple[int, int], List[List[Optional[float]]]] = {}
        self._dirty = False
        self._last_save = 0
        self._params = Params()

        self._load_from_params_if_exists()

        self._last_hit = None        # (gy, gx, b, ts_when_read)
        self._last_hit_read_ms = 0   # 毫秒

    # ----- 内部工具 -----

    def _ensure_cell(self, gy: int, gx: int) -> List[List[Optional[float]]]:
        arr = self._cells.get((gy, gx))
        if arr is None:
            arr = [[None, None] for _ in range(self.buckets)]  # [v, ts]
            self._cells[(gy, gx)] = arr
        return arr

    def _now(self) -> int:
        # int 秒
        return int(time.time())

    def _age(self, ts: Optional[float]) -> Optional[int]:
        if ts is None:
            return None
        return self._now() - int(ts)

    def _neighbor_indices(self, gy: int, gx: int) -> List[Tuple[int, int]]:
        r = self.neighbor_ring
        if r <= 0:
            return []
        out = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dy == 0 and dx == 0:
                    continue
                out.append((gy + dy, gx + dx))
        return out

    def _neighbors_8(self, gy, gx):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                yield gy + dy, gx + dx

    def _try_cell_bucket_old(self, arr, b):
        v, ts = arr[b]
        if v is None or ts is None:
            return None, None
        if self._now() - int(ts) < self.neighbor_old_threshold_s:
            return None, None
        return float(v), b
    # ----- 公共 API -----
    def export_cells_around(self, lat: float, lon: float,
                            heading_deg: float,
                            ring: int = 1, max_points: int = 64) -> str:
        """
        根据当前 lat, lon 返回周边网格 (ring 范围) 中
        包含有效值的单元格 (lat, lon, speed) 列表的 JSON 格式。
        用于直接存入 Params("CarrotSpeedViz")。
        """
        gy0, gx0 = quantize_1e4(lat, lon)
        b0 = heading_to_bucket(heading_deg)
        pts = []

        with self._lock:
            for dy in range(-ring, ring + 1):
                for dx in range(-ring, ring + 1):
                    gy = gy0 + dy
                    gx = gx0 + dx
                    arr = self._cells.get((gy, gx))
                    if not arr:
                        continue

                    # 首先匹配精确方向区间 (b0)
                    v, ts = arr[b0]
                    if v is not None:
                        cell_lat = (gy + 0.5) * 1e-4
                        cell_lon = (gx + 0.5) * 1e-4
                        pts.append([cell_lat, cell_lon, float(v)])
                        if len(pts) >= max_points:
                            return json.dumps({"pts": pts}, separators=(",",":"))

                    # 如果没有，尝试匹配左右方向
                    for b in ((b0 - 1) % self.buckets, (b0 + 1) % self.buckets):
                        v, ts = arr[b]
                        if v is None:
                            continue
                        cell_lat = (gy + 0.5) * 1e-4
                        cell_lon = (gx + 0.5) * 1e-4
                        pts.append([cell_lat, cell_lon, float(v)])
                        if len(pts) >= max_points:
                            return json.dumps({"pts": pts}, separators=(",",":"))

        return json.dumps({"pts": pts}, separators=(",",":"))

    def add_sample(self, lat: float, lon: float, heading_deg: float, speed_signed: float):
        """
        存储单个速度 (带正负号)。
        - 基准单元格 (当前位置) + 基于方向的前/左/右各单元格记录相同速度
        - 在每个单元格中，方向区间 b 以及 b±1 这三个区间都以相同策略更新。
        - >0: 替换现有的负数/None，若是正数则平滑平均。
        - <0: 始终覆盖为新的负值 (紧急减速优先)。
        - ==0: 忽略
        """
        v_in = round(float(speed_signed), 1)
        if v_in == 0.0:
            return

        # 当前位置量化为网格
        gy0, gx0 = quantize_1e4(lat, lon)
        b = heading_to_bucket(heading_deg)
        now = self._now()

        # 对应方向区间的进方向网格向量
        dy_f, dx_f = DIR_8[b]

        # 基于航向的左/右 1 个、2 个单元格 (不使用 project_point)
        # 左 = 前进向量旋转 90° (dy,dx) -> (dx,-dy)
        # 右 = 前进向量旋转 -90° (dy,dx) -> (-dx,dy)
        dy_l1, dx_l1 = dx_f, -dy_f
        dy_r1, dx_r1 = -dx_f, dy_f

        dy_l2, dx_l2 = 2 * dy_l1, 2 * dx_l1
        dy_r2, dx_r2 = 2 * dy_r1, 2 * dx_r1

        # 记录的目标单元格: 中央 + 左/右 1 格 + 左/右 2 格
        target_cells = {
            (gy0, gx0),
            (gy0 + dy_l1, gx0 + dx_l1),
            (gy0 + dy_r1, gx0 + dx_r1),
            (gy0 + dy_l2, gx0 + dx_l2),
            (gy0 + dy_r2, gx0 + dx_r2),
        }

        with self._lock:
            for gy, gx in target_cells:
                arr = self._ensure_cell(gy, gx)

                # b, b-1, b+1 三个方向区间均以相同策略更新
                for off in (0, -1, +1):
                    bi = (b + off) % self.buckets
                    v_old, ts_old = arr[bi]

                    if v_old is None:
                        # 首次写入
                        arr[bi] = [v_in, now]
                    else:
                        if v_in > 0.0:
                            # 加速信息: 若为旧的正数则取平均，若是负数则替换
                            if v_old < 0.0:
                                # 负数 -> 正数，直接替换 (保留原时间戳)
                                arr[bi] = [v_in, ts_old]
                            else:
                                new_val = round((v_old + v_in) / 2.0, 1)
                                arr[bi] = [new_val, ts_old]
                        else:
                            # 减速信息: 始终以新的负值覆盖，保留原时间戳
                            arr[bi] = [v_in, ts_old]

            self._dirty = True

       
    def query_target(self, lat: float, lon: float, heading_deg: float, v_ego: float,
                     lookahead_s: float = 2.0) -> float:
        dist = max(0.0, float(v_ego) * float(lookahead_s))
        return self.query_target_dist(lat, lon, heading_deg, dist)
    
    def query_target_dist(self, lat: float, lon: float, heading_deg: float, dist: float) -> float:
        b = heading_to_bucket(heading_deg)

        cand_ds = [dist]
        for off in (3.0, -3.0):
            d2 = dist + off
            if d2 >= 0.0:
                cand_ds.append(d2)

        with self._lock:
            for d in cand_ds:
                y, x = project_point(lat, lon, heading_deg, d)
                gy, gx = quantize_1e4(y, x)

                arr = self._cells.get((gy, gx))
                if not arr:
                    continue

                v, b_sel = self._try_cell_bucket_old(arr, b)
                if v is not None:
                    now_sec = int(time.time())
                    self._last_hit = (gy, gx, b_sel, now_sec)
                    self._last_hit_read_ms = int(time.time() * 1000)
                    return v

        return 0.0
    
    def invalidate_last_hit(self, window_s: float = 2.0, action: str = "clear") -> bool:
        if self._last_hit is None:
            return False
        gy, gx, b, read_ts = self._last_hit
        now = int(time.time())
        if (now - int(read_ts)) > window_s:
            return False

        with self._lock:
            arr = self._cells.get((gy, gx))
            if not arr:
                return False

            # 失效 b, b-1, b+1
            for off in (0, -1, +1):
                bi = (b + off) % self.buckets
                v, ts = arr[bi]

                if action == "clear":
                    if v is not None and v < 0.0:
                      arr[bi] = [None, None]
                else:  # "age_bump"
                    if v is not None:
                        arr[bi] = [v, now]
                    else:
                        # 若无值则跳过
                        pass

            self._dirty = True

        return True
    
    def maybe_save(self, interval_s: int = 60) -> None:
        now = self._now()
        if (not self._dirty) or (now - self._last_save < interval_s):
            return
        self.save()

    def save(self) -> None:
        payload = self._encode_payload()
        self._params.put_nonblocking(self.KEY, payload)
        self._last_save = self._now()
        self._dirty = False

    def close(self) -> None:
        try:
            if self._dirty:
                self.save()
        except Exception:
            pass

    # ----- 序列化 -----

    def _encode_payload(self) -> bytes:
        with self._lock:
            cells = {}
            for (gy, gx), arr in self._cells.items():
                key = f"{gy},{gx}"
                # arr: [[v, ts], ...] (ts 为 int 或 None)
                cells[key] = [[None if v is None else float(v),
                               None if ts is None else int(ts)] for (v, ts) in arr]
            obj = {"format": "v5", "dir_buckets": self.buckets, "cells": cells}
            raw = json.dumps(obj, separators=(",", ":")).encode("utf-8")
            if self.use_gzip:
                return gzip.compress(raw, compresslevel=self.gzip_level)
            return raw

    def _load_from_params_if_exists(self) -> None:
        raw = self._params.get(self.KEY)
        if not raw:
            return
        try:
            data_bytes = raw
            if _is_gzip(data_bytes):
                data_bytes = gzip.decompress(data_bytes)
            data = json.loads(data_bytes.decode("utf-8"))

            # 若非 v5 版本则删除/重置
            if data.get("format") != "v5":
                self._params.remove(self.KEY)
                with self._lock:
                    self._cells = {}
                    self._dirty = False
                return

            buckets = int(data.get("dir_buckets", 8))
            if buckets != 8:
                # 方向区间不匹配也删除/重置
                self._params.remove(self.KEY)
                with self._lock:
                    self._cells = {}
                    self._dirty = False
                return

            restored: Dict[Tuple[int, int], List[List[Optional[float]]]] = {}
            for key, arr in data.get("cells", {}).items():
                gy, gx = map(int, key.split(","))
                fixed: List[List[Optional[float]]] = []
                if isinstance(arr, list) and len(arr) == 8:
                    for pair in arr:
                        if isinstance(pair, list) and len(pair) == 2:
                            v, ts = pair
                            v2 = None if v is None else float(v)
                            ts2 = None if ts is None else int(ts)
                            fixed.append([v2, ts2])
                        else:
                            fixed.append([None, None])
                else:
                    fixed = [[None, None] for _ in range(8)]
                restored[(gy, gx)] = fixed

            with self._lock:
                self._cells = restored
                self._dirty = False

        except Exception:
            # 解析失败时安全重置
            self._params.remove(self.KEY)
            with self._lock:
                self._cells = {}
                self._dirty = False
