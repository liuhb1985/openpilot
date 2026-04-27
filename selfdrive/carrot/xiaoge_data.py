#!/usr/bin/env python3
"""
小鸽数据广播模块
从系统获取实时数据，通过 TCP 连接传输到 7711 端口
"""

import json
import socket
import struct
import threading
import time
import traceback
from typing import Dict, Any, List, Tuple

import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import PC


class XiaogeDataBroadcaster:
    # 常量定义（参考 radard.py:28）
    RADAR_TO_CAMERA = 1.52  # 雷达相对于相机中心的偏移（米）
    RADAR_LAT_FACTOR = 0.5  # 未来位置预测时间因子（秒），参考 radard.py 的 radar_lat_factor
    FILTER_INIT_FRAMES = 3  # 滤波器初始化所需的最小帧数（参考 radard.py:520-546 的 cnt > 3）

    # 优化：车道分类和检测阈值（参考 radard.py:520-546）
    LANE_PROB_THRESHOLD = 0.1  # 车道内概率阈值，用于区分当前车道和侧方车道（参考 radard.py:520）
    CUTIN_PROB_THRESHOLD = 0.1  # Cut-in 检测的车道内概率阈值（参考 radard.py:520）

    # 优化：历史数据配置
    HISTORY_SIZE = 10  # 历史数据保留帧数，用于计算横向速度

    # 优化：动态置信度阈值参数（参考 radard.py:126-157 的匹配逻辑）
    CONFIDENCE_BASE_THRESHOLD = 0.5  # 基础置信度阈值
    CONFIDENCE_DISTANCE_THRESHOLD = 50.0  # 距离阈值（米），超过此距离要求更高置信度
    CONFIDENCE_DISTANCE_BOOST = 0.7  # 距离超过阈值时的置信度提升
    CONFIDENCE_VELOCITY_DIFF_THRESHOLD = 10.0  # 速度差异阈值（m/s）
    CONFIDENCE_VELOCITY_BOOST = 0.6  # 速度差异超过阈值时的置信度提升

    # 优化：侧方车辆筛选参数（参考 radard.py:560-569）
    SIDE_VEHICLE_MIN_DISTANCE = 5.0  # 侧方车辆最小距离（米）
    SIDE_VEHICLE_MAX_DPATH = 3.5  # 侧方车辆最大路径偏移（米）

    # 优化：车道宽度计算参数
    DEFAULT_LANE_HALF_WIDTH = 1.75  # 默认车道半宽 3.5m / 2
    MIN_LANE_HALF_WIDTH = 0.1  # 最小车道半宽阈值（避免除零）
    TARGET_LANE_WIDTH_DISTANCE = 20.0  # 车道宽度计算的目标距离（米）

    def get_ip_address(self):
        """获取本机局域网 IP 地址"""
        try:
            # 创建一个 UDP socket 连接到外部地址（不需要实际连接成功）
            # 这样可以自动选择正确的网络接口 IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def __init__(self):
        self.tcp_port = 7711  # TCP 端口号
        self.sequence = 0
        self.device_ip = self.get_ip_address()  # 获取本机 IP

        # TCP 客户端连接管理（线程安全）
        self.clients = {}  # {addr: conn} 存储活跃的客户端连接
        self.clients_lock = threading.Lock()  # 保护客户端列表的锁
        self.server_socket = None  # TCP 服务器 socket
        self.server_running = False  # 服务器运行状态标志

        # 订阅消息（纯视觉数据，不使用雷达）
        self.sm = messaging.SubMaster([
            'carState',
            'modelV2',
            'selfdriveState',
        ])

        # 时间滤波：用于平滑侧方车辆数据（指数移动平均）
        self.filter_alpha = 0.3
        self.lead_left_filtered = {'x': 0.0, 'v': 0.0, 'y': 0.0, 'vRel': 0.0, 'dPath': 0.0, 'yRel': 0.0}
        self.lead_right_filtered = {'x': 0.0, 'v': 0.0, 'y': 0.0, 'vRel': 0.0, 'dPath': 0.0, 'yRel': 0.0}
        self.lead_left_count = 0  # 连续检测计数（用于滤波器初始化）
        self.lead_right_count = 0

        # 历史数据缓存：用于计算横向速度（yvRel）和滤波器初始化
        self.lead_left_history: List[Dict[str, float]] = []  
        self.lead_right_history: List[Dict[str, float]] = []

        # 车道线数据缓存：避免重复计算
        self._lane_cache = {
            'lane_xs': None,
            'left_ys': None,
            'right_ys': None,
            'position_x': None,
            'position_y': None,
            'position_valid': False,  
            'cache_valid': False
        }

    def recvall(self, sock, n):
        """
        接收指定字节数的数据（TCP 需要确保接收完整数据）
        """
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:  # 连接已关闭
                return None
            data.extend(packet)
        return data

    def send_packet_to_client(self, conn, packet):
        """
        向单个客户端发送数据包
        """
        try:
            # TCP 发送数据包格式: [数据长度(4字节)][数据]
            size = len(packet)
            conn.sendall(struct.pack('!I', size))
            conn.sendall(packet)
            return True
        except (socket.error, OSError):
            # 连接已断开或发送失败
            return False

    def handle_client(self, conn, addr):
        """
        处理单个客户端连接
        """
        print(f"客户端已从 {addr} 连接")

        with self.clients_lock:
            self.clients[addr] = conn

        try:
            while self.server_running:
                # 接收客户端请求 (4字节命令)
                cmd_data = self.recvall(conn, 4)

                if not cmd_data:
                    break

                cmd = struct.unpack('!I', cmd_data)[0]

                if cmd == 2:  # 心跳请求
                    try:
                        conn.sendall(struct.pack('!I', 0))
                    except (socket.error, OSError):
                        break
        except Exception as e:
            print(f"处理客户端 {addr} 时出错: {e}")
        finally:
            with self.clients_lock:
                self.clients.pop(addr, None)
            try:
                conn.close()
            except:
                pass
            print(f"客户端 {addr} 已断开连接")

    def start_tcp_server(self):
        """
        启动 TCP 服务器（在独立线程中运行）
        """
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(5)

            self.server_running = True
            print(f"TCP 服务器已启动，正在监听端口 {self.tcp_port}")

            while self.server_running:
                try:
                    conn, addr = self.server_socket.accept()
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(conn, addr),
                        daemon=True
                    )
                    client_thread.start()
                except socket.error as e:
                    if self.server_running:
                        print(f"接受连接出错: {e}")
                    break
        except Exception as e:
            print(f"TCP 服务器错误: {e}")
            traceback.print_exc()
        finally:
            self.server_running = False
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
            print("TCP 服务器已停止")

    def broadcast_to_clients(self, packet):
        """
        向所有连接的客户端广播数据包
        """
        if not packet:
            return

        with self.clients_lock:
            clients_copy = dict(self.clients)

        dead_clients = []

        for addr, conn in clients_copy.items():
            if not self.send_packet_to_client(conn, packet):
                dead_clients.append(addr)

        if dead_clients:
            with self.clients_lock:
                for addr in dead_clients:
                    self.clients.pop(addr, None)
                    try:
                        if addr in clients_copy:
                            clients_copy[addr].close()
                    except:
                        pass

    def shutdown(self):
        """
        优雅关闭服务器
        """
        print("正在关闭 TCP 服务器...")
        self.server_running = False

        with self.clients_lock:
            for addr, conn in self.clients.items():
                try:
                    conn.close()
                    print(f"已关闭到 {addr} 的连接")
                except:
                    pass
            self.clients.clear()

        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        print("TCP 服务器关闭完成")

    def collect_car_state(self, carState) -> Dict[str, Any]:
        """收集本车状态数据 - 简化版（只保留超车决策必需字段）"""
        vEgo = float(carState.vEgo)
        if vEgo < 0:
            vEgo = 0.0

        return {
            'vEgo': vEgo,  # 实际速度
            'steeringAngleDeg': float(carState.steeringAngleDeg),  # 方向盘角度
            'leftLatDist': float(carState.leftLatDist),  # 车道距离（用于返回原车道判断）
            'leftBlindspot': bool(carState.leftBlindspot) if hasattr(carState, 'leftBlindspot') else False,  # 左侧盲区
            'rightBlindspot': bool(carState.rightBlindspot) if hasattr(carState, 'rightBlindspot') else False,  # 右侧盲区
        }

    def _update_lane_cache(self, modelV2):
        """更新车道线数据缓存，避免重复计算"""
        try:
            if not hasattr(modelV2, 'laneLines') or len(modelV2.laneLines) < 3:
                self._lane_cache['cache_valid'] = False
                return

            if len(modelV2.laneLines) <= 2:
                self._lane_cache['cache_valid'] = False
                return

            # 提取车道线数据
            lane_xs = [float(x) for x in modelV2.laneLines[1].x]
            left_ys = [float(y) for y in modelV2.laneLines[1].y]
            right_ys = [float(y) for y in modelV2.laneLines[2].y]

            if not (len(lane_xs) == len(left_ys) == len(right_ys)):
                self._lane_cache['cache_valid'] = False
                return

            # 验证 x 坐标是否单调递增（np.interp() 的要求）
            if len(lane_xs) < 2 or not all(lane_xs[i] < lane_xs[i + 1] for i in range(len(lane_xs) - 1)):
                self._lane_cache['cache_valid'] = False
                return

            self._lane_cache['lane_xs'] = lane_xs
            self._lane_cache['left_ys'] = left_ys
            self._lane_cache['right_ys'] = right_ys

            # 更新规划路径数据
            if hasattr(modelV2, 'position') and len(modelV2.position.x) > 0:
                position_x = [float(x) for x in modelV2.position.x]
                position_y = [float(y) for y in modelV2.position.y]

                if len(position_x) == len(position_y) and len(position_x) >= 2:
                    if all(position_x[i] < position_x[i + 1] for i in range(len(position_x) - 1)):
                        self._lane_cache['position_x'] = position_x
                        self._lane_cache['position_y'] = position_y
                        self._lane_cache['position_valid'] = True  
                    else:
                        self._lane_cache['position_x'] = None
                        self._lane_cache['position_y'] = None
                        self._lane_cache['position_valid'] = False
                else:
                    self._lane_cache['position_x'] = None
                    self._lane_cache['position_y'] = None
                    self._lane_cache['position_valid'] = False
            else:
                self._lane_cache['position_x'] = None
                self._lane_cache['position_y'] = None
                self._lane_cache['position_valid'] = False

            self._lane_cache['cache_valid'] = (
                len(self._lane_cache['lane_xs']) > 0 and
                len(self._lane_cache['left_ys']) > 0 and
                len(self._lane_cache['right_ys']) > 0
            )
        except (IndexError, AttributeError, ValueError):
            self._lane_cache['cache_valid'] = False

    def _calculate_dpath(self, dRel: float, yRel: float, yvRel: float = 0.0, vLead: float = 0.0) -> Tuple[float, float, float]:
        """
        计算车辆相对于规划路径的横向偏移 (dPath) 和车道内概率 (in_lane_prob)
        """
        if not self._lane_cache['cache_valid']:
            return 0.0, 0.0, 0.0

        try:
            lane_xs = self._lane_cache['lane_xs']
            left_ys = self._lane_cache['left_ys']
            right_ys = self._lane_cache['right_ys']

            def d_path_interp(dRel_val: float, yRel_val: float) -> Tuple[float, float]:
                """内部函数：计算指定距离处的 dPath 和 in_lane_prob"""
                left_lane_y = np.interp(dRel_val, lane_xs, left_ys)
                right_lane_y = np.interp(dRel_val, lane_xs, right_ys)

                center_y = (left_lane_y + right_lane_y) / 2.0
                lane_half_width = abs(right_lane_y - left_lane_y) / 2.0
                if lane_half_width < self.MIN_LANE_HALF_WIDTH:
                    lane_half_width = self.DEFAULT_LANE_HALF_WIDTH

                dist_from_center = yRel_val - center_y
                in_lane_prob = max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width))

                if self._lane_cache.get('position_valid', False):
                    path_y = np.interp(dRel_val, self._lane_cache['position_x'], self._lane_cache['position_y'])
                    dPath = yRel_val - path_y
                else:
                    dPath = dist_from_center

                return dPath, in_lane_prob

            dPath, in_lane_prob = d_path_interp(dRel, yRel)

            # 计算未来位置（用于 Cut-in 检测）
            future_dRel = dRel + vLead * self.RADAR_LAT_FACTOR
            future_yRel = yRel + yvRel * self.RADAR_LAT_FACTOR
            _, in_lane_prob_future = d_path_interp(future_dRel, future_yRel)

            return float(dPath), float(in_lane_prob), float(in_lane_prob_future)

        except (IndexError, ValueError, TypeError):
            return 0.0, 0.0, 0.0

    def _estimate_lateral_velocity(self, current_yRel: float, current_dRel: float, history: List[Dict[str, float]]) -> float:
        """
        估计横向速度 (yvRel)
        """
        if len(history) < 2:
            return 0.0

        try:
            recent = history[-2:]
            if len(recent) < 2:
                return 0.0

            dt = recent[1]['timestamp'] - recent[0]['timestamp']
            if dt <= 0:
                return 0.0

            dyRel = recent[1]['yRel'] - recent[0]['yRel']
            yvRel = dyRel / dt

            return float(yvRel)
        except (KeyError, IndexError, ZeroDivisionError):
            return 0.0

    def _calculate_lane_width(self, modelV2) -> float:
        """
        在约 20 米处计算车道宽度
        """
        try:
            if self._lane_cache.get('cache_valid', False):
                lane_xs = self._lane_cache['lane_xs']
                left_ys = self._lane_cache['left_ys']
                right_ys = self._lane_cache['right_ys']

                target_distance = self.TARGET_LANE_WIDTH_DISTANCE

                if (
                    len(lane_xs) > 0 and
                    target_distance <= max(lane_xs) and target_distance >= min(lane_xs)
                ):
                    left_y_at_dist = np.interp(target_distance, lane_xs, left_ys)
                    right_y_at_dist = np.interp(target_distance, lane_xs, right_ys)
                    lane_width = abs(right_y_at_dist - left_y_at_dist)
                    return lane_width

            if not hasattr(modelV2, 'laneLines') or len(modelV2.laneLines) < 3:
                return 0.0

            left_lane = modelV2.laneLines[1]
            right_lane = modelV2.laneLines[2]
            target_distance = self.TARGET_LANE_WIDTH_DISTANCE

            if (
                len(left_lane.x) > 0 and len(left_lane.y) > 0 and
                len(right_lane.x) > 0 and len(right_lane.y) > 0
            ):
                left_x = [float(x) for x in left_lane.x]
                left_y = [float(y) for y in left_lane.y]
                right_x = [float(x) for x in right_lane.x]
                right_y = [float(y) for y in right_lane.y]

                if (
                    len(left_x) >= 2 and all(left_x[i] < left_x[i + 1] for i in range(len(left_x) - 1)) and
                    len(right_x) >= 2 and all(right_x[i] < right_x[i + 1] for i in range(len(right_x) - 1)) and
                    target_distance <= max(left_x) and target_distance <= max(right_x) and
                    target_distance >= min(left_x) and target_distance >= min(right_x)
                ):
                    left_y_at_dist = np.interp(target_distance, left_x, left_y)
                    right_y_at_dist = np.interp(target_distance, right_x, right_y)
                    lane_width = abs(right_y_at_dist - left_y_at_dist)
                    return lane_width
        except (IndexError, ValueError, TypeError):
            pass

        return 0.0

    def collect_model_data(self, modelV2, carState=None) -> Dict[str, Any]:
        """
        收集模型数据 - 优化版本
        通过 modelV2 数据间接推断侧方车辆情况，替代 radarState
        """
        data = {}

        v_ego = 0.0
        if carState is not None and hasattr(carState, 'vEgo'):
            v_ego = float(carState.vEgo)
        elif hasattr(modelV2, 'velocity') and len(modelV2.velocity.x) > 0:
            v_ego = float(modelV2.velocity.x[0])

        self._update_lane_cache(modelV2)
        current_time = time.time()

        left_vehicles: List[Dict[str, Any]] = []
        right_vehicles: List[Dict[str, Any]] = []
        center_vehicles: List[Dict[str, Any]] = []

        for i, lead in enumerate(modelV2.leadsV3):
            lead_prob = float(lead.prob)

            x = float(lead.x[0]) if len(lead.x) > 0 else 0.0  # 纵向距离
            v = float(lead.v[0]) if len(lead.v) > 0 else 0.0  # 速度

            # 动态置信度阈值
            min_prob = self.CONFIDENCE_BASE_THRESHOLD
            if x > self.CONFIDENCE_DISTANCE_THRESHOLD:
                min_prob = max(min_prob, self.CONFIDENCE_DISTANCE_BOOST)
            if abs(v - v_ego) > self.CONFIDENCE_VELOCITY_DIFF_THRESHOLD:
                min_prob = max(min_prob, self.CONFIDENCE_VELOCITY_BOOST)

            if lead_prob < min_prob:
                continue

            y = float(lead.y[0]) if len(lead.y) > 0 else 0.0  # 横向位置
            a = float(lead.a[0]) if len(lead.a) > 0 else 0.0  # 加速度
            v_rel = v - v_ego  
            dRel = x - self.RADAR_TO_CAMERA
            yRel = -y  
            yvRel = 0.0  
            vLead = v_ego + v_rel  

            # 计算路径偏移和车道内概率（使用缓存和未来位置预测）
            dPath, in_lane_prob, in_lane_prob_future = self._calculate_dpath(dRel, yRel, yvRel, vLead)

            vehicle_data = {
                'x': x,
                'dRel': dRel,  # 相对于雷达的距离（已考虑 RADAR_TO_CAMERA 偏移）
                'y': y,
                'yRel': yRel,  # 相对于相机的横向位置
                'v': v,
                'vLead': vLead,  # 前车绝对速度
                'a': a,
                'vRel': v_rel,  # 相对速度
                'yvRel': yvRel,  # 横向速度
                'dPath': dPath,  # 路径偏移
                'inLaneProb': in_lane_prob,  # 车道内概率
                'inLaneProbFuture': in_lane_prob_future,  # 未来车道内概率
                'prob': lead_prob,
                'timestamp': current_time  
            }

            if in_lane_prob > self.LANE_PROB_THRESHOLD:
                center_vehicles.append(vehicle_data)
            elif yRel < 0:  
                left_vehicles.append(vehicle_data)
            else:  
                right_vehicles.append(vehicle_data)

        # 选定主前车 (lead0)
        if center_vehicles:
            lead0 = min(center_vehicles, key=lambda v: v['x'])
            data['lead0'] = {
                'x': lead0['x'],
                'y': lead0['y'],
                'v': lead0['v'],
                'prob': lead0['prob'],
            }
        elif len(modelV2.leadsV3) > 0:
            lead0 = modelV2.leadsV3[0]
            x = float(lead0.x[0]) if len(lead0.x) > 0 else 0.0
            y = float(lead0.y[0]) if len(lead0.y) > 0 else 0.0
            v = float(lead0.v[0]) if len(lead0.v) > 0 else 0.0
            data['lead0'] = {
                'x': x, 'y': y, 'v': v, 'prob': float(lead0.prob),
            }
        else:
            data['lead0'] = {'x': 0.0, 'y': 0.0, 'v': 0.0, 'prob': 0.0}

        # 侧方车辆筛选
        left_filtered = [
            v for v in left_vehicles
            if v['dRel'] > self.SIDE_VEHICLE_MIN_DISTANCE and abs(v['dPath']) < self.SIDE_VEHICLE_MAX_DPATH
        ]
        right_filtered = [
            v for v in right_vehicles
            if v['dRel'] > self.SIDE_VEHICLE_MIN_DISTANCE and abs(v['dPath']) < self.SIDE_VEHICLE_MAX_DPATH
        ]

        if left_filtered:
            lead_left = min(left_filtered, key=lambda vehicle: vehicle['dRel'])
            data['leadLeft'] = {
                'dRel': lead_left['dRel'],
                'vRel': lead_left['vRel'],
                'status': True,
            }
        else:
            data['leadLeft'] = {'dRel': 0.0, 'vRel': 0.0, 'status': False}

        if right_filtered:
            lead_right = min(right_filtered, key=lambda vehicle: vehicle['dRel'])
            data['leadRight'] = {
                'dRel': lead_right['dRel'],
                'vRel': lead_right['vRel'],
                'status': True,
            }
        else:
            data['leadRight'] = {'dRel': 0.0, 'vRel': 0.0, 'status': False}

        data['laneLineProbs'] = [
            float(modelV2.laneLineProbs[1]) if len(modelV2.laneLineProbs) >= 3 else 0.0,
            float(modelV2.laneLineProbs[2]) if len(modelV2.laneLineProbs) >= 3 else 0.0,
        ]

        meta = modelV2.meta
        data['meta'] = {
            'distanceToRoadEdgeLeft': float(meta.distanceToRoadEdgeLeft),
            'distanceToRoadEdgeRight': float(meta.distanceToRoadEdgeRight),
        }

        if hasattr(modelV2, 'orientationRate') and len(modelV2.orientationRate.z) > 0:
            orientation_rate_z = [float(x) for x in modelV2.orientationRate.z]
            data['curvature'] = {
                'maxOrientationRate': max(orientation_rate_z, key=abs),
            }
        else:
            data['curvature'] = {'maxOrientationRate': 0.0}

        return data

    def collect_system_state(self, selfdriveState) -> Dict[str, Any]:
        """收集系统状态"""
        return {
            'enabled': bool(selfdriveState.enabled) if selfdriveState else False,
            'active': bool(selfdriveState.active) if selfdriveState else False,
        }

    def create_packet(self, data: Dict[str, Any]) -> bytes:
        """创建数据包"""
        packet_data = {
            'version': 1,
            'sequence': self.sequence,
            'timestamp': time.time(),
            'ip': self.device_ip,
            'data': data
        }

        json_str = json.dumps(packet_data)
        packet_bytes = json_str.encode('utf-8')

        if len(packet_bytes) > 1024 * 1024:
            print(f"警告: 数据包过大 {len(packet_bytes)} 字节")

        return packet_bytes

    def broadcast_data(self):
        """主循环：收集数据并通过 TCP 推送给所有连接的客户端"""
        rk = Ratekeeper(20, print_delay_threshold=None)

        server_thread = threading.Thread(
            target=self.start_tcp_server,
            daemon=True
        )
        server_thread.start()

        time.sleep(0.5)

        print(f"XiaogeDataBroadcaster 启动，TCP 服务器监听端口 {self.tcp_port}")

        try:
            while True:
                try:
                    start_time = time.perf_counter()
                    self.sm.update(0)
                    data = {}

                    if self.sm.alive['carState']:
                        data['carState'] = self.collect_car_state(self.sm['carState'])

                    if self.sm.alive['modelV2']:
                        carState = self.sm['carState'] if self.sm.alive['carState'] else None
                        data['modelV2'] = self.collect_model_data(self.sm['modelV2'], carState)

                    if self.sm.alive['selfdriveState']:
                        data['systemState'] = self.collect_system_state(
                            self.sm['selfdriveState']
                        )

                    processing_time = time.perf_counter() - start_time
                    if processing_time > 0.05:
                        print(f"警告: 检测到处理缓慢: {processing_time * 1000:.1f}ms")

                    if data:
                        packet = self.create_packet(data)
                        try:
                            self.broadcast_to_clients(packet)
                            self.sequence += 1

                            if self.sequence % 100 == 0:
                                with self.clients_lock:
                                    client_count = len(self.clients)
                                print(f"已发送 {self.sequence} 个数据包到 {client_count} 个客户端，最后大小: {len(packet)} 字节")
                        except Exception as e:
                            print(f"向客户端发送数据包失败: {e}")
                    else:
                        try:
                            heartbeat_packet = {
                                'version': 1,
                                'sequence': self.sequence,
                                'timestamp': time.time(),
                                'ip': self.device_ip,
                                'data': {}
                            }
                            json_str = json.dumps(heartbeat_packet)
                            packet_bytes = json_str.encode('utf-8')
                            self.broadcast_to_clients(packet_bytes)
                            self.sequence += 1
                        except Exception:
                            pass

                    rk.keep_time()

                except KeyboardInterrupt:
                    print("\n收到关闭信号，正在优雅关闭...")
                    break
                except Exception as e:
                    print(f"XiaogeDataBroadcaster 错误: {e}")
                    traceback.print_exc()
                    time.sleep(1)
        finally:
            self.shutdown()


def main():
    broadcaster = XiaogeDataBroadcaster()
    broadcaster.broadcast_data()


if __name__ == "__main__":
    main()