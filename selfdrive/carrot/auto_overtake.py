#!/usr/bin/env python3
"""
现代汽车自动超车控制器 - 完整中文注释版
集成到OpenPilot中的自动超车控制器
访问地址: http://op_ip:8088

核心功能：
1. 智能车道检测和编号计算
2. 自动超车决策和变道控制  
3. 智能返回原车道机制
4. 安全条件检查和有效性评估
5. Web界面数据提供和控制接口

作者: Yuzucheng
版本: 2.0
日期: 2025
"""

import os
import sys
import json
import time
import threading
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from cereal import log
from collections import Counter

# 🎯 导入OpenPilot消息类型
LaneChangeState = log.LaneChangeState

DEBUG = False
def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

# 添加到OpenPilot路径
sys.path.append('/data/openpilot')
try:
    import cereal.messaging as messaging
    from common.realtime import Ratekeeper
    from common.params import Params
    OP_AVAILABLE = True
    print("✅ OpenPilot环境检测成功")
except ImportError:
    print("❌ 错误：未找到OpenPilot环境")
    sys.exit(1)


class AutoOvertakeController:
    """
    自动超车控制器主类
    
    负责管理整个自动超车系统的状态、决策和控制流程。
    与OpenPilot深度集成，提供安全的自动超车功能。
    """
    
    def __init__(self):
        """
        初始化自动超车控制器
        
        核心组件：
        - vehicle_data: 车辆状态数据存储
        - control_state: 控制状态数据存储  
        - config: 系统配置参数存储
        - 消息发布/订阅系统
        - UDP指令发送客户端
        - Web服务器接口
        """
        self.vehicle_data = self._init_vehicle_data()
        self.control_state = self._init_control_state()
        self.config = self._init_config()
        self.lane_change_cnt = 0
        self.lane_change_finishing = False
        
        # 🎯 关键状态变量初始化
        # OP赋值+1表示变道成功
        self.control_state.setdefault('overtakeSuccessCount', 0)
        # OP赋值1时表示OP正在控制转向，取消一切超车行为
        self.vehicle_data.setdefault('system_auto_control', 0)
        self.vehicle_data.setdefault('last_op_control_time', 0)

        # 📡 消息系统初始化
        self.pm = messaging.PubMaster(['autoOvertake'])
        self.sm = messaging.SubMaster([
            'carState', 'carControl', 'radarState',
            'modelV2', 'selfdriveState', 'liveLocationKalman', 'carrotMan'
        ])
        self.params = Params()

        # 📶 UDP客户端用于发送指令
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_ip = "127.0.0.1"  # 目标IP地址
        self.remote_port = 4211       # 目标端口

        # 🔢 指令索引和计时
        self.cmd_index = 0
        self.last_command_time = 0

        # 🧵 线程控制
        self.running = True
        self.data_thread = None
        self.web_server = None

        # 📥 加载持久化配置
        self.load_persistent_config()

        # 📊 车道序号稳定性优化
        self.lane_number_history = []    # 车道编号历史记录
        self.lane_count_history = []     # 车道总数历史记录  
        self.max_history_size = 10       # 历史记录最大长度

        print("✅ 控制器初始化完成")

    def _init_vehicle_data(self):
        """
        初始化车辆数据字典
        
        返回:
            dict: 包含所有车辆状态数据的字典
        """
        return {
            # 🚗 速度相关
            'v_cruise_kph': 0,      # 巡航速度 (km/h)
            'v_ego_kph': 0,         # 本车速度 (km/h) 
            'desire_speed': 0,      # 期望速度
            'lead_speed': 0,        # 前车速度
            'lead_distance': 0,     # 前车距离
            'lead_relative_speed': 0, # 前车相对速度
            
            # 🛣️ 车道信息
            'lane_count': 3,        # 车道总数
            'l_lane_width': 3.2,    # 左侧车道宽度
            'r_lane_width': 3.2,    # 右侧车道宽度  
            'l_edge_dist': 1.5,     # 左侧边缘距离
            'r_edge_dist': 1.5,     # 右侧边缘距离
            
            # 🎮 控制状态
            'IsOnroad': False,      # 是否在道路上
            'active': False,        # 系统是否激活
            'engaged': False,       # 巡航是否激活
            'steering_angle': 0.0,  # 方向盘角度
            'lat_a': 0.0,           # 横向加速度
            'road_curvature': 0.0,  # 道路曲率
            'max_curve': 0.0,       # 最大曲率
            
            # 👁️ 盲区检测
            'left_blindspot': False,    # 左侧盲区有车
            'right_blindspot': False,   # 右侧盲区有车
            'l_front_blind': False,     # 左侧前盲区
            'r_front_blind': False,     # 右侧前盲区
            
            # 🚘 侧方车辆信息
            'left_lead_speed': 0,           # 左侧前车速度
            'left_lead_distance': 0,        # 左侧前车距离
            'left_lead_relative_speed': 0,  # 左侧前车相对速度
            'right_lead_speed': 0,          # 右侧前车速度  
            'right_lead_distance': 0,       # 右侧前车距离
            'right_lead_relative_speed': 0, # 右侧前车相对速度
            
            # 🚦 车辆信号
            'blinker': 'none',      # 转向灯状态
            'gas_press': False,     # 油门踏板
            'break_press': False,   # 刹车踏板
            
            # 🔧 系统控制
            'system_auto_control': 0,   # OP自动控制状态
            'last_op_control_time': 0,  # 最后OP控制时间
            'atc_type': 'none'          # 自动控制类型
        }

    def _init_control_state(self):
        """
        初始化控制状态字典
        
        返回:
            dict: 包含所有控制状态数据的字典
        """
        return {
            # 📊 基本状态
            'current_status': '就绪',          # 当前状态描述
            'last_command': '',               # 最后执行的命令
            'blinker_state': 'none',          # 转向灯状态
            'cruise_active': False,           # 巡航激活状态
            
            # 🚀 超车状态
            'isOvertaking': False,            # 是否正在超车
            'overtakeState': '等待超车条件',   # 超车状态描述
            'overtakeReason': '分析道路情况中...', # 超车原因
            'overtakingCompleted': False,     # 超车是否完成
            'overtakeSuccessCount': 0,        # 超车成功次数
            'lastOvertakeDirection': '',      # 最后超车方向
            'lastOvertakeTime': 0,            # 最后超车时间
            
            # 🔄 变道控制
            'lane_change_in_progress': False, # 变道进行中
            'lastLaneChangeCommandTime': 0,   # 最后变道命令时间
            
            # 🧭 智能返回系统
            'net_lane_changes': 0,            # 净变道次数（左+1, 右-1）
            'max_return_attempts': 2,         # 最大返回尝试次数
            'return_attempts': 0,             # 当前返回尝试次数
            'return_conditions_met': False,   # 返回条件是否满足
            'return_timer_start': 0,          # 返回计时开始时间
            'last_return_direction': None,    # 最后返回方向
            'return_retry_count': 0,          # 返回重试次数
            'original_lane_clear': False,     # 🔥 新增：原车道前车是否已超越
            
            # ⏰ 跟车计时
            'follow_start_time': None,        # 跟车开始时间
            'is_following_slow_vehicle': False, # 是否跟随慢车
            'max_follow_time_reached': False, # 是否达到最大跟车时间
            
            # ❄️ 冷却系统
            'last_overtake_result': 'none',   # 最后超车结果
            'dynamic_cooldown': 8000,         # 动态冷却时间(ms)
            'consecutive_failures': 0,        # 连续失败次数
            
            # 🔧 自动超车专用
            'last_auto_overtake_time': 0,     # 最后自动超车时间
            'return_timeout': 40000,          # 返回超时时间(ms)
            'is_auto_overtake': False,        # 是否为自动超车
            
            # 🛑 OP控制冷却
            'op_control_cooldown': 0,         # OP控制冷却时间
            'last_op_control_end_time': 0     # OP控制结束时间
        }

    def _init_config(self):
        """
        初始化系统配置参数
        
        返回:
            dict: 包含所有配置参数的字典
        """
        return {
            # 🛣️ 道路和车道配置
            'road_type': 'highway',           # 道路类型: highway-高速, normal-普通
            'lane_count': 3,                  # 车道总数
            'current_lane_number': 2,         # 当前车道编号(1=最左)
            'lane_count_mode': 'auto',        # 车道计数模式: manual-手动, auto-自动, op-OP获取
            'manual_lane_count': 3,           # 手动模式下车道总数
            
            # 🚀 超车功能开关
            'autoOvertakeEnabled': False,     # 自动超车是否启用
            'shouldReturnToLane': True,       # 是否应返回原车道
            
            # ⚡ 超车触发条件参数
            'HIGHWAY_MIN_SPEED': 75.0,        # 高速公路最低超车速度(km/h)
            'NORMAL_ROAD_MIN_SPEED': 40.0,    # 普通道路最低超车速度(km/h)
            'CRUISE_SPEED_RATIO_THRESHOLD': 0.8,  # 巡航速度比例阈值(80%)
            'FOLLOW_TIME_GAP_THRESHOLD': 3.0, # 跟车时间距离阈值(秒)
            'MAX_FOLLOW_TIME': 120000,        # 最大跟车时间(毫秒)-2分钟
            'LEAD_RELATIVE_SPEED_THRESHOLD': -5.0, # 前车相对速度阈值(km/h)
            
            # 🛡️ 安全变道条件参数
            'MIN_LANE_WIDTH': 2.5,            # 最小车道宽度(米)
            'SAFE_LANE_WIDTH': 3.0,           # 安全车道宽度(米)
            'SIDE_LEAD_DISTANCE_MIN': 15.0,   # 侧方前车最小安全距离(米)
            'SIDE_RELATIVE_SPEED_THRESHOLD': 20, # 侧方车辆相对速度阈值(km/h)
            
            # 🏔️ 弯道检测参数
            'CURVATURE_THRESHOLD': 0.02,      # 曲率阈值
            'STEERING_THRESHOLD': 20.0,       # 方向盘角度阈值(度)
            
            # ❄️ 冷却时间参数(毫秒)
            'OVERTAKE_COOLDOWN_BASE': 8000,       # 基础冷却时间
            'OVERTAKE_COOLDOWN_FAILED': 3000,     # 失败后冷却时间
            'OVERTAKE_COOLDOWN_SUCCESS': 15000,   # 成功后冷却时间  
            'OVERTAKE_COOLDOWN_CONDITION': 5000,  # 条件不满足冷却时间
            
            # 📊 惩罚权重系统
            'PENALTY_WEIGHTS': {
                'lead_relative_speed': 2.0,   # 前车相对速度权重
                'side_lead_distance': 1.5,    # 侧前车距离权重
                'side_relative_speed': 1.8,   # 侧方相对速度权重
                'lane_width': 1.2,            # 车道宽度权重
                'blindspot': 3.0,             # 盲区权重
                'curvature': 1.5,             # 曲率权重
                'min_speed_advantage': 5.0    # 最小速度优势
            },
            
            # 🎯 决策阈值
            'PENALTY_THRESHOLD': 50.0,        # 惩罚阈值
            'MIN_SPEED_ADVANTAGE': 5.0,       # 最小速度优势(km/h)
            
            # 🛣️ 高速公路专用策略
            'HIGHWAY_STRATEGY': {
                'prefer_left_overtake': True,     # 优先左侧超车
                'avoid_rightmost_lane': True,     # 避免最右车道
                'emergency_lane_penalty': 100,    # 应急车道惩罚
                'fast_lane_bonus': 15,            # 快车道奖励
                'min_advantage_threshold': 3      # 最小优势阈值
            }
        }

    def load_persistent_config(self):
        """从持久化存储加载配置"""
        try:
            config_json = self.params.get("AutoOvertakeConfig")
            if config_json is not None:
                saved_config = json.loads(config_json)
                print(f"📥 加载保存的配置")
                # 只更新已存在的配置项
                for key, value in saved_config.items():
                    if key in self.config:
                        self.config[key] = value
            else:
                print("📥 使用默认配置")
        except Exception as e:
            print(f"⚠️ 加载配置失败: {e}")

    def save_persistent_config(self):
        """保存配置到持久化存储"""
        try:
            self.params.put("AutoOvertakeConfig", json.dumps(self.config))
            print("✅ 配置已保存")
        except Exception as e:
            print(f"⚠️ 保存配置失败: {e}")

    def calculate_lane_count(self):
        """
        根据当前模式计算车道总数
        
        模式说明:
        - manual: 使用用户手动设置的车道数
        - auto: 根据道路边缘和车道宽度自动计算
        - op: 使用OpenPilot提供的车道总数
        
        返回:
            int: 计算得到的车道总数
        """
        cfg = self.config
        mode = cfg['lane_count_mode']

        if mode == 'manual':
            # 🎮 手动模式：使用用户设置的值
            cfg['lane_count'] = cfg['manual_lane_count']
            return cfg['manual_lane_count']
        elif mode == 'auto':
            # 🤖 自动模式：根据道路边缘和车道宽度计算
            lane_count = self._calculate_auto_lane_count()
            cfg['lane_count'] = lane_count
            return lane_count
        elif mode == 'op':
            # 📡 OP获取模式：使用OpenPilot提供的车道总数
            op_lane_count = self._get_op_lane_count()
            if op_lane_count is not None:
                cfg['lane_count'] = op_lane_count
                return op_lane_count
            else:
                # OP获取失败，回退到自动模式
                debug_print("⚠️ OP车道总数获取失败，使用自动模式")
                lane_count = self._calculate_auto_lane_count()
                cfg['lane_count'] = lane_count
                return lane_count

        # 默认回退
        cfg['lane_count'] = 3
        return 3

    def _calculate_auto_lane_count(self):
        """
        自动计算车道总数 - 使用平滑算法
        
        计算逻辑:
        1. 获取左右边缘距离和车道宽度
        2. 计算总道路宽度和平均车道宽度
        3. 使用历史数据平滑计算结果
        4. 根据道路类型调整范围
        
        返回:
            int: 估计的车道总数
        """
        vd = self.vehicle_data

        left_edge_dist = vd.get('l_edge_dist', 0)
        right_edge_dist = vd.get('r_edge_dist', 0)
        left_lane_width = vd.get('l_lane_width', 3.2)
        right_lane_width = vd.get('r_lane_width', 3.2)

        # 计算平均车道宽度
        avg_lane_width = (left_lane_width + right_lane_width) / 2
        if avg_lane_width <= 0:
            avg_lane_width = 3.2  # 默认值

        total_road_width = left_edge_dist + right_edge_dist

        if total_road_width > 0 and avg_lane_width > 0:
            # 计算估计的车道数
            estimated_lanes = total_road_width / avg_lane_width
            
            # 📊 使用历史数据平滑
            self.lane_count_history.append(estimated_lanes)
            if len(self.lane_count_history) > self.max_history_size:
                self.lane_count_history.pop(0)
            
            smoothed_lanes = sum(self.lane_count_history) / len(self.lane_count_history)
            
            # 取整并限制范围
            lane_count = max(2, min(5, round(smoothed_lanes)))

            # 道路类型修正
            if self.config['road_type'] == 'highway':
                # 高速公路通常是2-4车道
                lane_count = max(2, min(4, lane_count))
            else:
                # 普通道路通常是2-3车道
                lane_count = max(2, min(3, lane_count))

            debug_print(f"🛣️ 自动计算车道总数: {estimated_lanes:.1f} → {lane_count}车道")
            return lane_count
        else:
            # 数据不足，使用默认值
            default_lanes = 3 if self.config['road_type'] == 'highway' else 2
            debug_print(f"⚠️ 自动计算数据不足，使用默认值: {default_lanes}车道")
            return default_lanes

    def _get_op_lane_count(self):
        """
        从OpenPilot获取车道总数
        
        返回:
            int or None: 车道总数或None(获取失败时)
        """
        try:
            if self.sm.alive['modelV2']:
                # 这里需要根据实际的OpenPilot消息结构来获取车道总数
                # 暂时返回None，表示需要根据实际情况实现
                return None
            return None
        except Exception as e:
            debug_print(f"❌ 获取OP车道总数失败: {e}")
            return None

    def update_lane_number(self):
        """
        更新车道编号 - 使用稳定性优化算法
        
        关键逻辑:
        - 1 = 最左车道，最大编号 = 最右车道
        - 使用历史数据平滑车道编号
        - 采用多数投票机制确定稳定值
        """
        vd = self.vehicle_data
        cfg = self.config

        # 重新计算车道总数
        self.calculate_lane_count()

        left_lane_width = vd.get('l_lane_width', 3.2)
        right_lane_width = vd.get('r_lane_width', 3.2)
        left_edge_dist = vd.get('l_edge_dist', 1.5)
        right_edge_dist = vd.get('r_edge_dist', 1.5)

        total_lanes = cfg['lane_count']

        avg_lane_width = (left_lane_width + right_lane_width) / 2
        if avg_lane_width <= 0:
            avg_lane_width = 3.2

        if left_edge_dist > 0 and right_edge_dist > 0 and avg_lane_width > 0:
            # 🧮 基于相对位置计算车道编号
            total_road_width = left_edge_dist + right_edge_dist
            relative_position = left_edge_dist / total_road_width
            
            # 关键：1=最左车道，最大编号=最右车道
            lane_number = 1 + round(relative_position * (total_lanes - 1))
            lane_number = max(1, min(total_lanes, lane_number))
            
            # 📊 历史数据平滑
            self.lane_number_history.append(lane_number)
            if len(self.lane_number_history) > self.max_history_size:
                self.lane_number_history.pop(0)
            
            # 🗳️ 多数投票机制
            if len(self.lane_number_history) >= 3:
                counter = Counter(self.lane_number_history)
                most_common = counter.most_common(1)[0][0]
                if counter[most_common] > len(self.lane_number_history) / 2:
                    lane_number = most_common

            if lane_number != cfg['current_lane_number']:
                cfg['current_lane_number'] = lane_number
                debug_print(f"🛣️ 更新车道编号: {lane_number} (总数: {total_lanes})")
        else:
            # 数据不足，使用默认值
            default_lane = 2 if total_lanes >= 2 else 1
            if default_lane != cfg['current_lane_number']:
                cfg['current_lane_number'] = default_lane
                debug_print(f"⚠️ 车道数据不足，使用默认车道: {default_lane}")

    def calculate_time_gap(self):
        """
        计算跟车时间距离（秒）
        
        公式:
        时间距离 = 前车距离(米) / 本车速度(米/秒)
        
        返回:
            float: 时间距离(秒)
        """
        vd = self.vehicle_data

        if vd['lead_distance'] <= 0 or vd['v_ego_kph'] <= 0:
            return 0

        # 将本车速度从km/h转换为m/s
        v_ego_ms = vd['v_ego_kph'] / 3.6
        time_gap = vd['lead_distance'] / v_ego_ms if v_ego_ms > 0 else 0
        return time_gap

    def update_vehicle_data(self):
        """更新车辆数据 - 使用真实OpenPilot数据"""
        try:
            # 基础在线状态
            isOnroad = self.params.get_bool("IsOnroad")
            self.vehicle_data['IsOnroad'] = isOnroad

            if isOnroad:
                self.sm.update(100)  # 100ms超时
            else:
                self.sm.update(0)  # 不阻塞，不触发等待

            if isOnroad:
                # 车辆状态数据
                if self.sm.alive['carState']:
                    carState = self.sm['carState']

                    # 速度相关
                    v_ego_kph = int(carState.vEgo * 3.6 + 0.5) if carState.vEgo else 0
                    v_cruise_kph = carState.vCruise

                    self.vehicle_data.update({
                        'v_ego_kph': v_ego_kph,
                        'v_cruise_kph': v_cruise_kph,
                        'cruise_speed': v_cruise_kph,
                        'steering_angle': round(carState.steeringAngleDeg, 1) if carState.steeringAngleDeg else 0.0,
                        'blinker': self._get_blinker_state(carState.leftBlinker, carState.rightBlinker),
                        'gas_press': carState.gasPressed,
                        'break_press': carState.brakePressed,
                        'engaged': carState.cruiseState.enabled,
                        'left_blindspot': bool(carState.leftBlindspot),
                        'right_blindspot': bool(carState.rightBlindspot)
                    })

                    # 加速度
                    if carState.aEgo:
                        self.vehicle_data['lat_a'] = round(carState.aEgo, 1)

                # 雷达数据 - 前车
                if self.sm.alive['radarState']:
                    radarState = self.sm['radarState']

                    # 主前车
                    if radarState.leadOne.status:
                        leadOne = radarState.leadOne
                        self.vehicle_data.update({
                            'lead_distance': int(leadOne.dRel),
                            'lead_speed': int(leadOne.vLead * 3.6),
                            'lead_relative_speed': int(leadOne.vRel * 3.6)
                        })
                    else:
                        # 前车消失时重置数据
                        self.vehicle_data.update({
                            'lead_distance': 0,
                            'lead_speed': 0,
                            'lead_relative_speed': 0
                        })

                    # 左侧前车
                    if radarState.leadLeft.status:
                        leadLeft = radarState.leadLeft
                        self.vehicle_data.update({
                            'left_lead_distance': int(leadLeft.dRel),
                            'left_lead_speed': int(leadLeft.vLead * 3.6),
                            'left_lead_relative_speed': int(leadLeft.vRel * 3.6)
                        })
                    else:
                        # 左侧前车消失时重置数据
                        self.vehicle_data.update({
                            'left_lead_distance': 0,
                            'left_lead_speed': 0,
                            'left_lead_relative_speed': 0
                        })

                    # 右侧前车
                    if radarState.leadRight.status:
                        leadRight = radarState.leadRight
                        self.vehicle_data.update({
                            'right_lead_distance': int(leadRight.dRel),
                            'right_lead_speed': int(leadRight.vLead * 3.6),
                            'right_lead_relative_speed': int(leadRight.vRel * 3.6)
                        })
                    else:
                        # 右侧前车消失时重置数据
                        self.vehicle_data.update({
                            'right_lead_distance': 0,
                            'right_lead_speed': 0,
                            'right_lead_relative_speed': 0
                        })

                # 期望速度
                self.vehicle_data['desire_speed'] = 90

            carrot_left_blind = False
            carrot_right_blind = False
            current_time = time.time() * 1000
            
            # 记录OP控制状态变化
            old_op_control = self.vehicle_data['system_auto_control']
            
            if self.sm.alive['carrotMan']:
                carrotMan = self.sm['carrotMan']
                # 更精确的OP控制状态检测
                is_op_controlling = ("none" not in carrotMan.atcType and 
                                   "prepare" not in carrotMan.atcType and
                                   "standby" not in carrotMan.atcType)
                
                if is_op_controlling:
                    self.vehicle_data['system_auto_control'] = 1
                    self.vehicle_data['last_op_control_time'] = current_time
                    # OP控制开始时重置冷却时间
                    if old_op_control == 0:
                        debug_print("🔄 OP控制开始，重置自动超车状态")
                        self.control_state['op_control_cooldown'] = 0
                        self.control_state['last_op_control_end_time'] = 0
                else:
                    self.vehicle_data['system_auto_control'] = 0
                    # 记录OP控制结束时间
                    if old_op_control == 1:
                        self.control_state['last_op_control_end_time'] = current_time
                        self.control_state['op_control_cooldown'] = 3000  # OP控制后3秒冷却
                        debug_print(f"🔄 OP控制结束，开始{self.control_state['op_control_cooldown']}ms冷却")
                
                carrot_left_blind = carrotMan.leftBlind
                carrot_right_blind = carrotMan.rightBlind

            # 模型数据 - 车道信息和盲区
            if self.sm.alive['modelV2']:
                modelV2 = self.sm['modelV2']
                meta = modelV2.meta

                self.vehicle_data.update({
                    'blinker': meta.blinker,
                    'l_front_blind': meta.leftFrontBlind or carrot_left_blind,
                    'r_front_blind': meta.rightFrontBlind or carrot_right_blind,
                    'l_lane_width': round(meta.laneWidthLeft, 1),
                    'r_lane_width': round(meta.laneWidthRight, 1),
                    'l_edge_dist': round(meta.distanceToRoadEdgeLeft, 1),
                    'r_edge_dist': round(meta.distanceToRoadEdgeRight, 1)
                })

                if self.lane_change_finishing and meta.laneChangeState != LaneChangeState.laneChangeFinishing:
                    self.lane_change_cnt += 1
                    self.control_state['overtakeSuccessCount'] += 1
                    self.lane_change_finishing = False
                    debug_print("🔄 检测到变道完成，强制更新车道序号")
                    self.update_lane_number()
                if meta.laneChangeState == LaneChangeState.laneChangeFinishing:
                    self.lane_change_finishing = True

            # 自驾状态
            if self.sm.alive['selfdriveState']:
                selfdriveState = self.sm['selfdriveState']
                self.vehicle_data['active'] = "on" if selfdriveState.active else "off"

        except Exception as e:
            debug_print(f"更新车辆数据错误: {e}")

    def _get_blinker_state(self, left_blinker, right_blinker):
        """获取转向灯状态"""
        if left_blinker and right_blinker:
            return "hazard"
        elif left_blinker:
            return "left"
        elif right_blinker:
            return "right"
        else:
            return "none"

    def update_following_status(self):
        """更新跟车状态"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        now = time.time() * 1000

        time_gap = self.calculate_time_gap()
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0

        is_following = (
            vd['lead_distance'] > 0 and (
                vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD'] or
                (0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']) or
                speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']
            )
        )

        if is_following:
            if cs['follow_start_time'] is None:
                cs['follow_start_time'] = now
                cs['is_following_slow_vehicle'] = True
                debug_print(f"🚗 开始跟车计时")
            follow_duration = now - cs['follow_start_time']
            if follow_duration >= cfg['MAX_FOLLOW_TIME'] and not cs['max_follow_time_reached']:
                cs['max_follow_time_reached'] = True
                minutes = cfg['MAX_FOLLOW_TIME'] // 60000
                cs['overtakeReason'] = f"跟车时间超过{minutes}分钟，强制超车"
                debug_print(f"⏰ 达到最大跟车时间: {follow_duration/60000:.1f}分钟")
        else:
            if cs['follow_start_time'] is not None:
                debug_print(f"🔄 重置跟车计时器")
            cs['follow_start_time'] = None
            cs['is_following_slow_vehicle'] = False
            cs['max_follow_time_reached'] = False

    def check_op_control_cooldown(self):
        """检查OP控制后的冷却时间"""
        cs = self.control_state
        current_time = time.time() * 1000
        
        if cs['op_control_cooldown'] > 0:
            elapsed = current_time - cs['last_op_control_end_time']
            if elapsed < cs['op_control_cooldown']:
                remaining = (cs['op_control_cooldown'] - elapsed) / 1000
                cs['overtakeReason'] = f"OP控制后冷却中，请等待{remaining:.1f}秒"
                return True
            else:
                cs['op_control_cooldown'] = 0
                debug_print("🔄 OP控制冷却时间结束")
        
        return False

    def calculate_dynamic_cooldown(self):
        """计算动态冷却时间"""
        cs = self.control_state
        cfg = self.config

        base_cooldown = cfg['OVERTAKE_COOLDOWN_BASE']

        if cs['last_overtake_result'] == 'success':
            cooldown = cfg['OVERTAKE_COOLDOWN_SUCCESS']
            cs['consecutive_failures'] = 0
        elif cs['last_overtake_result'] == 'failed':
            cooldown = cfg['OVERTAKE_COOLDOWN_FAILED']
            cs['consecutive_failures'] += 1
        elif cs['last_overtake_result'] == 'condition':
            cooldown = cfg['OVERTAKE_COOLDOWN_CONDITION']
            cs['consecutive_failures'] += 1
        else:
            cooldown = base_cooldown

        if cs['consecutive_failures'] > 3:
            penalty = min(10000, cs['consecutive_failures'] * 2000)
            cooldown += penalty
            debug_print(f"⚠️ 连续失败{cs['consecutive_failures']}次，增加冷却时间{penalty/1000}秒")

        if self.config['road_type'] == 'highway':
            cooldown = max(5000, cooldown * 0.8)
        else:
            cooldown = cooldown * 1.2

        cs['dynamic_cooldown'] = cooldown
        return cooldown

    def get_trigger_conditions(self):
        """获取当前触发超车的条件状态"""
        vd = self.vehicle_data
        cfg = self.config
        cs = self.control_state

        conditions = []

        if cs['max_follow_time_reached']:
            conditions.append("⏰ 最大跟车时间触发")
            return conditions

        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0

        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            conditions.append(f"🚗 前车较慢: {vd['lead_relative_speed']}km/h")
            return conditions

        time_gap = self.calculate_time_gap()
        if 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            conditions.append(f"⏱️ 跟车时间: {time_gap:.1f}秒")
            return conditions

        if speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            conditions.append(f"🚀 速度比例: {speed_ratio*100:.0f}%")
            return conditions

        return conditions

    def check_overtake_conditions(self):
        """检查超车条件"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        now = time.time() * 1000

        if vd['system_auto_control'] == 1:
            cs['overtakeReason'] = "OP自动控制中，暂停超车"
            cs['last_overtake_result'] = 'condition'
            return False

        if self.check_op_control_cooldown():
            cs['last_overtake_result'] = 'condition'
            return False

        if not vd['IsOnroad']:
            cs['overtakeReason'] = "车辆不在道路上"
            cs['last_overtake_result'] = 'condition'
            return False

        if not vd['engaged']:
            cs['overtakeReason'] = "巡航未激活"
            cs['last_overtake_result'] = 'condition'
            return False

        if cs['max_follow_time_reached']:
            cs['overtakeReason'] = f"跟车时间超过{cfg['MAX_FOLLOW_TIME']//60000}分钟，强制超车"
            return True

        if vd['lead_distance'] <= 0:
            cs['overtakeReason'] = "前方无车辆"
            cs['last_overtake_result'] = 'condition'
            return False

        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0

        has_trigger = False
        trigger_reason = ""

        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"前车相对速度{vd['lead_relative_speed']}km/h"

        time_gap = self.calculate_time_gap()
        if not has_trigger and 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"跟车时间距离{time_gap:.1f}秒"

        if not has_trigger and speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"速度比例{speed_ratio*100:.0f}%"

        if not has_trigger:
            cs['overtakeReason'] = "未满足任何超车触发条件"
            cs['last_overtake_result'] = 'condition'
            return False

        if cfg['road_type'] == 'highway' and vd['v_ego_kph'] < cfg['HIGHWAY_MIN_SPEED']:
            cs['overtakeReason'] = f"高速公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            cs['last_overtake_result'] = 'condition'
            return False

        if cfg['road_type'] == 'normal' and vd['v_ego_kph'] < cfg['NORMAL_ROAD_MIN_SPEED']:
            cs['overtakeReason'] = f"普通公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            cs['last_overtake_result'] = 'condition'
            return False

        current_cooldown = self.calculate_dynamic_cooldown()
        if now - cs['lastOvertakeTime'] < current_cooldown:
            remaining = (current_cooldown - (now - cs['lastOvertakeTime'])) / 1000
            reason_suffix = ""
            if cs['last_overtake_result'] == 'success':
                reason_suffix = "（成功超车后冷却）"
            elif cs['last_overtake_result'] == 'failed':
                reason_suffix = "（超车失败后快速重试）"
            elif cs['last_overtake_result'] == 'condition':
                reason_suffix = "（条件不满足冷却）"

            cs['overtakeReason'] = f"超车冷却中，请等待{remaining:.1f}秒{reason_suffix}"
            return False

        cs['overtakeReason'] = f"触发超车: {trigger_reason}"
        return True

    def evaluate_overtake_effectiveness(self, direction):
        """评估超车有效性"""
        vd = self.vehicle_data
        cfg = self.config

        if direction == "LEFT":
            side_lead_speed = vd['left_lead_speed']
            side_lead_distance = vd['left_lead_distance']
            side_relative_speed = vd['left_lead_relative_speed']
        else:
            side_lead_speed = vd['right_lead_speed']
            side_lead_distance = vd['right_lead_distance']
            side_relative_speed = vd['right_lead_relative_speed']

        current_speed = vd['v_ego_kph']
        current_lead_speed = vd['lead_speed']

        effectiveness = 100
        reasons = []

        if side_lead_speed > 0 and side_lead_speed < current_lead_speed - 5:
            effectiveness -= 30
            reasons.append(f"侧前车速度{side_lead_speed}km/h比当前前车{current_lead_speed}km/h更慢")

        if side_lead_speed > 0 and side_lead_speed < current_speed - 10:
            effectiveness -= 40
            reasons.append(f"侧前车速度{side_lead_speed}km/h比本车{current_speed}km/h慢太多")

        if side_lead_distance > 0 and side_lead_distance < 20:
            effectiveness -= 20
            reasons.append(f"侧前车距离{side_lead_distance}m过近")

        if side_relative_speed < -15:
            effectiveness -= 25
            reasons.append(f"侧前车相对速度{side_relative_speed}km/h，明显更慢")

        if direction == "RIGHT" and cfg['road_type'] == 'highway':
            effectiveness -= 10
            reasons.append("右侧车道通常较慢")

        effectiveness = max(0, effectiveness)
        return effectiveness, reasons

    def is_overtake_effective(self, direction):
        """判断超车是否有效"""
        effectiveness, reasons = self.evaluate_overtake_effectiveness(direction)
        min_effectiveness = 60
        is_effective = effectiveness >= min_effectiveness
        return is_effective, effectiveness, reasons

    def check_lane_safety(self, side):
        """检查车道安全性"""
        vd = self.vehicle_data
        cfg = self.config

        if side == "left":
            if vd['l_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"
            if vd['left_blindspot'] or vd['l_front_blind']:
                return False, "盲区有车⚠️禁止变道"
            if vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                return False, "侧车过近⚠️禁止变道"
            if abs(vd['left_lead_relative_speed']) > cfg['SIDE_RELATIVE_SPEED_THRESHOLD']:
                return False, "侧车相对⚠️速度过高"
            return True, "安全"

        elif side == "right":
            if vd['r_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"
            if vd['right_blindspot'] or vd['r_front_blind']:
                return False, "盲区有车⚠️禁止变道"
            if vd['right_lead_distance'] > 0 and vd['right_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                return False, "侧车过近⚠️禁止变道"
            if abs(vd['right_lead_relative_speed']) > cfg['SIDE_RELATIVE_SPEED_THRESHOLD']:
                return False, "侧车相对⚠️速度过高"
            return True, "安全"

        return False, "未知方向"

    def evaluate_lane_suitability(self, side):
        """评估车道适合度"""
        vd = self.vehicle_data
        cfg = self.config
        current_lane = cfg['current_lane_number']
        total_lanes = cfg['lane_count']

        if side == "left":
            target_lane = current_lane - 1
        else:
            target_lane = current_lane + 1

        if self.is_emergency_lane(target_lane):
            return 0, ["🚫 应急车道，禁止行驶"]

        penalty_score = 0
        analysis = []
        weights = cfg['PENALTY_WEIGHTS']

        if side == "left":
            if vd['left_blindspot'] or vd['l_front_blind']:
                penalty_score += 100
                analysis.append("❌ 盲区有车")
                return penalty_score, analysis

            lane_width = vd['l_lane_width']
            if lane_width < cfg['MIN_LANE_WIDTH']:
                penalty_score += 80
                analysis.append(f"❌ 车道过窄: {lane_width}m")
            elif lane_width < cfg['SAFE_LANE_WIDTH']:
                penalty_score += (cfg['SAFE_LANE_WIDTH'] - lane_width) * weights['lane_width'] * 10
                analysis.append(f"⚠️ 车道略窄: {lane_width}m")
            else:
                analysis.append(f"✅ 车道宽度正常: {lane_width}m")

            if cfg['road_type'] == 'highway' and target_lane == 1:
                analysis.append("🚀 快车道 - 超车优先")
                penalty_score -= 15

            side_distance = vd['left_lead_distance']
            if side_distance > 0:
                if side_distance < cfg['SIDE_LEAD_DISTANCE_MIN']:
                    penalty_score += (cfg['SIDE_LEAD_DISTANCE_MIN'] - side_distance) * weights['side_lead_distance']
                    analysis.append(f"⚠️ 侧前车过近: {side_distance}m")
                else:
                    distance_advantage = side_distance - cfg['SIDE_LEAD_DISTANCE_MIN']
                    penalty_score -= min(distance_advantage * 0.5, 20)
                    analysis.append(f"✅ 侧前车安全距离: {side_distance}m")

            side_relative_speed = vd['left_lead_relative_speed']
            if side_relative_speed != 0:
                if side_relative_speed < -weights['min_speed_advantage']:
                    penalty_score += abs(side_relative_speed) * weights['side_relative_speed']
                    analysis.append(f"❌ 侧前车较慢: {side_relative_speed}km/h")
                elif side_relative_speed > weights['min_speed_advantage']:
                    speed_advantage = min(side_relative_speed * 0.8, 25)
                    penalty_score -= speed_advantage
                    analysis.append(f"✅ 侧前车较快: +{side_relative_speed}km/h")
                else:
                    analysis.append(f"➖ 侧前车速度相当: {side_relative_speed}km/h")

        elif side == "right":
            if vd['right_blindspot'] or vd['r_front_blind']:
                penalty_score += 100
                analysis.append("❌ 盲区有车")
                return penalty_score, analysis

            lane_width = vd['r_lane_width']
            if lane_width < cfg['MIN_LANE_WIDTH']:
                penalty_score += 80
                analysis.append(f"❌ 车道过窄: {lane_width}m")
            elif lane_width < cfg['SAFE_LANE_WIDTH']:
                penalty_score += (cfg['SAFE_LANE_WIDTH'] - lane_width) * weights['lane_width'] * 10
                analysis.append(f"⚠️ 车道略窄: {lane_width}m")
            else:
                analysis.append(f"✅ 车道宽度正常: {lane_width}m")

            if self.is_emergency_lane(target_lane):
                return 0, ["🚫 应急车道，禁止行驶"]

            if cfg['road_type'] == 'highway' and target_lane == total_lanes:
                analysis.append("⚠️ 右侧车道通常较慢")
                penalty_score += 10

            side_distance = vd['right_lead_distance']
            if side_distance > 0:
                if side_distance < cfg['SIDE_LEAD_DISTANCE_MIN']:
                    penalty_score += (cfg['SIDE_LEAD_DISTANCE_MIN'] - side_distance) * weights['side_lead_distance']
                    analysis.append(f"⚠️ 侧前车过近: {side_distance}m")
                else:
                    distance_advantage = side_distance - cfg['SIDE_LEAD_DISTANCE_MIN']
                    penalty_score -= min(distance_advantage * 0.5, 20)
                    analysis.append(f"✅ 侧前车安全距离: {side_distance}m")

            side_relative_speed = vd['right_lead_relative_speed']
            if side_relative_speed != 0:
                if side_relative_speed < -weights['min_speed_advantage']:
                    penalty_score += abs(side_relative_speed) * weights['side_relative_speed']
                    analysis.append(f"❌ 侧前车较慢: {side_relative_speed}km/h")
                elif side_relative_speed > weights['min_speed_advantage']:
                    speed_advantage = min(side_relative_speed * 0.8, 25)
                    penalty_score -= speed_advantage
                    analysis.append(f"✅ 侧前车较快: +{side_relative_speed}km/h")
                else:
                    analysis.append(f"➖ 侧前车速度相当: {side_relative_speed}km/h")

        penalty_score = max(0, penalty_score)
        suitability_score = max(0, 100 - penalty_score)
        analysis.insert(0, f"适合度评分: {suitability_score:.1f}/100")
        return suitability_score, analysis

    def get_current_lane_penalty(self):
        """计算当前车道的惩罚分数"""
        vd = self.vehicle_data
        cfg = self.config

        penalty = 0
        analysis = []

        if vd['lead_relative_speed'] < -cfg['MIN_SPEED_ADVANTAGE']:
            speed_penalty = abs(vd['lead_relative_speed']) * cfg['PENALTY_WEIGHTS']['lead_relative_speed']
            penalty += speed_penalty
            analysis.append(f"当前前车较慢: {vd['lead_relative_speed']}km/h → +{speed_penalty:.1f}惩罚")

        time_gap = self.calculate_time_gap()
        if time_gap > 0 and time_gap < cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            distance_penalty = (cfg['FOLLOW_TIME_GAP_THRESHOLD'] - time_gap) * 10
            penalty += distance_penalty
            analysis.append(f"跟车时间较近: {time_gap:.1f}秒 → +{distance_penalty:.1f}惩罚")

        return penalty, analysis

    def get_available_overtake_directions(self):
        """获取可用的超车方向 - 最终修正版"""
        vd = self.vehicle_data
        cfg = self.config
        current_lane = cfg['current_lane_number']
        total_lanes = cfg['lane_count']

        available_directions = []
        debug_info = f"当前位置: 车道{current_lane}/{total_lanes}"

        # 基本方向可用性检查
        if current_lane > 1:  # 不是最左车道，可以向左变道
            available_directions.append("LEFT")
            debug_info += " | 可向左"

        if current_lane < total_lanes:  # 不是最右车道，可以向右变道
            # 检查是否为应急车道
            if not self.is_emergency_lane(current_lane + 1):
                available_directions.append("RIGHT")
                debug_info += " | 可向右"
            else:
                debug_info += " | 右侧为应急车道"

        # 高速公路优先级策略 - 修正逻辑
        if cfg['road_type'] == 'highway':
            debug_info += " | 高速公路"
            
            if current_lane == 1:
                # 最左车道：只能向右变道（如果有的话）
                debug_info += " | 最左车道"
                if "RIGHT" in available_directions and "LEFT" in available_directions:
                    available_directions.remove("LEFT")  # 最左车道不能向左
                    
            elif current_lane == total_lanes:
                # 最右车道：只能向左变道（如果有的话）
                debug_info += " | 最右车道" 
                if "RIGHT" in available_directions and "LEFT" in available_directions:
                    available_directions.remove("RIGHT")  # 最右车道不能向右
                    
            else:
                # 中间车道：向左变道（向快车道）优先
                if "LEFT" in available_directions and "RIGHT" in available_directions:
                    available_directions.remove("LEFT")
                    available_directions.remove("RIGHT")
                    available_directions.insert(0, "LEFT")   # 向左优先
                    available_directions.append("RIGHT")     # 向右次之
                    debug_info += " | 中间车道向左优先"

        debug_print(f"🛣️ {debug_info}")
        return available_directions

    def is_emergency_lane(self, lane_number):
        """判断是否为应急车道 - 增强检测"""
        cfg = self.config
        
        # 高速公路的最右车道通常是应急车道
        if cfg['road_type'] == 'highway' and lane_number == cfg['lane_count']:
            return True
            
        # 额外检查：如果车道宽度异常窄，可能是应急车道
        if lane_number == cfg['lane_count']:  # 最右车道
            right_lane_width = self.vehicle_data.get('r_lane_width', 3.2)
            if right_lane_width < 2.8:  # 应急车道通常较窄
                return True
                
        return False

    def _has_overtaken_original_lane_lead(self):
        """检查是否已经超过原车道的前车"""
        vd = self.vehicle_data
        cs = self.control_state
        
        if cs['net_lane_changes'] > 0:  # 之前向左变道，原车道在右侧
            if vd['r_front_blind']:
                cs['original_lane_clear'] = False
                return False
            else:
                if not cs['original_lane_clear']:
                    cs['original_lane_clear'] = True
                    debug_print("✅ 检测到已超过原车道（右侧）前车")
                return True
                
        else:  # 之前向右变道，原车道在左侧
            if vd['l_front_blind']:
                cs['original_lane_clear'] = False
                return False
            else:
                if not cs['original_lane_clear']:
                    cs['original_lane_clear'] = True
                    debug_print("✅ 检测到已超过原车道（左侧）前车")
                return True

    def _is_return_effective(self, check_side, return_direction):
        """检查返回是否有效"""
        vd = self.vehicle_data

        if check_side == "left":
            lead_speed = vd['left_lead_speed']
            lead_distance = vd['left_lead_distance']
            relative_speed = vd['left_lead_relative_speed']
        else:
            lead_speed = vd['right_lead_speed']
            lead_distance = vd['right_lead_distance']
            relative_speed = vd['right_lead_relative_speed']

        if lead_distance <= 0:
            return True

        current_speed = vd['v_ego_kph']
        if lead_speed > current_speed + 5:
            return True

        if (relative_speed < -10 and lead_distance < 30):
            return False

        return True

    def _is_return_direction_available(self, return_direction):
        """检查返回方向是否可用"""
        current_lane = self.config['current_lane_number']
        total_lanes = self.config['lane_count']

        if return_direction == "RIGHT":
            return current_lane < total_lanes
        else:
            return current_lane > 1

    def check_smart_return_conditions(self):
        """检查智能返回条件 - 基于超车完成检测"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config

        if self.check_return_timeout():
            return False

        if cs['net_lane_changes'] == 0:
            return False

        if not cs['is_auto_overtake']:
            return False

        if cs['return_attempts'] >= cs['max_return_attempts']:
            debug_print(f"⚠️ 达到最大返回尝试次数({cs['max_return_attempts']})，放弃返回")
            self.reset_net_lane_changes()
            return False

        if cs['isOvertaking']:
            return False

        if cs['net_lane_changes'] > 0:
            return_direction = "RIGHT"
            check_side = "right"
        else:
            return_direction = "LEFT"
            check_side = "left"

        if not self._is_return_direction_available(return_direction):
            debug_print(f"❌ 返回方向{return_direction}不可用")
            return False

        # 🔥 关键修正：检查是否完成了超车
        if not self._has_overtaken_original_lane_lead():
            cs['overtakeState'] = "正在超越前车"
            cs['overtakeReason'] = "尚未完全超过原车道前车，暂不返回"
            return False

        is_safe, safety_reason = self.check_lane_safety(check_side)
        if not is_safe:
            cs['overtakeState'] = f"返回{return_direction}不安全"
            cs['overtakeReason'] = f"安全条件: {safety_reason}"
            return False

        if not self._is_return_effective(check_side, return_direction):
            cs['overtakeState'] = f"目标车道有慢车"
            cs['overtakeReason'] = f"返回{return_direction}车道效率低"
            return False

        if cs['return_timer_start'] == 0:
            cs['return_timer_start'] = time.time() * 1000
            delay = 3000
            debug_print(f"⏰ 开始返回计时: {delay/1000}秒 (方向: {return_direction})")
            return False

        current_time = time.time() * 1000
        delay = 3000

        if current_time - cs['return_timer_start'] >= delay:
            cs['return_conditions_met'] = True
            return True

        return False

    def perform_smart_return(self):
        """执行智能返回"""
        cs = self.control_state

        if not cs['return_conditions_met']:
            return

        if cs['net_lane_changes'] > 0:
            return_direction = "RIGHT"
        else:
            return_direction = "LEFT"

        current_count = cs['overtakeSuccessCount']
        success = self.send_command("LANECHANGE", return_direction)

        if success:
            cs['lane_change_in_progress'] = True
            cs['return_conditions_met'] = False
            cs['return_attempts'] += 1
            cs['lastLaneChangeCommandTime'] = time.time() * 1000
            cs['return_start_count'] = current_count
            cs['last_return_direction'] = return_direction

            direction_text = "右" if return_direction == "RIGHT" else "左"
            attempt_text = f"第{cs['return_attempts']}次"
            cs['overtakeState'] = f"{attempt_text}{direction_text}返回"
            cs['overtakeReason'] = f"净变道{cs['net_lane_changes']}次，尝试返回"

            debug_print(f"🔄 {attempt_text}返回: {direction_text}变道")

    def check_return_completion(self):
        """检查返回是否完成"""
        cs = self.control_state

        if not cs.get('lane_change_in_progress') or cs.get('return_start_count') is None:
            return

        current_count = cs['overtakeSuccessCount']
        start_count = cs['return_start_count']

        if current_count > start_count:
            cs['lane_change_in_progress'] = False

            if cs['last_return_direction'] == "RIGHT":
                cs['net_lane_changes'] -= 1
                cs['last_auto_overtake_time'] = time.time() * 1000
            else:
                cs['net_lane_changes'] += 1
                cs['last_auto_overtake_time'] = time.time() * 1000

            cs['return_timer_start'] = 0
            cs['original_lane_clear'] = False

            del cs['return_start_count']

            direction_text = "右" if cs['last_return_direction'] == "RIGHT" else "左"
            current_net = cs['net_lane_changes']

            cs['overtakeState'] = f"{direction_text}返回完成"
            cs['overtakeReason'] = f"净变道次数: {current_net}"

            debug_print(f"✅ 返回完成: {direction_text}变道 | 净变道: {current_net}")

            if current_net == 0 or cs['return_attempts'] >= cs['max_return_attempts']:
                self.reset_net_lane_changes()
                debug_print("🎯 返回流程完成或达到最大尝试次数")

    def _execute_overtake_decision(self):
        """执行超车决策"""
        available_directions = self.get_available_overtake_directions()

        if not available_directions:
            self.control_state['overtakeState'] = "无可用变道方向"
            self.control_state['overtakeReason'] = "当前车道位置限制"
            return

        # 🔥 关键修正：开始新超车时重置返回状态
        self.control_state['return_timer_start'] = 0
        self.control_state['return_conditions_met'] = False
        self.control_state['original_lane_clear'] = False

        current_penalty, current_analysis = self.get_current_lane_penalty()

        direction_scores = {}
        direction_analysis = {}
        direction_effectiveness = {}

        for direction in available_directions:
            side = "left" if direction == "LEFT" else "right"

            safety_score, safety_analysis = self.evaluate_lane_suitability(side)
            is_effective, effectiveness_score, effectiveness_reasons = self.is_overtake_effective(direction)

            effectiveness_factor = effectiveness_score / 100.0
            combined_score = safety_score * effectiveness_factor

            direction_scores[direction] = combined_score
            direction_effectiveness[direction] = {
                'score': effectiveness_score,
                'is_effective': is_effective,
                'reasons': effectiveness_reasons
            }

            full_analysis = safety_analysis.copy()
            if effectiveness_reasons:
                full_analysis.extend([f"⚠️ {reason}" for reason in effectiveness_reasons])
            if is_effective:
                full_analysis.append(f"✅ 超车有效性: {effectiveness_score}%")
            else:
                full_analysis.append(f"❌ 超车无效: {effectiveness_score}%")

            direction_analysis[direction] = full_analysis

        best_direction = None
        best_score = 0
        detailed_reason = ""

        for direction in available_directions:
            score = direction_scores[direction]
            effectiveness_info = direction_effectiveness[direction]

            if not effectiveness_info['is_effective']:
                continue

            if self.config['road_type'] == 'highway':
                current_lane = self.config['current_lane_number']

                if current_lane == self.config['lane_count'] and direction == "LEFT":
                    score += 20
                    direction_analysis[direction].append("🔄 最右车道优先向左")

                elif current_lane == 1 and direction == "RIGHT":
                    score -= 15
                    direction_analysis[direction].append("⚠️ 快车道向右需谨慎")

            if score > self.config['PENALTY_THRESHOLD'] and score > best_score:
                best_direction = direction
                best_score = score

                effectiveness_text = f"有效性{effectiveness_info['score']}%"
                safety_text = f"安全性{score:.1f}%"
                analysis_text = " | ".join(direction_analysis[direction])
                detailed_reason = f"{direction}车道 {effectiveness_text} | {safety_text} | {analysis_text}"

        if best_direction and best_score > self.config['PENALTY_THRESHOLD']:
            target_advantage = best_score - (100 - current_penalty)

            min_advantage = 5
            if self.config['road_type'] == 'highway':
                min_advantage = 3

            if target_advantage >= min_advantage:
                self.execute_overtake(best_direction)
                self.control_state['overtakeReason'] = detailed_reason
                debug_print(f"🎯 智能车道选择: {best_direction}变道 | 综合评分: {best_score:.1f}%")
            else:
                self.control_state['overtakeState'] = "目标车道优势不足"
                self.control_state['overtakeReason'] = f"目标车道优势不足: +{target_advantage:.1f}% | 需要至少+{min_advantage}%"
        else:
            no_overtake_reasons = []

            for direction in available_directions:
                score = direction_scores[direction]
                effectiveness_info = direction_effectiveness[direction]

                if not effectiveness_info['is_effective']:
                    reason = f"{direction}:无效超车({effectiveness_info['score']}%)"
                    if effectiveness_info['reasons']:
                        reason += f"[{','.join(effectiveness_info['reasons'])}]"
                elif score <= self.config['PENALTY_THRESHOLD']:
                    reason = f"{direction}:安全性不足({score:.1f}%)"
                else:
                    reason = f"{direction}:条件满足({score:.1f}%)"

                no_overtake_reasons.append(reason)

            if not available_directions:
                no_overtake_reasons.append("无可用变道方向")

            self.control_state['overtakeState'] = "分析车道中"
            self.control_state['overtakeReason'] = f"超车条件分析: {', '.join(no_overtake_reasons)}"

    def perform_auto_overtake(self):
        """执行自动超车 - 优化返回优先级"""
        if not self.config['autoOvertakeEnabled'] or self.control_state['isOvertaking']:
            return

        if self.vehicle_data['system_auto_control'] == 1:
            self.control_state['overtakeState'] = "OP控制中"
            self.control_state['overtakeReason'] = "OP自动控制中，暂停超车"
            return

        if self.check_op_control_cooldown():
            return

        # 🔥 关键修正：超车条件检查优先于返回条件检查
        if self.check_overtake_conditions():
            self._execute_overtake_decision()
            return

        # 🔥 关键修正：只有在没有超车机会时才检查返回
        if (self.control_state['net_lane_changes'] != 0 and 
            self.control_state['is_auto_overtake']):
            
            return_ready = self.check_smart_return_conditions()
            if return_ready:
                self.perform_smart_return()
            else:
                self._handle_return_fallback()
            
            self.check_return_completion()

    def _handle_return_fallback(self):
        """处理返回失败的情况"""
        cs = self.control_state
        
        current_time = time.time() * 1000
        if (cs['return_timer_start'] > 0 and 
            current_time - cs['return_timer_start'] > 20000):
            
            debug_print("⏰ 返回条件长时间不满足，放弃本次返回")
            cs['return_timer_start'] = 0
            cs['return_attempts'] += 1
            
            if cs['return_attempts'] >= cs['max_return_attempts']:
                debug_print(f"⚠️ 达到最大返回尝试次数({cs['max_return_attempts']})，放弃返回")
                self.reset_net_lane_changes()

    def execute_overtake(self, direction):
        """执行超车操作"""
        current_success_count = self.control_state['overtakeSuccessCount']

        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['isOvertaking'] = True
            self.control_state['lane_change_in_progress'] = True
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            self.control_state['return_timer_start'] = 0
            self.control_state['return_conditions_met'] = False
            self.control_state['original_lane_clear'] = False

            self.update_net_lane_changes(direction, is_auto_overtake=True)

            self.control_state['follow_start_time'] = None
            self.control_state['is_following_slow_vehicle'] = False
            self.control_state['max_follow_time_reached'] = False

            self.control_state['overtake_start_count'] = current_success_count

            if direction == "LEFT":
                self.control_state['overtakeState'] = "← 准备向左变道超车"
                self.control_state['current_status'] = "自动左变道"
            else:
                self.control_state['overtakeState'] = "→ 准备向右变道超车"
                self.control_state['current_status'] = "自动右变道"

            debug_print(f"🚀 开始超车: {direction}变道 | 净变道: {self.control_state['net_lane_changes']}")

    def check_overtake_completion(self):
        """检查超车完成状态"""
        if not self.control_state['lane_change_in_progress']:
            return

        current_count = self.control_state['overtakeSuccessCount']
        start_count = self.control_state.get('overtake_start_count', current_count)

        if current_count > start_count:
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = True

            self.control_state['original_lane_clear'] = False

            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'success'

            direction = self.control_state['lastOvertakeDirection']
            direction_text = "左" if direction == "LEFT" else "右"
            net_changes = self.control_state['net_lane_changes']

            self.control_state['overtakeState'] = f"{direction_text}变道完成，等待超越前车"
            self.control_state['overtakeReason'] = f"变道完成，检测超越原车道前车中..."
            self.control_state['current_status'] = "变道完成"

            debug_print(f"✅ 变道完成: {direction_text}变道成功 | 等待超越前车 | 净变道: {net_changes}")

            if 'overtake_start_count' in self.control_state:
                del self.control_state['overtake_start_count']

        elif time.time() * 1000 - self.control_state['lastLaneChangeCommandTime'] > 15000:
            self.control_state['lane_change_in_progress'] = False
            self.control_state['isOvertaking'] = False

            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'failed'

            self.control_state['overtakeState'] = "变道超时"
            self.control_state['overtakeReason'] = "15秒内未检测到变道完成，快速重试"
            debug_print("❌ 变道超时，未检测到完成信号")

    def get_no_overtake_reasons(self):
        """获取未超车的具体原因"""
        vd = self.vehicle_data
        cfg = self.config
        cs = self.control_state

        reasons = []

        if vd['system_auto_control'] == 1:
            reasons.append("OP自动控制中")
            return reasons

        if self.check_op_control_cooldown():
            reasons.append("OP控制后冷却中")
            return reasons

        if not vd['IsOnroad']:
            reasons.append("车辆不在道路上")
            return reasons

        if not vd['engaged']:
            reasons.append("巡航未激活")
            return reasons

        if vd['lead_distance'] <= 0:
            reasons.append("前方无车辆")
            return reasons

        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        time_gap = self.calculate_time_gap()

        trigger_conditions_met = []

        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            trigger_conditions_met.append("前车相对速度")

        if 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            trigger_conditions_met.append("跟车时间距离")

        if speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            trigger_conditions_met.append("速度比例")

        if not trigger_conditions_met:
            reasons.append("未满足任何超车触发条件")
            reasons.append(f"相对速度:{vd['lead_relative_speed']}km/h(阈值:{cfg['LEAD_RELATIVE_SPEED_THRESHOLD']}km/h)")
            reasons.append(f"时间距离:{time_gap:.1f}秒(阈值:{cfg['FOLLOW_TIME_GAP_THRESHOLD']}秒)")
            reasons.append(f"速度比例:{speed_ratio*100:.0f}%(阈值:{cfg['CRUISE_SPEED_RATIO_THRESHOLD']*100:.0f}%)")
            return reasons

        if cfg['road_type'] == 'highway' and vd['v_ego_kph'] < cfg['HIGHWAY_MIN_SPEED']:
            reasons.append(f"高速车速{vd['v_ego_kph']}km/h过低(阈值:{cfg['HIGHWAY_MIN_SPEED']}km/h)")

        if cfg['road_type'] == 'normal' and vd['v_ego_kph'] < cfg['NORMAL_ROAD_MIN_SPEED']:
            reasons.append(f"普通路车速{vd['v_ego_kph']}km/h过低(阈值:{cfg['NORMAL_ROAD_MIN_SPEED']}km/h)")

        now = time.time() * 1000
        if cs['lastOvertakeTime'] > 0 and now - cs['lastOvertakeTime'] < cs['dynamic_cooldown']:
            remaining = (cs['dynamic_cooldown'] - (now - cs['lastOvertakeTime'])) / 1000
            reasons.append(f"冷却时间剩余{remaining:.1f}秒")

        available_directions = self.get_available_overtake_directions()
        if not available_directions:
            reasons.append("无可用变道方向")
        else:
            for direction in available_directions:
                side = "left" if direction == "LEFT" else "right"
                safety_score, safety_analysis = self.evaluate_lane_suitability(side)
                is_effective, effectiveness_score, effectiveness_reasons = self.is_overtake_effective(direction)

                if not is_effective:
                    reasons.append(f"{direction}车道无效超车")
                elif safety_score < cfg['PENALTY_THRESHOLD']:
                    reasons.append(f"{direction}车道不安全")

        if trigger_conditions_met and reasons:
            reasons.insert(0, f"触发条件: {', '.join(trigger_conditions_met)}")

        return reasons

    def update_net_lane_changes(self, direction, is_auto_overtake=True):
        """更新净变道次数"""
        cs = self.control_state
        
        if is_auto_overtake:
            if direction == "LEFT":
                cs['net_lane_changes'] += 1
                cs['lastOvertakeDirection'] = "LEFT"
                cs['last_auto_overtake_time'] = time.time() * 1000
                cs['is_auto_overtake'] = True
                debug_print(f"🔄 自动超车净变道次数更新: {cs['net_lane_changes']} (方向: {direction})")
            elif direction == "RIGHT":
                cs['net_lane_changes'] -= 1
                cs['lastOvertakeDirection'] = "RIGHT"
                cs['last_auto_overtake_time'] = time.time() * 1000
                cs['is_auto_overtake'] = True
                debug_print(f"🔄 自动超车净变道次数更新: {cs['net_lane_changes']} (方向: {direction})")
        else:
            self.reset_net_lane_changes()
            debug_print(f"🔄 手动变道，清零净变道次数")

    def reset_net_lane_changes(self):
        """重置净变道次数"""
        cs = self.control_state
        cs['net_lane_changes'] = 0
        cs['return_attempts'] = 0
        cs['return_conditions_met'] = False
        cs['return_timer_start'] = 0
        cs['last_auto_overtake_time'] = 0
        cs['is_auto_overtake'] = False
        cs['original_lane_clear'] = False
        debug_print("🔄 净变道次数已重置")

    def check_return_timeout(self):
        """检查返回超时"""
        cs = self.control_state
        current_time = time.time() * 1000
        
        if cs['net_lane_changes'] != 0 and cs['last_auto_overtake_time'] > 0:
            time_since_last_auto = current_time - cs['last_auto_overtake_time']
            if time_since_last_auto > cs['return_timeout']:
                debug_print(f"⏰ 返回超时({time_since_last_auto/1000:.1f}秒)，清零净变道次数")
                self.reset_net_lane_changes()
                return True
        return False

    def send_command(self, cmd_type, arg):
        """发送控制命令"""
        self.cmd_index += 1
        command = {
            "index": self.cmd_index,
            "cmd": cmd_type,
            "arg": arg,
            "timestamp": int(time.time() * 1000)
        }

        try:
            message = json.dumps(command).encode('utf-8')
            self.udp_socket.sendto(message, (self.remote_ip, self.remote_port))
            self.control_state['last_command'] = f"{cmd_type}: {arg}"
            self.last_command_time = time.time()
            print(f"📤 发送指令: {command}")
            return True
        except Exception as e:
            print(f"❌ 发送指令错误: {e}")
            return False

    def manual_overtake(self, lane):
        """手动变道"""
        current_success_count = self.control_state['overtakeSuccessCount']

        direction = "LEFT" if lane == "left" else "RIGHT"
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000
            self.control_state['manual_start_count'] = current_success_count

            self.update_net_lane_changes(direction, is_auto_overtake=False)

            if lane == "left":
                self.control_state['current_status'] = "强制左变道"
                self.control_state['overtakeState'] = "← 手动左变道"
            else:
                self.control_state['current_status'] = "强制右变道"
                self.control_state['overtakeState'] = "→ 手动右变道"
            self.control_state['overtakeReason'] = "用户强制变道指令（忽略系统自动控制）"
            print(f"🔧 手动变道指令: {direction} | 净变道已清零")

    def check_manual_lane_change_completion(self):
        """检查手动变道是否完成"""
        cs = self.control_state

        if cs.get('manual_start_count') is not None:
            current_count = cs['overtakeSuccessCount']
            start_count = cs['manual_start_count']

            if current_count > start_count:
                direction = cs['lastOvertakeDirection']
                direction_text = "左" if direction == "LEFT" else "右"
                cs['current_status'] = f"手动{direction_text}变道完成"
                cs['overtakeState'] = f"手动{direction_text}变道完成"
                cs['overtakeReason'] = "手动变道完成"
                print(f"✅ 手动变道完成: {direction_text}变道 | 净变道已清零")

                del cs['manual_start_count']

    def cancel_overtake(self):
        """取消超车"""
        success = self.send_command("CANCEL_OVERTAKE", "true")
        if success:
            self.control_state['current_status'] = "取消超车"
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = False

    def change_speed(self, direction):
        """改变速度"""
        if direction == "UP":
            self.send_command("SPEED", direction)
        elif direction == "DOWN":
            self.send_command("SPEED", direction)

    def update_curve_detection(self):
        """更新弯道检测"""
        vd = self.vehicle_data
        cfg = self.config

        is_curve = (vd['max_curve'] >= 1.0 or
                   abs(vd['road_curvature']) > cfg['CURVATURE_THRESHOLD'] or
                   abs(vd['steering_angle']) > cfg['STEERING_THRESHOLD'])

        if is_curve and self.control_state['isOvertaking']:
            self.cancel_overtake()
            self.control_state['current_status'] = "弯道中取消超车"
            self.control_state['overtakeReason'] = "检测到弯道，安全第一"

    def run_data_loop(self):
        """数据循环"""
        ratekeeper = Ratekeeper(10)

        while self.running:
            try:
                self.update_vehicle_data()
                self.update_lane_number()
                self.update_curve_detection()
                self.update_following_status()

                current_time = time.time() * 1000
                if (hasattr(self, 'last_lane_count_calc') and
                    current_time - self.last_lane_count_calc > 5000):
                    self.calculate_lane_count()
                    self.last_lane_count_calc = current_time
                elif not hasattr(self, 'last_lane_count_calc'):
                    self.calculate_lane_count()
                    self.last_lane_count_calc = current_time

                self.check_return_timeout()

                if self.config['autoOvertakeEnabled']:
                    self.perform_auto_overtake()
                    self.check_overtake_completion()

                    if (self.control_state['net_lane_changes'] != 0 and 
                        self.control_state['is_auto_overtake']):
                        self.check_return_completion()

                self.check_manual_lane_change_completion()

                ratekeeper.keep_time()
            except Exception as e:
                print(f"数据循环错误: {e}")
                time.sleep(0.1)

    def get_status_data(self):
        """获取状态数据"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config

        time_gap = self.calculate_time_gap()
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0

        remaining_cooldown = 0
        now = time.time() * 1000
        if cs['lastOvertakeTime'] > 0:
            elapsed = now - cs['lastOvertakeTime']
            remaining_cooldown = max(0, cs['dynamic_cooldown'] - elapsed) / 1000

        remaining_return_timeout = 0
        if cs['net_lane_changes'] != 0 and cs['last_auto_overtake_time'] > 0:
            elapsed_auto = now - cs['last_auto_overtake_time']
            remaining_return_timeout = max(0, cs['return_timeout'] - elapsed_auto) / 1000

        remaining_op_cooldown = 0
        if cs['op_control_cooldown'] > 0:
            elapsed_op = now - cs['last_op_control_end_time']
            remaining_op_cooldown = max(0, cs['op_control_cooldown'] - elapsed_op) / 1000

        trigger_conditions = self.get_trigger_conditions()
        no_overtake_reasons = self.get_no_overtake_reasons()

        left_lane_narrow = vd.get('l_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)
        right_lane_narrow = vd.get('r_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)

        left_warnings = []
        right_warnings = []

        if left_lane_narrow:
            left_warnings.append("车道过窄⚠️禁止变道")
        if vd.get('left_blindspot', False) or vd.get('l_front_blind', False):
            left_warnings.append("盲区有车⚠️禁止变道")
        if vd.get('left_lead_distance', 0) > 0 and vd.get('left_lead_distance', 0) < cfg.get('SIDE_LEAD_DISTANCE_MIN', 15):
            left_warnings.append("侧车过近⚠️禁止变道")
        if abs(vd.get('left_lead_relative_speed', 0)) > cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20):
            left_warnings.append("侧车相对⚠️速度过高")

        if right_lane_narrow:
            right_warnings.append("车道过窄⚠️禁止变道")
        if vd.get('right_blindspot', False) or vd.get('r_front_blind', False):
            right_warnings.append("盲区有车⚠️禁止变道")
        if vd.get('right_lead_distance', 0) > 0 and vd.get('right_lead_distance', 0) < cfg.get('SIDE_LEAD_DISTANCE_MIN', 15):
            right_warnings.append("侧车过近⚠️禁止变道")
        if abs(vd.get('right_lead_relative_speed', 0)) > cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20):
            right_warnings.append("侧车相对⚠️速度过高")

        return {
            # 🌐 系统状态
            'w': True,  # 系统运行中
            'ip': self.get_local_ip(),  # 本地IP地址
            
            # 🚗 速度信息
            's': vd.get('v_ego_kph', 0),      # 本车速度
            'c': vd.get('v_cruise_kph', 0),   # 巡航速度
            'd': vd.get('desire_speed', 0),   # 期望速度
            
            # 🚘 前车信息
            'ls': vd.get('lead_speed', 0),            # 前车速度
            'ld': vd.get('lead_distance', 0),         # 前车距离
            'lrs': vd.get('lead_relative_speed', 0),  # 前车相对速度
            
            # 👁️ 盲区状态
            'lb': bool(vd.get('left_blindspot', False)),
            'rb': bool(vd.get('right_blindspot', False)),
            'l_front_blind': bool(vd.get('l_front_blind', False)),
            'r_front_blind': bool(vd.get('r_front_blind', False)),
            
            # 🛣️ 车道几何信息
            'llw': float(vd.get('l_lane_width', 3.2)),    # 左侧车道宽度
            'rlw': float(vd.get('r_lane_width', 3.2)),    # 右侧车道宽度
            'led': float(vd.get('l_edge_dist', 1.5)),     # 左侧边缘距离
            'red': float(vd.get('r_edge_dist', 1.5)),     # 右侧边缘距离
            
            # 🚘 侧方车辆信息
            'lls': vd.get('left_lead_speed', 0),          # 左侧前车速度
            'lld': vd.get('left_lead_distance', 0),       # 左侧前车距离
            'llrs': vd.get('left_lead_relative_speed', 0),# 左侧前车相对速度
            'rls': vd.get('right_lead_speed', 0),         # 右侧前车速度
            'rld': vd.get('right_lead_distance', 0),      # 右侧前车距离
            'rlrs': vd.get('right_lead_relative_speed', 0),# 右侧前车相对速度
            
            # ⚙️ 配置信息
            'rt': cfg.get('road_type', 'highway'),        # 道路类型
            'lc': cfg.get('lane_count', 3),               # 车道总数
            'cl': cfg.get('current_lane_number', 2),      # 当前车道编号
            'lane_count_mode': cfg.get('lane_count_mode', 'auto'), # 车道计数模式
            
            # 🚀 超车状态
            'os': cs.get('overtakeState', '等待超车条件'), # 超车状态
            'or': cs.get('overtakeReason', '分析道路情况中...'), # 超车原因
            'oc': cs.get('overtakeSuccessCount', 0),      # 超车成功次数
            
            # 🎛️ 超车参数
            'hms': cfg.get('HIGHWAY_MIN_SPEED', 75),      # 高速最低速度
            'nms': cfg.get('NORMAL_ROAD_MIN_SPEED', 40),  # 普通路最低速度
            'sr': cfg.get('CRUISE_SPEED_RATIO_THRESHOLD', 0.8), # 速度比例阈值
            'ftg': cfg.get('FOLLOW_TIME_GAP_THRESHOLD', 3.0), # 跟车时间阈值
            'mft': cfg.get('MAX_FOLLOW_TIME', 120000),    # 最大跟车时间
            'mlw': cfg.get('MIN_LANE_WIDTH', 2.5),        # 最小车道宽度
            'slw': cfg.get('SAFE_LANE_WIDTH', 3.0),       # 安全车道宽度
            'sld': cfg.get('SIDE_LEAD_DISTANCE_MIN', 15), # 侧前车安全距离
            'srs': cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20), # 侧车相对速度阈值
            'lrs_threshold': cfg.get('LEAD_RELATIVE_SPEED_THRESHOLD', -5.0), # 前车相对速度阈值
            
            # 🔧 功能开关
            'aoe': cfg.get('autoOvertakeEnabled', True),  # 自动超车启用
            'srtl': cfg.get('shouldReturnToLane', True),  # 返回原车道启用
            
            # ⚠️ 警告状态
            'left_lane_narrow': left_lane_narrow,         # 左侧车道过窄
            'right_lane_narrow': right_lane_narrow,       # 右侧车道过窄
            
            # 🎮 系统控制状态
            'system_auto_control': vd.get('system_auto_control', 0), # OP自动控制状态
            
            # 🔄 智能返回系统
            'net_lane_changes': cs.get('net_lane_changes', 0),       # 净变道次数
            'return_attempts': cs.get('return_attempts', 0),         # 返回尝试次数
            'original_lane_clear': cs.get('original_lane_clear', False), # 🔥 原车道是否已超越
            
            # ❄️ 冷却系统
            'remaining_cooldown': remaining_cooldown,               # 剩余冷却时间
            'dynamic_cooldown': cs.get('dynamic_cooldown', 8000),   # 动态冷却时间
            'last_overtake_result': cs.get('last_overtake_result', 'none'), # 最后超车结果
            'consecutive_failures': cs.get('consecutive_failures', 0), # 连续失败次数
            
            # 📊 实时指标
            'time_gap': time_gap,                          # 时间距离
            'speed_ratio': speed_ratio,                    # 速度比例
            'sr_threshold': cfg.get('CRUISE_SPEED_RATIO_THRESHOLD', 0.8), # 速度比例阈值
            
            # 📋 条件分析
            'trigger_conditions': trigger_conditions,      # 触发条件
            'no_overtake_reasons': no_overtake_reasons,    # 未超车原因
            
            # ⏰ 超时信息
            'remaining_return_timeout': remaining_return_timeout, # 返回超时剩余
            'remaining_op_cooldown': remaining_op_cooldown,       # OP冷却剩余
            
            # 🚨 警告信息
            'left_warnings': left_warnings,                # 左侧警告列表
            'right_warnings': right_warnings,              # 右侧警告列表
            
            # 🔥 新增状态
            'is_auto_overtake': cs.get('is_auto_overtake', False) # 是否为自动超车
        }

    def get_local_ip(self):
        """获取本地IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"

    def start_web_server(self):
        """启动Web服务器"""
        from http.server import HTTPServer
        handler = self.create_web_handler()
        self.web_server = HTTPServer(('0.0.0.0', 8088), handler)
        print("🌐 Web服务器启动在端口 8088")
        self.web_server.serve_forever()

    def create_web_handler(self):
        """创建Web处理器"""
        controller = self

        class OvertakeHTTPHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                """处理GET请求"""
                if self.path == '/':
                    self.send_html_response()
                elif self.path == '/status':
                    self.send_json_status()
                else:
                    print(f"page {self.path} not found!")

            def do_POST(self):
                """处理POST请求"""
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(post_data) if post_data else {}

                    if self.path == '/control':
                        self.handle_control(data)
                    elif self.path == '/overtake':
                        self.handle_overtake(data)
                    elif self.path == '/config':
                        self.handle_config(data)
                    elif self.path == '/params':
                        self.handle_params(data)
                    else:
                        self.send_error(404, "接口未找到")
                except Exception as e:
                    print(f"请求处理错误: {e}")
                    self.send_error(400, "请求解析错误")

            def handle_control(self, data):
                """处理控制命令"""
                cmd_type = data.get('type', '')
                value = data.get('value', '')
                if cmd_type == 'SPEED':
                    controller.change_speed(value)
                self.send_json_response({'status': 'success', 'command': f'{cmd_type}: {value}'})

            def handle_overtake(self, data):
                """处理超车命令"""
                if 'manual' in data:
                    controller.manual_overtake(data['manual'])
                    self.send_json_response({'status': 'success', 'action': f'manual_{data["manual"]}'})
                elif 'cancel' in data:
                    controller.cancel_overtake()
                    self.send_json_response({'status': 'success', 'action': 'cancel'})
                elif 'auto' in data:
                    controller.config['autoOvertakeEnabled'] = bool(data['auto'])
                    controller.save_persistent_config()
                    self.send_json_response({'status': 'success', 'autoOvertake': controller.config['autoOvertakeEnabled']})
                elif 'return' in data:
                    controller.config['shouldReturnToLane'] = bool(data['return'])
                    controller.save_persistent_config()
                    self.send_json_response({'status': 'success', 'returnToLane': controller.config['shouldReturnToLane']})
                else:
                    self.send_json_response({'status': 'error', 'message': '未知操作'})

            def handle_config(self, data):
                """处理配置更新"""
                if 'lane_count_mode' in data:
                    mode = data['lane_count_mode']
                    if mode in ['manual', 'auto', 'op']:
                        controller.config['lane_count_mode'] = mode
                        controller.calculate_lane_count()
                        controller.save_persistent_config()

                if 'lanes' in data and controller.config['lane_count_mode'] == 'manual':
                    lanes = int(data['lanes'])
                    if 1 <= lanes <= 5:
                        controller.config['lane_count'] = lanes
                        controller.config['manual_lane_count'] = lanes
                        controller.save_persistent_config()

                if 'manual_lane_count' in data and controller.config['lane_count_mode'] == 'manual':
                    lanes = int(data['manual_lane_count'])
                    if 1 <= lanes <= 5:
                        controller.config['manual_lane_count'] = lanes
                        controller.config['lane_count'] = lanes
                        controller.save_persistent_config()

                if 'road_type' in data:
                    controller.config['road_type'] = data['road_type']
                    controller.calculate_lane_count()
                    controller.save_persistent_config()

                self.send_json_response({'status': 'success', 'config': controller.config})

            def handle_params(self, data):
                """处理参数更新"""
                param_map = {
                    'highwayMinSpeed': 'HIGHWAY_MIN_SPEED',
                    'normalMinSpeed': 'NORMAL_ROAD_MIN_SPEED',
                    'speedRatio': 'CRUISE_SPEED_RATIO_THRESHOLD',
                    'followTimeGap': 'FOLLOW_TIME_GAP_THRESHOLD',
                    'maxFollowTime': 'MAX_FOLLOW_TIME',
                    'minLaneWidth': 'MIN_LANE_WIDTH',
                    'safeLaneWidth': 'SAFE_LANE_WIDTH',
                    'sideLeadDist': 'SIDE_LEAD_DISTANCE_MIN',
                    'sideRelSpeed': 'SIDE_RELATIVE_SPEED_THRESHOLD',
                    'leadRelSpeed': 'LEAD_RELATIVE_SPEED_THRESHOLD'
                }

                for web_key, config_key in param_map.items():
                    if web_key in data:
                        if web_key == 'maxFollowTime':
                            # 将分钟转换为毫秒
                            controller.config[config_key] = int(data[web_key]) * 1
                        elif web_key == 'speedRatio':
                            # 将百分比转换为小数
                            controller.config[config_key] = float(data[web_key]) / 100.0
                        else:
                            controller.config[config_key] = float(data[web_key])

                controller.save_persistent_config()
                self.send_json_response({'status': 'success', 'message': '参数已保存'})

            def send_html_response(self):
                """发送HTML页面"""
                html = self.get_html_content()
                self.send_response(200)
                self.send_header('Content-type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))

            def send_json_status(self):
                """发送JSON状态数据"""
                status_data = controller.get_status_data()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(status_data, ensure_ascii=False).encode('utf-8'))

            def send_json_response(self, data):
                """发送JSON响应"""
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))

            def get_html_content(self):
                """获取HTML文件内容"""
                html_file_path = os.path.join(os.path.dirname(__file__), 'web_interface.html')
                try:
                    with open(html_file_path, 'r', encoding='utf-8') as f:
                        return f.read()
                except FileNotFoundError:
                    return "<html><body><h1>错误：未找到HTML界面文件</h1></body></html>"

            def log_message(self, format, *args):
                """禁用访问日志"""
                pass

        return OvertakeHTTPHandler

    def start(self):
        """启动控制器"""
        print("🚗 启动现代汽车自动超车控制器...")
        self.data_thread = threading.Thread(target=self.run_data_loop, daemon=True)
        self.data_thread.start()
        self.start_web_server()

    def stop(self):
        """停止控制器"""
        self.running = False
        if self.web_server:
            self.web_server.shutdown()
        if self.udp_socket:
            self.udp_socket.close()
        print("现代汽车自动超车控制器已停止")


def main():
    """主函数"""
    print("="*50)
    print("现代汽车自动超车控制器")
    print("访问地址: http://<op_ip>:8088")
    print("="*50)

    controller = AutoOvertakeController()
    try:
        controller.start()
    except KeyboardInterrupt:
        print("\n收到停止信号...")
    except Exception as e:
        print(f"运行错误: {e}")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
