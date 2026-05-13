import fcntl
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
import traceback
import numpy as np
import zmq
from datetime import datetime

from ftplib import FTP
from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ
from openpilot.selfdrive.carrot.carrot_speed import CarrotSpeed
from openpilot.selfdrive.carrot.config import UnifiedParams

from openpilot.common.gps import get_gps_location_service

try:
  from shapely.geometry import LineString
  SHAPELY_AVAILABLE = True
except ImportError:
  SHAPELY_AVAILABLE = False

NetworkType = log.DeviceState.NetworkType
DATA_PATH = '/data' if not PC else os.path.expanduser('~/.comma/carrot_data')
os.makedirs(DATA_PATH, exist_ok=True)

################ CarrotNavi
## 国家法令信息中心：道路设计标准
#V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 15, 5]

# Haversine 公式计算两个 GPS 坐标之间的距离
#haversine_cache = {}
def haversine(lon1, lat1, lon2, lat2):
    #key = (lon1, lat1, lon2, lat2)
    #if key in haversine_cache:
    #    return haversine_cache[key]

    R = 6371000  # 地球半径（米）
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    #haversine_cache[key] = distance
    return distance


# 获取两个坐标段上距离当前位置最近的点
def closest_point_on_segment(p1, p2, current_position):
    x1, y1 = p1
    x2, y2 = p2
    px, py = current_position

    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return p1  # p1 和 p2 是同一个点

    # 参数 t 是投影到线段上的比例因子
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))  # 将 t 限制在线段内

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return (closest_x, closest_y)


# 获取距离当前位置一定距离之后的路径
def get_path_after_distance(start_index, coordinates, current_position, distance_m):
    total_distance = 0
    path_after_distance = []
    closest_index = -1
    closest_point = None
    min_distance = float('inf')

    start_index = max(0, start_index - 2)

    # 修正为仅搜索邻近点
    for i in range(start_index, len(coordinates) - 1):
        p1 = coordinates[i]
        p2 = coordinates[i + 1]
        candidate_point = closest_point_on_segment(p1, p2, current_position)
        distance = haversine(current_position[0], current_position[1], candidate_point[0], candidate_point[1])

        if distance < min_distance:
            min_distance = distance
            closest_point = candidate_point
            closest_index = i
        elif distance > min_distance and min_distance < 10:
            break

    start_index = closest_index
    # 从最近点开始计算指定距离后的路径
    if closest_index != -1:
        path_after_distance.append(closest_point)

        path_after_distance.append(coordinates[closest_index + 1])
        total_distance = haversine(closest_point[0], closest_point[1], coordinates[closest_index + 1][0],
                                   coordinates[closest_index + 1][1])

        # 从下一个点开始向前遍历路径
        for i in range(closest_index + 1, len(coordinates) - 1):
            coord1 = coordinates[i]
            coord2 = coordinates[i + 1]
            segment_distance = haversine(coord1[0], coord1[1], coord2[0], coord2[1])

            if total_distance + segment_distance >= distance_m and segment_distance > 0:
                remaining_distance = distance_m - total_distance
                ratio = remaining_distance / segment_distance
                interpolated_lon = coord1[0] + ratio * (coord2[0] - coord1[0])
                interpolated_lat = coord1[1] + ratio * (coord2[1] - coord1[1])
                path_after_distance.append((interpolated_lon, interpolated_lat))
                break

            total_distance += segment_distance
            path_after_distance.append(coord2)

    return path_after_distance, start_index, closest_point


def calculate_angle(point1, point2):
    delta_lon = point2[0] - point1[0]
    delta_lat = point2[1] - point1[1]
    return math.degrees(math.atan2(delta_lat, delta_lon))

# 将 GPS 坐标转换为基于参考点和航向角的相对 x, y 坐标
def gps_to_relative_xy(gps_path, reference_point, heading_deg):
    ref_lon, ref_lat = reference_point
    relative_coordinates = []

    # 将航向角从度转换为弧度
    heading_rad = math.radians(heading_deg)

    for lon, lat in gps_path:
        # 将经纬度差转换为米（假设小距离下的简单近似）
        x = (lon - ref_lon) * 40008000 * math.cos(math.radians(ref_lat)) / 360
        y = (lat - ref_lat) * 40008000 / 360

        # 根据航向角旋转坐标以对齐车辆方向
        x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
        y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)

        relative_coordinates.append((y_rot, x_rot))

    return relative_coordinates


# 使用更快的矢量方法计算三个点之间的曲率
#curvature_cache = {}
def calculate_curvature(p1, p2, p3):
    #key = (p1, p2, p3)
    #if key in curvature_cache:
    #    return curvature_cache[key]

    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])

    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    if len_v1 * len_v2 == 0:
        curvature = 0
    else:
        curvature = cross_product / (len_v1 * len_v2 * len_v1)

    #curvature_cache[key] = curvature
    return curvature

class CarrotMan:
  def __init__(self):
    print("************************************************CarrotMan init************************************************")
    self.params = UnifiedParams()
    self.params_memory = Params("/dev/shm/params")
    self.gps_location_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster(['deviceState', 'carState', 'controlsState', 'radarState', 'longitudinalPlan', 'modelV2', 'selfdriveState', 'carControl', 'navRouteNavd', self.gps_location_service, 'navInstruction'])
    self.pm = messaging.PubMaster(['carrotMan', "navRoute", "navInstructionCarrot"])

    self.carrot_serv = CarrotServ()

    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 7705
    self.carrot_man_port = 7706
    self.connection = None

    self.ip_address = "0.0.0.0"
    self.remote_addr = None

    self.turn_speed_last = 250
    self.curvatureFilter = MyMovingAverage(20)
    self.vturn_speed_filter = MyMovingAverage(5)
    self.carrot_curve_speed_params()

    self.v_cruise_last = 0
    self.long_active = False
    self.v_cruise_change = 0
    self._last_vt = 0.0
    self.gas_pressed_count = 0
    self._last_viz_t = 0.0
    self.autoNaviSpeedDecelRate = 1.5
    self.activeCarrot = 0

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

    self.carrot_route_thread = threading.Thread(target=self.carrot_route, args=[])
    self.carrot_route_thread.daemon = True
    self.carrot_route_thread.start()

    self.is_running = True
    threading.Thread(target=self.broadcast_version_info).start()

    self.navi_points = []
    self.navi_points_start_index = 0
    self.navi_points_active = False
    self.navd_active = False

    self.active_carrot_last = False

    self.is_metric = self.params.get_bool("IsMetric")

  def get_broadcast_address(self):
    # 修改为支持 PC 的多接口检测
    if PC:
        interfaces = ['wlan0', 'eth0', 'enp0s3', 'br0']  # 常见 PC 接口
        for iface in interfaces:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    ip = fcntl.ioctl(
                        s.fileno(),
                        0x8919,  # SIOCGIFBRDADDR
                        struct.pack('256s', iface.encode('utf-8')[:15])
                    )[20:24]
                    return socket.inet_ntoa(ip)
            except Exception:
                continue
        return "255.255.255.255"  # 回退地址
    else:
      iface = b'wlan0'
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,
          struct.pack('256s', iface)
        )[20:24]
        return socket.inet_ntoa(ip)
    except (OSError, Exception):
      return None

  def get_local_ip(self):
      try:
          # 通过连接外部服务器确认本地 IP
          with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
              s.connect(("8.8.8.8", 80))  # 尝试连接 Google DNS
              return s.getsockname()[0]
      except Exception as e:
          return f"Error: {e}"

  # 发送广播消息
  def broadcast_version_info(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    self.save_toggle_values()

    carrot_speed = CarrotSpeed(neighbor_ring=2)
    self.params_memory.put_int_nonblocking("CarrotSpeed", 0)

    rk = Ratekeeper(20, print_delay_threshold=None)

    frame = 0
    self.save_toggle_values()

    carrotIndex_last = self.carrot_serv.carrotIndex
    phone_gps_frame = self.carrot_serv.phone_gps_frame
    carrot_speed_active_count = 0
    while self.is_running:
      try:
        self.sm.update(0)
        if self.sm.updated['navRouteNavd']:
          self.send_routes(self.sm['navRouteNavd'].coordinates, True)
        remote_addr = self.remote_addr
        remote_ip = remote_addr[0] if remote_addr is not None else ""
        vturn_speed, atc_curvature = self.carrot_curve_speed(self.sm)
        coords, distances, route_speed = self.carrot_navi_route()

        self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed, coords, distances, route_speed, self.gps_location_service, atc_curvature=atc_curvature)

        if phone_gps_frame != self.carrot_serv.phone_gps_frame:
          phone_gps_frame = self.carrot_serv.phone_gps_frame
          carrot_speed_active_count = 10
        else:
          carrot_speed_active_count -= 1

        if carrot_speed_active_count > 0:
          self.carrot_speed_serv(carrot_speed, frame)

        if frame % 20 == 0 or remote_addr is not None:
          try:
            self.broadcast_ip = self.get_broadcast_address() if remote_addr is None else remote_addr[0]
            if not PC:
              ip_address = socket.gethostbyname(socket.gethostname())
            else:
              ip_address = self.get_local_ip()
            if ip_address != self.ip_address:
              self.ip_address = ip_address
              self.remote_addr = None
            self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)

            msg = self.make_send_message()
            if self.broadcast_ip is not None:
              dat = msg.encode('utf-8')
              sock.sendto(dat, (self.broadcast_ip, self.broadcast_port))
            #for i in range(1, 255):
            #  ip_tuple = socket.inet_aton(self.broadcast_ip)
            #  new_ip = ip_tuple[:-1] + bytes([i])
            #  address = (socket.inet_ntoa(new_ip), self.broadcast_port)
            #  sock.sendto(dat, address)

            if remote_addr is None:
              #print(f"Broadcasting: {self.broadcast_ip}:{msg}")
              if not self.navd_active:
                #print("clear path_points: navd_active: ", self.navd_active)
                self.navi_points = []
                self.navi_points_active = False

          except OSError as e:
            if e.errno != 101:  # 屏蔽 PC 上的网络不可达垃圾信息
              if self.connection:
                self.connection.close()
              self.connection = None
              print(f"##### broadcast_error...: {e}")
              traceback.print_exc()
          except Exception as e:
            if self.connection:
              self.connection.close()
            self.connection = None
            print(f"##### broadcast_error...: {e}")
            traceback.print_exc()

        rk.keep_time()
        frame += 1
      except Exception as e:
        print(f"broadcast_version_info error...: {e}")
        traceback.print_exc()
        time.sleep(1)

  def carrot_speed_serv(self, carrot_speed, frame):
    v_ego = a_ego = 0.0
    gas_pressed = False
    if self.sm.alive['carState'] and self.sm.alive['carControl']:
      CS = self.sm['carState']
      CC = self.sm['carControl']
      v_ego = CS.vEgo
      a_ego = CS.aEgo
      gas_pressed = CS.gasPressed
      v_ego_kph = v_ego * 3.6
      if gas_pressed:
        self.gas_pressed_count = 200
        self.v_cruise_change = 0
      elif self._last_vt == CS.vCruise:
        self.v_cruise_last = CS.vCruise
      elif self.long_active and CC.longActive: # and self.gas_pressed_count == 0:
        if self.v_cruise_last < CS.vCruise:  # 当速度增加时
          if v_ego_kph < CS.vCruise:
            self.v_cruise_change = 100
          else:
            self.v_cruise_change = 0
        elif self.v_cruise_last > CS.vCruise: # 当速度减少时
          if v_ego_kph < CS.vCruise: # 如果行驶速度慢
            self.v_cruise_change = -100 #100
          else:                       # 如果行驶速度快
            self.v_cruise_change = -100

        if self.v_cruise_change != 0:
          self.gas_pressed_count = 0
      else:
        self.v_cruise_change = 0
      self.long_active = CC.longActive
      self.v_cruise_last = CS.vCruise
    else:
      self.v_cruise_change = 0
      return
    v_cruise_apply = max(min(CS.vCruise, v_ego_kph), 20)

    now = time.monotonic()
    heading = self.carrot_serv.bearing #nPosAnglePhone
    lat, lon = self.carrot_serv.vpPosPointLat, self.carrot_serv.vpPosPointLon #self.carrot_serv.estimate_position(self.carrot_serv.phone_latitude, self.carrot_serv.phone_longitude, heading, v_ego, now - self.carrot_serv.last_update_gps_time_phone)
    # 优化点：前瞻查询 (Lookahead Query)
    # 不仅看当前 (0.0m)，还看前方 30m 和 60m 的速度趋势，实现提前预判减速
    vt_now = carrot_speed.query_target_dist(lat, lon, heading, 0.0)
    vt_mid = carrot_speed.query_target_dist(lat, lon, heading, 30.0)
    vt_far = carrot_speed.query_target_dist(lat, lon, heading, 60.0)

    vt_candidates = [v for v in [vt_now, vt_mid, vt_far] if v > 5.0]
    vt_target = min(vt_candidates) if vt_candidates else vt_now

    # 增加变化率平滑 (Rate Limiting)，防止车速指令跳变
    if not hasattr(self, 'vt_filtered'): self.vt_filtered = vt_target
    self.vt_filtered = self.vt_filtered * 0.9 + vt_target * 0.1
    vt = self.vt_filtered

    if self.v_cruise_change != 0:
      carrot_speed.add_sample(lat, lon, heading, v_cruise_apply if self.v_cruise_change > 0 else (- v_cruise_apply))
      if self.v_cruise_change > 0:
        self.v_cruise_change -= 1
      if self.v_cruise_change < 0:
        self.v_cruise_change += 1
    else:
      if self.gas_pressed_count > 0:
        vt = max(vt, v_cruise_apply)
        carrot_speed.add_sample(lat, lon, heading, vt)

      self.params_memory.put_int_nonblocking("CarrotSpeed", int(vt))

    self._last_vt = vt
    if gas_pressed and a_ego < -0.5: #self._last_vt < 0.0:
      carrot_speed.invalidate_last_hit(window_s=2.0, action="clear")
    self.gas_pressed_count = max(0, self.gas_pressed_count - 1)

    if now - self._last_viz_t > 0.5: # 2Hz
        self._last_viz_t = now
        viz_json = carrot_speed.export_cells_around(lat, lon, heading, ring=2, max_points=64)
        # 最好写入内存 Params (而非磁盘)
        self.params_memory.put_nonblocking("CarrotSpeedViz", viz_json)

    carrot_speed.maybe_save()



  def carrot_navi_route(self):

    if self.carrot_serv.active_carrot > 1:
      if False and self.navd_active:  # mabox 始终激活
        self.navd_active = False
        self.params.remove("NavDestination")
    if not self.navi_points_active or not SHAPELY_AVAILABLE or (self.carrot_serv.active_carrot <= 1 and not self.navd_active):
      #print(f"navi_points_active: {self.navi_points_active}, active_carrot: {self.carrot_serv.active_carrot}")
      if self.navi_points_active:
        print("navi_points_active: ", self.navi_points_active, "active_carrot: ", self.carrot_serv.active_carrot, "navd_active: ", self.navd_active)
        #haversine_cache.clear()
        #curvature_cache.clear()
        self.navi_points = []
        self.navi_points_active = False
        if self.active_carrot_last > 1:
          #self.params.remove("NavDestination")
          pass
      self.active_carrot_last = self.carrot_serv.active_carrot
      return [],[],300

    current_position = (self.carrot_serv.vpPosPointLon, self.carrot_serv.vpPosPointLat)
    heading_deg = self.carrot_serv.bearing

    distance_interval = 10.0
    out_speed = 300
    path, self.navi_points_start_index, start_point = get_path_after_distance(self.navi_points_start_index, self.navi_points, current_position, 300)
    relative_coords = []
    if path:
        #relative_coords = gps_to_relative_xy(path, current_position, heading_deg)
        relative_coords = gps_to_relative_xy(path, start_point, heading_deg)
        # 使用 LineString 以 5m 为间隔重新采样相对坐标
        line = LineString(relative_coords)
        resampled_points = []
        resampled_distances = []
        current_distance = 0
        while current_distance <= line.length:
            point = line.interpolate(current_distance)
            resampled_points.append((point.x, point.y))
            resampled_distances.append(current_distance)
            current_distance += distance_interval

        curvatures = []
        distances = []
        distance = 10.0
        sample = 4
        if len(resampled_points) >= sample * 2 + 1:
            # 计算曲率并根据曲率计算速度
            speeds = []
            for i in range(len(resampled_points) - sample * 2):
                distance += distance_interval
                p1, p2, p3 = resampled_points[i], resampled_points[i + sample], resampled_points[i + sample * 2]
                curvature = calculate_curvature(p1, p2, p3)
                curvatures.append(curvature)
                speed = np.interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
                if abs(curvature) < 0.02:
                  speed = max(speed, self.carrot_serv.nRoadLimitSpeed)
                speeds.append(speed)
                distances.append(distance)
            #print(f"curvatures= {[round(s, 4) for s in curvatures]}")
            #print(f"speeds= {[round(s, 1) for s in speeds]}")
            # 反向应用加速度限制以调整速度
            accel_limit = self.carrot_serv.autoNaviSpeedDecelRate # m/s^2
            accel_limit_kmh = accel_limit * 3.6  # 转换为 km/h 每秒
            out_speeds = [0] * len(speeds)
            out_speeds[-1] = speeds[-1]  # 设置最后一个速度为初始值
            v_ego_kph = self.sm['carState'].vEgo * 3.6

            time_delay = self.carrot_serv.autoNaviSpeedCtrlEnd
            time_wait = 0
            for i in range(len(speeds) - 2, -1, -1):
                target_speed = speeds[i]
                next_out_speed = out_speeds[i + 1]

                if target_speed < next_out_speed:
                  time_delay = max(0, ((v_ego_kph - target_speed) / accel_limit_kmh))
                  time_wait = - time_delay

                # 根据速度计算当前路段的时间间隔
                time_interval = distance_interval / (next_out_speed / 3.6) if next_out_speed > 0 else 0

                time_apply = min(time_interval, max(0, time_interval + time_wait))

                # 计算加速度限制下的最大允许速度
                max_allowed_speed = next_out_speed + (accel_limit_kmh * time_apply)
                adjusted_speed = min(target_speed, max_allowed_speed)

                #time_wait += time_interval
                time_wait += min(2.0, time_interval)

                out_speeds[i] = adjusted_speed

            #distance_advance = self.sm['carState'].vEgo * 3.0  # 提前 3.0 秒预测距离
            #out_speed = interp(distance_advance, distances, out_speeds)
    # 优化点：目标速度平滑限制器 (Rate Limiter)
    # 防止在导航路径曲率剧变时，目标速度指令出现阶跃
    if not hasattr(self, 'out_speed_filtered'): self.out_speed_filtered = float(out_speed)
    # 限制每帧速度变化不超过 2.0 km/h (约 20km/h 每秒，考虑到 10Hz 更新)
    speed_diff = float(out_speed) - self.out_speed_filtered
    self.out_speed_filtered += np.clip(speed_diff, -2.0, 1.0) # 减速可以快一点，加速要平稳
    out_speed = self.out_speed_filtered

    return resampled_points, resampled_distances, out_speed #speeds, distances


  def make_send_message(self):
    msg = {}
    msg['Carrot2'] = self.params.get("Version").decode('utf-8')
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = self.navi_points_active
    msg['ip'] = self.ip_address
    msg['port'] = self.carrot_man_port
    self.controls_active = False
    self.xState = 0
    self.trafficState = 0
    v_ego_kph = 0
    log_carrot = ""
    v_cruise_kph = 0
    carcruiseSpeed = 0
    if not isOnroad:
      self.xState = 0
      self.trafficState = 0
    else:
      if self.sm.alive['carState']:
        carState = self.sm['carState']
        v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
        log_carrot = carState.logCarrot
        v_cruise_kph = carState.vCruise
        carcruiseSpeed = carState.cruiseState.speed * 3.6
      if self.sm.alive['selfdriveState']:
        selfdrive = self.sm['selfdriveState']
        self.controls_active = selfdrive.active
      if self.sm.alive['longitudinalPlan']:
        lp = self.sm['longitudinalPlan']
        self.xState = lp.xState
        self.trafficState = lp.trafficState

    msg['log_carrot'] = log_carrot
    msg['v_cruise_kph'] = v_cruise_kph
    msg['carcruiseSpeed'] = carcruiseSpeed
    msg['v_ego_kph'] = v_ego_kph
    msg['tbt_dist'] = self.carrot_serv.xDistToTurn
    msg['left_turn_sec'] = self.carrot_serv.carrot_left_sec
    msg['sdi_dist'] = self.carrot_serv.xSpdDist
    msg['active'] = self.controls_active
    msg['xState'] = self.xState
    msg['trafficState'] = self.trafficState
    return json.dumps(msg)

  def receive_fixed_length_data(self, sock, length):
    buffer = b""
    while len(buffer) < length:
      data = sock.recv(length - len(buffer))
      if not data:
        raise ConnectionError("在接收所有数据之前连接已关闭")
      buffer += data
    return buffer


  def carrot_man_thread(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)  # 设置套接字超时（10秒）
          sock.bind(('0.0.0.0', self.carrot_man_port))  # UDP 端口绑定
          print("#########carrot_man_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 接收数据 (UDP 使用 recvfrom)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 最大接收 4096 字节
                #print(f"Received data from {self.remote_addr}")

                if not data:
                  raise ConnectionError("未收到数据")

                if self.remote_addr is None:
                  print("已连接到: ", remote_addr)
                self.remote_addr = remote_addr
                try:
                  json_obj = json.loads(data.decode())
                  self.carrot_serv.update(json_obj)
                except Exception as e:
                  print(f"carrot_man_thread: json 错误...: {e}")
                  print(data)

                # 生成并发送响应消息 (UDP 使用 sendto)
                #try:
                #  msg = self.make_send_message()
                #  sock.sendto(msg.encode('utf-8'), self.remote_addr)
                #except Exception as e:
                #  print(f"carrot_man_thread: send 错误...: {e}")

              except TimeoutError:
                if not hasattr(self, 'carrot_timeout_cnt'): self.carrot_timeout_cnt = 0
                self.carrot_timeout_cnt += 1
                if self.carrot_timeout_cnt % 100 == 1:
                  print(f"[Carrot] 等待数据中 (超时次数: {self.carrot_timeout_cnt})...")
                self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"carrot_man_thread: 错误...: {e}")
                self.remote_addr = None
                break

            except Exception as e:
              print(f"carrot_man_thread: recv 错误...: {e}")
              self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        self.remote_addr = None
        print(f"网络错误，重试中...: {e}")
        time.sleep(2)


  def parse_kisa_data(self, data: bytes):
    result = {}

    try:
      decoded = data.decode('utf-8')
    except UnicodeDecodeError:
      print("解码错误:", data)
      return result

    parts = decoded.split('/')
    for part in parts:
      if ':' in part:
        key, value = part.split(':', 1)
        try:
          result[key] = int(value)
        except ValueError:
          result[key] = value
    return result

  def kisa_app_thread(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
          sock.settimeout(10)  # 设置套接字超时（10秒）
          sock.bind(('', 12345))  # UDP 端口绑定
          print("#########kisa_app_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 接收数据 (UDP 使用 recvfrom)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 最大接收 4096 字节
                #print(f"Received data from {self.remote_addr}")

                if not data:
                  raise ConnectionError("未收到数据")

                #if self.remote_addr is None:
                #  print("已连接到: ", remote_addr)
                #self.remote_addr = remote_addr
                try:
                  print(data)
                  kisa_data = self.parse_kisa_data(data)
                  self.carrot_serv.update_kisa(kisa_data)
                  #json_obj = json.loads(data.decode())
                  #print(json_obj)
                except Exception as e:
                  traceback.print_exc()
                  print(f"kisa_app_thread: json 错误...: {e}")
                  print(data)

              except TimeoutError:
                if not hasattr(self, 'kisa_timeout_cnt'): self.kisa_timeout_cnt = 0
                self.kisa_timeout_cnt += 1
                if self.kisa_timeout_cnt % 100 == 1:
                  print(f"[Kisa] 等待数据中 (超时次数: {self.kisa_timeout_cnt})...")
                #self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"kisa_app_thread: 错误...: {e}")
                #self.remote_addr = None
                break

            except Exception as e:
              print(f"kisa_app_thread: recv 错误...: {e}")
              #self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        #self.remote_addr = None
        print(f"网络错误，重试中...: {e}")
        time.sleep(2)

  def make_tmux_data(self):
    try:
      subprocess.run(f"rm {DATA_PATH}/media/tmux.log; tmux capture-pane -pq -S-1000 > {DATA_PATH}/media/tmux.log", shell=True, capture_output=True, text=False)
      from openpilot.common.basedir import BASEDIR
      subprocess.run(os.path.join(BASEDIR, "selfdrive/apilot.py"), shell=True, capture_output=True, text=False)
    except Exception as e:
      print(f"TMUX 创建错误: {e}")
      return

  def send_tmux(self, ftp_password, tmux_why, send_settings=False):

    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)
    car_selected = Params().get("CarName")
    if car_selected is None:
      car_selected = "none"
    else:
      car_selected = car_selected.decode('utf-8')

    git_branch = Params().get("GitBranch").decode('utf-8')
    try:
      ftp.mkd(git_branch)
    except Exception as e:
      print(f"目录创建失败: {e}")
    ftp.cwd(git_branch)

    directory = car_selected + " " + Params().get("DongleId").decode('utf-8')
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = tmux_why + "-" + current_time + "-" + git_branch + ".txt"

    try:
      ftp.mkd(directory)
    except Exception as e:
      print(f"目录创建失败: {e}")
    ftp.cwd(directory)

    try:
      with open(os.path.join(DATA_PATH, "media/tmux.log"), "rb") as file:
        ftp.storbinary(f'STOR {filename}', file)
    except Exception as e:
      print(f"FTP 发送错误...: {e}")

    if send_settings:
      self.save_toggle_values()
      try:
        #with open("/data/backup_params.json", "rb") as file:
        with open(os.path.join(DATA_PATH, "toggle_values.json"), "rb") as file:
          ftp.storbinary(f'STOR toggles-{current_time}.json', file)
      except Exception as e:
        print(f"FTP 参数发送错误...: {e}")

    ftp.quit()

  def carrot_panda_debug(self):
    #time.sleep(2)
    while True:
      if self.show_panda_debug:
        self.show_panda_debug = False
        try:
          from openpilot.common.basedir import BASEDIR
          subprocess.run(os.path.join(BASEDIR, "selfdrive/debug/debug_console_carrot.py"), shell=True)
        except Exception as e:
          print(f"debug_console 错误: {e}")
          time.sleep(2)
      else:
        time.sleep(1)

  def save_toggle_values(self):
    try:
      import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet

      toggle_values = fleet.get_all_toggle_values()
      file_path = os.path.join(DATA_PATH, 'toggle_values.json')
      with open(file_path, 'w') as file:
        json.dump(toggle_values, file, indent=2)
    except Exception as e:
      print(f"save_toggle_values 错误: {e}")

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    def setup_socket():
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:7710")
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        return socket, poller

    socket, poller = setup_socket()
    isOnroadCount = 0
    is_tmux_sent = False

    print("#########carrot_cmd_zmq: thread started...")
    while True:
      try:
        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          print(f"收到 7710 请求: {message}")
          json_obj = json.loads(message.decode())
        else:
          json_obj = None

        if json_obj is None:
          isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
          if isOnroadCount == 0:
            is_tmux_sent = False
          if isOnroadCount == 1:
            self.show_panda_debug = True

          network_type = self.sm['deviceState'].networkType # if not force_wifi else NetworkType.wifi
          networkConnected = False if network_type == NetworkType.none else True

          if isOnroadCount == 500:
            self.make_tmux_data()
          if isOnroadCount > 500 and not is_tmux_sent and networkConnected:
            self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
            is_tmux_sent = True
          if self.params.get_bool("CarrotException") and networkConnected:
            self.params.put_bool("CarrotException", False)
            self.make_tmux_data()
            self.send_tmux("Ekdrmsvkdlffjt7710", "exception")
        elif 'echo_cmd' in json_obj:
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, capture_output=True, text=False)
            exitStatus = result.returncode
            try:
              stdout = result.stdout.decode('utf-8')
              stderr = result.stderr.decode('utf-8')
            except UnicodeDecodeError:
              stdout = result.stdout.decode('euc-kr', 'ignore')
              stderr = result.stderr.decode('euc-kr', 'ignore')

            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": stdout, "error": stderr})
          except Exception as e:
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": "", "error": f"exception error: {str(e)}"})
          #print(echo)
          socket.send(echo.encode())
        elif 'tmux_send' in json_obj:
          self.make_tmux_data()
          self.send_tmux(json_obj['tmux_send'], "tmux_send")
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
          socket.send(echo.encode())
      except Exception as e:
        print(f"carrot_cmd_zmq 错误: {e}")
        socket.close()
        time.sleep(1)
        socket, poller = setup_socket()

  def recvall(self, sock, n):
    """循环接收数据直到接收到 n 个字节"""
    data = bytearray()
    while len(data) < n:
      packet = sock.recv(n - len(data))
      if not packet:
        return None
      data.extend(packet)
    return data

  def receive_double(self, sock):
    double_data = self.recvall(sock, 8)  # Double 为 8 字节
    return struct.unpack('!d', double_data)[0]

  def receive_float(self, sock):
    float_data = self.recvall(sock, 4)  # Float 为 4 字节
    return struct.unpack('!f', float_data)[0]


  def send_routes(self, coords, from_navd=False):
    if from_navd:
      if len(coords) > 0:
        self.navi_points = [(c.longitude, c.latitude) for c in coords]
        self.navi_points_start_index = 0
        self.navi_points_active = True
        print("从 navd 收到路径点:", len(self.navi_points))
        self.navd_active = True

        # 路径接收 -> carrotman 激活时会存在轻微时间延迟
        if not from_navd:
          self.carrot_serv.active_count = 80
          self.carrot_serv.active_sdi_count = self.carrot_serv.active_sdi_count_max
          self.carrot_serv.active_carrot = 2

        coords = [{"latitude": c.latitude, "longitude": c.longitude} for c in coords]
        #print("navdNaviPoints=", self.navi_points)
      else:
        print("从 navd 收到 0 个路径点")
        self.navd_active = False

    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords
    self.pm.send('navRoute', msg)

  def carrot_route(self):
    host = '0.0.0.0'  # 或其他主机地址
    port = 7709  # 端口号

    try:
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()

        while True:
          print("################# waiting connection from CarrotMan route #####################")
          conn, addr = s.accept()
          with conn:
            print(f"Connected by {addr}")
            #self.clear_route()

            # 接收总数据大小
            total_size_bytes = self.recvall(conn, 4)
            if not total_size_bytes:
              print("连接已关闭或发生错误")
              continue
            try:
              total_size = struct.unpack('!I', total_size_bytes)[0]
              # 一次性接收所有数据
              all_data = self.recvall(conn, total_size)
              if all_data is None:
                  print("连接已关闭或数据接收不完整")
                  continue

              self.navi_points = []
              points = []
              for i in range(0, len(all_data), 8):
                x, y = struct.unpack('!ff', all_data[i:i+8])
                self.navi_points.append((x, y))
                coord = Coordinate.from_mapbox_tuple((x, y))
                points.append(coord)
              coords = [c.as_dict() for c in points]
              self.navi_points_start_index = 0
              self.navi_points_active = True
              print("收到路径点数量:", len(self.navi_points))
              #print("Received points:", self.navi_points)

              self.send_routes(coords)

              if len(coords):
                dest = coords[-1]
                dest['place_name'] = "External Navi"
                self.params.put("NavDestination", json.dumps(dest))

            except Exception as e:
              print(e)
    except Exception as e:
      print("################# CarrotMan route server error #####################")
      print(e)

  def carrot_curve_speed_params(self):
    val_factor = self.params.get_int("AutoCurveSpeedFactor")
    self.autoCurveSpeedFactor = (val_factor if val_factor > 0 else 100) * 0.01
    val_agg = self.params.get_int("AutoCurveSpeedAggressiveness")
    self.autoCurveSpeedAggressiveness = (val_agg if val_agg > 0 else 100) * 0.01

  def carrot_curve_speed(self, sm):
    self.carrot_curve_speed_params()
    if not sm.alive['carState'] and not sm.alive['modelV2']:
        return 250, 0.0
    #print(len(sm['modelV2'].orientationRate.z))
    if len(sm['modelV2'].orientationRate.z) == 0:
        return 250, 0.0

    return self.vturn_speed(sm['carState'], sm)

  def vturn_speed(self, CS, sm):
    TARGET_LAT_A = 1.9  # m/s^2

    modelData = sm['modelV2']
    if len(modelData.orientationRate.z) == 0 or len(modelData.velocity.x) == 0:
      return 250, 0.0

    # 设置弯道敏感度
    orientation_rate = np.abs(np.array(modelData.orientationRate.z)) * self.autoCurveSpeedFactor
    velocity = np.maximum(np.array(modelData.velocity.x), 1.0) # 避免除以零

    # 计算曲率 kappa = yaw_rate / velocity
    curvatures = orientation_rate / velocity
    max_idx = np.argmax(np.abs(curvatures))
    max_curvature = np.abs(curvatures[max_idx])
    curv_direction = np.sign(modelData.orientationRate.z[max_idx])

    # 设置目标侧向加速度
    adjusted_target_lat_a = TARGET_LAT_A * self.autoCurveSpeedAggressiveness

    if max_curvature < 1e-4:
      turnSpeed = 250
    else:
      # turnSpeed = sqrt(a / kappa)
      turnSpeed = (adjusted_target_lat_a / max_curvature)**0.5 * 3.6

    turnSpeed = self.vturn_speed_filter.process(turnSpeed)
    return np.clip(turnSpeed, 5, 250) * curv_direction, max_curvature * curv_direction



import traceback

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man 错误...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
