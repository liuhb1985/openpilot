import math
import numpy as np
from cereal import log
try:
  from openpilot.common.filter_simple import FirstOrderFilter
except ImportError:
  from common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
# from openpilot.common.swaglog import cloudlog
# from openpilot.common.logger import sLogger
from openpilot.common.params import Params

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0.04 # Match lateral_planner.py
MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
MAX_LANE_CENTERING_AWAY = 1.85
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MIN_DISTANCE_FROM_EDGELANE = 1.15

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    #self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_estimate = FirstOrderFilter(3.2, 3.0, DT_MDL)
    self.lane_width = 3.2
    self.lane_width_last = self.lane_width
    self.lane_change_multiplier = 1
    #self.lane_width_updated_count = 0

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.debugText = ""
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 2.0, DT_MDL)
    self.lane_offset_lead_filtered = FirstOrderFilter(0.0, 1.0, DT_MDL)

    self.lanefull_mode = False
    self.d_prob_count = 0

    self.params = Params()

  def parse_model(self, md):

    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) >= 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.Desire.laneChangeRight]

  def get_d_path(self, CS, v_ego, path_t, path_xyz, curve_speed, lane_offset=0.0, curve_offset=0.0, radarState=None):
    #if v_ego > 0.1:
    #  self.lane_width_updated_count = max(0, self.lane_width_updated_count - 1)
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = np.interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      #prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      prob_mods.append(np.interp(width_at_t, [4.5, 6.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    self.l_prob, self.r_prob = l_prob, r_prob

    # Find current lanewidth
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])

    max_updated_count = 10.0 * DT_MDL
    both_lane_available = False
    #speed_lane_width = np.interp(v_ego*3.6, [0., 60.], [2.8, 3.5])
    if l_prob > 0.5 and r_prob > 0.5 and self.lane_change_multiplier > 0.5:
      both_lane_available = True
      #self.lane_width_updated_count = max_updated_count
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width_last = self.lane_width_estimate.x
    #elif self.lane_width_updated_count <= 0 and v_ego > 0.1:   # 双侧车道线缺失时.... 经过一定时间（10秒）后应用 speed 车道宽度.
    #  self.lane_width_estimate.update(speed_lane_width)
    else:
      self.lane_width_estimate.update(self.lane_width_last)

    self.lane_width =  self.lane_width_estimate.x
    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # 选择车道线最清晰的一侧..
    self.d_prob = max(l_prob, r_prob) if not both_lane_available else 1.0

    # 过滤左/右车道宽度.
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
      #self.lane_width_left_filtered.x = self.lane_width_left #立即应用
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)
      #self.lane_width_right_filtered.x = self.lane_width_right #立即应用

    self.adjustLaneOffset = lane_offset
    self.adjustCurveOffset = curve_offset
    #self.adjustCurveOffset = self.adjustLaneOffset #float(self.params.get_int("AdjustCurveOffset")) * 0.01
    ADJUST_OFFSET_LIMIT = 0.4 #max(self.adjustLaneOffset, self.adjustCurveOffset)
    ## [Optimization] 增加切大弯平滑抑制 (Curve Offset Smoothing) 
    # 吸纳模型给出的切弯偏移请求并低通滤波，防止遇弯方向盘突兀猛拽
    raw_offset_curve = np.interp(abs(curve_speed), [40, 50, 200, 250], [0.0, self.adjustCurveOffset, 0.0, 0.0]) * np.sign(curve_speed)
    if not hasattr(self, 'offset_curve_ema'):
      self.offset_curve_ema = raw_offset_curve
    self.offset_curve_ema = 0.02 * raw_offset_curve + 0.98 * self.offset_curve_ema
    offset_curve = self.offset_curve_ema

    diff_width = self.lane_width_left_filtered.x - self.lane_width_right_filtered.x
    offset_lane = np.interp(diff_width, [-0.4, 0.4], [-self.adjustLaneOffset, self.adjustLaneOffset])
    offset_lane *= np.interp(self.lane_width, [2.5, 2.9], [0.0, 1.0])

    #select lane path
    # 车道变窄时，优先沿道路边界一侧的车道行驶.
    if self.lane_width < 2.5:
      if r_prob > 0.5 and self.lane_width_right_filtered.x < self.lane_width_left_filtered.x:
        lane_path_y = path_from_right_lane
      elif l_prob > 0.5 and self.lane_width_left_filtered.x < 2.0:
        lane_path_y = path_from_left_lane
      else:
        lane_path_y = path_from_left_lane if l_prob > 0.5 or l_prob > r_prob else path_from_right_lane
    elif l_prob > 0.7 and r_prob > 0.7:
      lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
      # 由于 lane_width 过滤，逐渐减少时会出现偏向中心线的现象..
      #if self.lane_width > 3.2:
      #  lane_path_y = path_from_right_lane
      #else:
      #  lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
    # 否则沿清晰的车道线行驶.
    else:
      lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)

    # [Optimization] 激活纯视觉模型下的动态居中偏移对齐 (Fix Right Bias)
    # 对于纯模型偏右问题，通过比对视觉左/右线的预期几何中心与模型裸中心，补回偏移量
    use_laneless_center_adjust = True
    if use_laneless_center_adjust:
      # 看前方 1.0 秒（更远、更平滑，不建议用 0.5 秒）
      lane_path_y_center = np.interp(1.0, path_t, lane_path_y)
      path_xyz_y_center = np.interp(1.0, path_t, path_xyz[:,1])
      
      if not self.lanefull_mode:
        # 模型偏差差值 * 0.6 (防修正过猛画龙) * d_prob (车道线置信度)
        diff_center = (lane_path_y_center - path_xyz_y_center) * 0.6 * self.d_prob
      else:
        diff_center = 0.0
    else:
      diff_center = 0.0
    self.offset_lead = 0.0
    if radarState is not None and radarState.leadOne.status:
      lead = radarState.leadOne
      
      # [Optimization] 前车交互双模体系: 绕开静止物体 vs 平滑跟随防加塞
      if 2.0 < lead.dRel < 50.0:
        is_stationary_or_slow = lead.vLead < 5.0 # 时速低于 ~18km/h 判为临停或缓行路口挪车
        
        if is_stationary_or_slow and 0.5 < abs(lead.yRel) < 2.5:
          # [模式一: 危险边缘物体逆向绕行] (Stationary Avoidance)
          # 对于越跨线的静止车躲避得越宽（最大 0.5m 反向偏移）
          self.offset_lead = np.interp(abs(lead.yRel), [0.5, 1.5, 2.5], [0.3, 0.5, 0.0]) * -np.sign(lead.yRel)
        elif not is_stationary_or_slow and abs(lead.yRel) < 1.2:
          # [模式二: 正常行驶前车正向平顺跟随] (Traffic Following)
          # 在车道线不清（d_prob < 0.6）时，优雅地追踪前车留在前方的安全轨迹，有车道线时弱化
          follow_strength = np.interp(self.d_prob, [0.0, 0.6], [0.4, 0.0]) 
          self.offset_lead = np.interp(abs(lead.yRel), [0.0, 1.2], [0.0, follow_strength]) * np.sign(lead.yRel)

        # 随距离产生效果衰减：太远没必要，太近由于来不及而交还给 AEB
        dist_factor = np.interp(lead.dRel, [2, 10, 35, 50], [0.0, 1.0, 1.0, 0.0])
        self.offset_lead *= dist_factor
    
    self.lane_offset_lead_filtered.update(self.offset_lead)

    if offset_curve * offset_lane < 0:
      offset_total = np.clip(offset_curve + offset_lane + diff_center + self.lane_offset_lead_filtered.x, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)
    else:
      offset_total = np.clip(max_abs(offset_curve, offset_lane) + diff_center + self.lane_offset_lead_filtered.x, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    ## self.d_prob = 0 if lane_changing
    self.d_prob *= self.lane_change_multiplier  ## 变更车道期间关闭.
    if self.lane_change_multiplier < 0.5:
      #self.lane_offset_filtered.x = 0.0
      pass
    else:
      self.lane_offset_filtered.update(np.interp(self.d_prob, [0, 0.3], [0, offset_total]))

    ## laneless at lowspeed
    self.d_prob *= np.interp(v_ego*3.6, [5., 10.], [0.0, 1.0])

    #self.debugText = "OFFSET({:.2f}={:.2f}+{:.2f}+{:.2f}),Vc:{:.2f},dp:{:.1f},lf:{},lrw={:.1f}|{:.1f}|{:.1f}".format(
    #  self.lane_offset_filtered.x,
    #  diff_center, offset_lane, offset_curve,
    #  curve_speed,
    #  self.d_prob, self.lanefull_mode,
    #  self.lane_width_left_filtered.x, self.lane_width, self.lane_width_right_filtered.x)

    adjustLaneTime = self.params.get_float("LatMpcInputOffset") * 0.01 # 0.06
    laneline_active = False
    self.d_prob_count = self.d_prob_count + 1 if self.d_prob > 0.3 else 0
    if self.lanefull_mode and self.d_prob_count > int(1 / DT_MDL):
      laneline_active = True
      use_dist_mode = False  ## 无论怎么看.. 好像都是同一种方法...
      if use_dist_mode:
        lane_path_y_interp = np.interp(path_xyz[:,0] + v_ego * adjustLaneTime, self.ll_x, lane_path_y)
        path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
      else:
        safe_idxs = np.isfinite(self.ll_t)
        if safe_idxs[0]:
          lane_path_y_interp = np.interp(path_t * (1.0 + adjustLaneTime), self.ll_t[safe_idxs], lane_path_y[safe_idxs])
          path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]


    path_xyz[:, 1] += (CAMERA_OFFSET + self.lane_offset_filtered.x)

    self.offset_total = self.lane_offset_filtered.x

    return path_xyz, laneline_active

  def calculate_plan_yaw_and_yaw_rate(self, path_xyz):
    if path_xyz.shape[0] < 3:
        # 如果太短，假设直行
        N = path_xyz.shape[0]
        return np.zeros(N), np.zeros(N)

    # 提取 x, y
    x = path_xyz[:, 0]
    y = path_xyz[:, 1]

    # 确认是否所有点都相同
    if np.allclose(x, x[0]) and np.allclose(y, y[0]):
        return np.zeros(len(x)), np.zeros(len(x))

    # 安全的 diff 计算
    dx = np.diff(x)
    dy = np.diff(y)
    mask = (dx == 0) & (dy == 0)
    dx[mask] = 1e-4
    dy[mask] = 0.0

    yaw = np.arctan2(dy, dx)
    yaw = np.append(yaw, yaw[-1])  # N-1 → N
    yaw = np.unwrap(yaw)

    dx_full = np.clip(np.diff(x), 1e-4, None)
    yaw_rate = np.diff(yaw) / dx_full
    yaw_rate = np.append(yaw_rate, yaw_rate[-1])
    yaw_rate = np.append(yaw_rate, 0.0)

    # NaN/Inf 防御
    if np.any(np.isnan(yaw_rate)) or np.any(np.isinf(yaw_rate)):
        yaw_rate = np.zeros_like(yaw_rate)
    if np.any(np.isnan(yaw)) or np.any(np.isinf(yaw)):
        yaw = np.zeros_like(yaw)

    return yaw, yaw_rate
