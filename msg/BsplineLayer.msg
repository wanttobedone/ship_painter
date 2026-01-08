# 包含所有层的B样条轨迹
Bspline[] layers               # 每层的B样条
Bspline[] transitions          # 层间过渡轨迹

# 记录每一层在全局轨迹中结束的时间点 (秒)
float64[] layer_end_times
