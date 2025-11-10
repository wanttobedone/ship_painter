# B样条轨迹消息（参考EGO-Planner）
int32 order                    # B样条阶数
time start_time                # 轨迹开始时间
float64 duration               # 轨迹持续时间

float64[] knots                # 节点向量
geometry_msgs/Point[] pos_pts  # 位置控制点
geometry_msgs/Vector3[] normals # 对应的法向量
