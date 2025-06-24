# PathWithBoundaries.msg

# 路径信息
geometry_msgs/Point[] path

# 左边界点
geometry_msgs/Point[] sorted_left

# 右边界点
geometry_msgs/Point[] sorted_right

# 带虚拟点的左边界
geometry_msgs/Point[] left_cones_with_virtual

# 带虚拟点的右边界
geometry_msgs/Point[] right_cones_with_virtual

# 左右边界匹配
int32[] left_to_right_match
int32[] right_to_left_match
