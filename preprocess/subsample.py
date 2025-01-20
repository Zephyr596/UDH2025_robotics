import pandas as pd
import numpy as np
import rasterio

# 文件路径
dsm_file = "/home/zephyr/wep/UDH2025_robotics/world/Wadi_Birq_elevation_dsm_cropped_margin.tif"
filtered_point_file = "/home/zephyr/wep/UDH2025_robotics/input_data/filtered_geom_lim_10x10.csv"

# 加载 DSM 文件
with rasterio.open(dsm_file) as dsm:
    elevation_data = dsm.read(1)
    transform = dsm.transform

# 读取过滤后的点数据
filtered_df = pd.read_csv(filtered_point_file)

# 添加海拔高度函数
def get_elevation(X, Y):
    row, col = ~transform * (X, Y)
    row, col = int(row), int(col)
    if 0 <= row < elevation_data.shape[0] and 0 <= col < elevation_data.shape[1]:
        return elevation_data[row, col]
    else:
        return np.nan

# 添加海拔高度信息
filtered_df['Z'] = filtered_df.apply(lambda row: get_elevation(row['X'], row['Y']), axis=1)

# 合并逻辑
def merge_points(df, elevation_threshold=5, max_distance=50):
    """
    合并点：如果点在同一区域内（满足海拔差和空间距离条件），选择一个点作为代表
    """
    merged_points = []
    while not df.empty:
        # 基准点
        base_point = df.iloc[0]
        # 找到与基准点在条件范围内的邻近点
        neighbors = df[
            (np.abs(df['Z'] - base_point['Z']) <= elevation_threshold) &  # 海拔差
            (np.sqrt((df['X'] - base_point['X'])**2 + (df['Y'] - base_point['Y'])**2) <= max_distance)  # 空间距离
        ]
        
        # 从邻居中选择一个点作为代表（这里选基准点）
        representative_point = base_point.to_dict()
        merged_points.append(representative_point)
        
        # 移除已处理的点
        df = df.drop(neighbors.index)
    
    return pd.DataFrame(merged_points)

# 执行点合并
merged_df = merge_points(filtered_df, elevation_threshold=20, max_distance=100)

# 保存结果
merged_df.to_csv("/home/zephyr/wep/UDH2025_robotics/preprocess/merged_points.csv", index=False)

print(f"点合并完成，结果保存至 /home/zephyr/wep/UDH2025_robotics/preprocess/merged_points.csv")
