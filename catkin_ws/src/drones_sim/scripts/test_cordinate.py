import pandas as pd
import random

# 文件路径
original_file = "/home/zephyr/wep/UDH2025_robotics/input_data/preplan_geom_lim_10m.csv"
refined_file = "/home/zephyr/wep/UDH2025_robotics/filter/refined.csv"
sdf_output_file = "/home/zephyr/wep/UDH2025_robotics/world/wadibirk_updated.sdf"

# 加载原始和过滤数据
original_data = pd.read_csv(original_file)
refined_data = pd.read_csv(refined_file)

# 提取边界值
x_min, x_max = original_data['X'].min(), original_data['X'].max()
y_min, y_max = original_data['Y'].min(), original_data['Y'].max()

# 定义坐标转换函数
def transform_to_gazebo(x, y, x_min, x_max, y_min, y_max):
    gazebo_x = ((x - x_min) * 1990 / (x_max - x_min)) - 995
    gazebo_y = ((y - y_min) * 1990 / (y_max - y_min)) - 995
    return gazebo_x, gazebo_y

# 转换过滤后的数据
refined_data[['gazebo_x', 'gazebo_y']] = refined_data.apply(
    lambda row: transform_to_gazebo(row['X'], row['Y'], x_min, x_max, y_min, y_max), axis=1, result_type='expand'
)

# 随机抽样 1000 个点
sampled_points = refined_data.sample(n=1000, random_state=42)  # 固定随机种子以便结果一致

# 生成 .sdf 文件
sdf_template = """
<model name="start_area_{id}">
  <static>true</static>
  <pose>{x} {y} 25 0 0 0</pose>
  <link name="start_area_link_{id}">
    <collision name="start_area_collision_{id}">
      <geometry>
        <box>
          <size>2 2 10</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode/>
        </friction>
        <bounce/>
        <contact/>
      </surface>
    </collision>
    <visual name="start_area_visual_{id}">
      <geometry>
        <box>
          <size>2 2 10</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
  </link>
</model>
"""

# 构建模型部分
models = ""
for idx, row in sampled_points.iterrows():
    model = sdf_template.format(id=idx, x=row['gazebo_x'], y=row['gazebo_y'])
    models += model

# 读取原始 .sdf 文件
with open("/home/zephyr/wep/UDH2025_robotics/world/wadibirk.sdf", "r") as sdf_file:
    original_sdf = sdf_file.read()

# 插入新的模型
updated_sdf = original_sdf.replace("</world>", models + "\n</world>")

# 写入更新后的 .sdf 文件
with open(sdf_output_file, "w") as output_file:
    output_file.write(updated_sdf)

print(f"Updated .sdf file with 1000 randomly sampled points has been saved to: {sdf_output_file}")
