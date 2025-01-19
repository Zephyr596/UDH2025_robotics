import rasterio
from rasterio.plot import show
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import cv2

# Paths
height_map_path = r"./world/dsm/Wadi_Birq_elevation_dsm.tif"
texture_path = r"./world/geotiff_RGB/texture_origin_margin.png"
csv_path = r"./input_data/preplan_geom_lim_10m.csv"

# Read height map data
with rasterio.open(height_map_path) as height_src:
    height_data = height_src.read(1)
    height_transform = height_src.transform
    dx = abs(height_transform[0])  # Resolution in x direction
    dy = abs(height_transform[4])  # Resolution in y direction

# Calculate slope
if dx == 0:
    dx = 1e-6  # Avoid division by zero
if dy == 0:
    dy = 1e-6

# Compute gradients
slope_x = np.gradient(height_data, axis=1) / dx
slope_y = np.gradient(height_data, axis=0) / dy
slope = np.arctan(np.sqrt(slope_x**2 + slope_y**2)) * (180 / np.pi)

# Read CSV file
df = pd.read_csv(csv_path)

# Define bounding box from CSV points
min_x, max_x = df['X'].min()-5, df['X'].max()+5
min_y, max_y = df['Y'].min()-5, df['Y'].max()+5

# Convert bounding box to raster indices
min_row, min_col = rasterio.transform.rowcol(height_transform, min_x, max_y)
max_row, max_col = rasterio.transform.rowcol(height_transform, max_x, min_y)

# Subset slope data
slope_subset = slope[min_row:max_row+1, min_col:max_col+1]
subset_transform = height_transform * rasterio.Affine.translation(min_col, min_row)

# Filter points with slope > 15 degrees
filtered_points = []
for _, row in df.iterrows():
    x, y = row['X'], row['Y']
    row_idx, col_idx = rasterio.transform.rowcol(height_transform, x, y)
    if 0 <= row_idx < slope.shape[0] and 0 <= col_idx < slope.shape[1]:
        if slope[row_idx, col_idx] <= 15:
            filtered_points.append((x, y))

filtered_df = pd.DataFrame(filtered_points, columns=['X', 'Y'])

# 添加房屋筛选
def detect_steep_regions(image_path, threshold_value=50):
    """
    读取卫星图像并大致检测“陡峭”区域（基于梯度），在原图上框出这些区域。
    :param image_path: PNG 格式卫星图像路径
    :param threshold_value: Sobel 梯度强度阈值，越大则检测到的区域越少、越精细
    """

    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"无法读取图片文件: {image_path}")
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    
    magnitude = np.sqrt(sobelx**2 + sobely**2)
    magnitude = np.uint8(np.clip(magnitude, 0, 255))
    
    _, thresh = cv2.threshold(magnitude, threshold_value, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return image, contours

image, contours = detect_steep_regions(r"E:\WEP\CZH\UDH2025_robotics\world\geotiff_RGB\texture_origin.png",
                                        threshold_value=170,
                                        )

with rasterio.open(texture_path) as src:
    # Get geotransform and image data
     # 获取地理变换参数
    transform = src.transform
    width = src.width
    height = src.height

rectangles = []
for c in contours:
    x, y, w, h = cv2.boundingRect(c)
    x_world, y_world = transform * (x, y)
    x2_world, y2_world = transform * (x + w, y + h)
    x_world = int(x_world)
    y_world = int(y_world)
    x2_world = int(x2_world)
    y2_world = int(y2_world)
    # print(f"x_world: {x_world}, y_world: {y_world}, x2_world: {x2_world}, y2_world: {y2_world}")
    if x_world > 669500 and x_world < 670500 and w > 15 and h > 15 and y_world < 2577100 and x2_world < 670475:
        rectangles.append((x_world, y_world, x2_world, y2_world))
        cv2.rectangle(image, (x_world, y_world), (x2_world, y2_world), (0, 0, 255), 2)

circle_center = (671300, 2575200)
circle_radius = 220
cv2.circle(image, circle_center, circle_radius, (0, 0, 255), 2)

# 修改
# 过滤掉位于矩形或圆形区域内的点
def is_in_rectangle(x, y, rect):
    """判断点是否在矩形内"""
    return rect[0] <= x <= rect[2] and rect[1] >= y >= rect[3]

def is_in_circle(x, y, center, radius):
    """判断点是否在圆内"""
    return (x - center[0])**2 + (y - center[1])**2 < radius**2


final_points = []
for _, row in filtered_df.iterrows():
    x, y = row['X'], row['Y']
    # 将地理坐标转换为图像像素坐标
    # col, row = rasterio.transform.rowcol(transform, x, y)
    
    # 检查是否在任何矩形或圆形区域内
    in_rectangle = any(is_in_rectangle(x, y, rect) for rect in rectangles)
    in_circle = is_in_circle(x, y, circle_center, circle_radius)
    
    if not in_rectangle and not in_circle:
        final_points.append((x, y))


final_df = pd.DataFrame(final_points, columns=['X', 'Y'])

print(filtered_df.shape)
print(final_df.shape)

# 将最终筛选后的点保存为CSV文件
final_df.to_csv('filtered_points.csv', index=False)



# Read texture image
with rasterio.open(texture_path) as src:
    # Get geotransform and image data
     # 获取地理变换参数
    transform = src.transform
    # 读取图片数据
    img = src.read()

# Plot texture image and filtered points
fig, ax = plt.subplots(figsize=(15, 15), dpi=300)  # Increased DPI and figure size

# Prepare image for plotting
img_for_plot = img.transpose(1, 2, 0)  # Move channel dimension to the end

# Display image
ax.imshow(img_for_plot, extent=(
    transform[2]-0.5*transform[0] , transform[2] + transform[0] * (img.shape[2]+0.5),
    transform[5] + transform[4] * (img.shape[1]+0.5), transform[5]-0.5*transform[4] ))

# Plot filtered points
ax.scatter(final_df['X'], final_df['Y'], c='red', s=0.5, marker='o', alpha=0.7)

plt.title("Filtered Points with Slope <= 15 Degrees (Cropped Region)")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.tight_layout()
plt.savefig("filtered_points_cropped_visualization.png", dpi=600)  # High-res output
# plt.show()
plt.close()
