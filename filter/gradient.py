import rasterio
from rasterio.plot import show
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import subprocess
# Paths
height_map_path = r"../gis_input_data/dsm/Wadi_Birq_elevation_dsm.tif"
texture_path = r"../world/geotiff_RGB/texture_origin_margin.png"
csv_path = r"../input_data/preplan_geom_lim_10m.csv"

# if the corresponding real x,y of index_x and index_y in dataframe filter_data, remove the entry from filter_data
def check_filter(df,index_x,index_y,min_x,max_y):
    df = df.drop(df[(df['X'] == min_x+index_x*10) & (df['Y'] == max_y-10*index_y)].index)
        
    return df



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

minmin_x = df['X'].min()
minmin_y = df['Y'].max()
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



# Read texture image
with rasterio.open(texture_path) as src:
    # Get geotransform and image data
     # 获取地理变换参数
    transform = src.transform
    # 读取图片数据
    img = src.read()


# Plot texture image and filtered points
fig, ax = plt.subplots(figsize=(15, 15), dpi=100)  # Increased DPI and figure size

# Prepare image for plotting
img_for_plot = img.transpose(1, 2, 0)  # Move channel dimension to the end

# Display image
ax.imshow(img_for_plot, extent=(
    transform[2]-0.5*transform[0], transform[2] + transform[0] * (img.shape[2]+0.5),
    transform[5]+ transform[4] * (img.shape[1]+0.5), transform[5]-0.5*transform[4]))



# final_filter = pd.read_csv('filtered_points.csv')

# filtered_df = filtered_df.merge(final_filter, on=['X', 'Y'], how='inner')

# filtered_df = check_filter(filtered_df,80,106,minmin_x,minmin_y)
# for x in range (72,81):
#     for y in range(107,110):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (73,81):
#     for y in range(113,114):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (80,82):
#     for y in range(122,123):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (87,88):
#     for y in range(110,113):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (85,89):
#     for y in range(118,119):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (83,89):
#     for y in range(120,122):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (84,90):
#     for y in range(124,129):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (76,83):
#     for y in range(125,131):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (77,83):
#     for y in range(131,136):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (73,82):
#     for y in range(120,121):
#         filtered_df.loc[len(filtered_df)] = [minmin_x+x*10, minmin_y + y*-10]
# for x in range (89,95):
#     for y in range(106,117):
#         filtered_df.loc[len(filtered_df)] = [minmin_x+x*10, minmin_y + y*-10]
# for x in range (85,91):
#     for y in range(134,143):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (77,84):
#     for y in range(137,144):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (85,92):
#     for y in range(144,158):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (68,86):
#     for y in range(145,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (67,68):
#     for y in range(146,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (82,89):
#     for y in range(116,124):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (66,67):
#     for y in range(147,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (65,66):
#     for y in range(148,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (64,65):
#     for y in range(151,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (63,64):
#     for y in range(155,159):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (62,93):
#     for y in range(159,160):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (60,72):
#     for y in range(160,174):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (73,86):
#     for y in range(160,174):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (87,95):
#     for y in range(159,175):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (86,95):
#     for y in range(176,180):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (88,95):
#     for y in range(180,187):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (81,86):
#     for y in range(181,187):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (70,76):
#     for y in range(180,189):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (70,75):
#     for y in range(191,178):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (85,95):
#     for y in range(188,194):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (84,89):
#     for y in range(194,200):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (93,94):
#     for y in range(195,196):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (47,52):
#     for y in range(180,185):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (56,60):
#     for y in range(183,191):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (61,66):
#     for y in range(190,197):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (61,66):
#     for y in range(198,200):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (55,60):
#     for y in range(191,196):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (59,60):
#     for y in range(196,200):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (56,59):
#     for y in range(198,200):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (52,54):
#     for y in range(194,196):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (43,49):
#     for y in range(196,200):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (72,82):
#     for y in range(115,122):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# for x in range (70,76):
#     for y in range(191,198):
#         filtered_df = check_filter(filtered_df,x,y,minmin_x,minmin_y)
# filtered_df.to_csv('refined.csv', index=False)
# Plot filtered points
# ax.scatter(filtered_df['X'], filtered_df['Y'], c='red', s=0.5, marker='o', alpha=0.7)
ser_filtered_file = "../input_data/filtered_geom_lim_10x10.csv"

refined_file = "./refined.csv"
df = pd.read_csv(ser_filtered_file)

ax.scatter(df['X'], df['Y'], c='red', s=0.5, marker='o', alpha=0.7)

plt.title("Filtered Points")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.tight_layout()
plt.savefig("filtered_points_ser.png", dpi=600)  # High-res output
plt.close()