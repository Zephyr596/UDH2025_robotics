import pandas as pd
import os
from osgeo import gdal, osr

def reproject_raster(input_file, output_file, target_proj):
    """
    重投影 GeoTIFF 文件
    """
    src_ds = gdal.Open(input_file)
    
    # 设置目标投影
    target_srs = osr.SpatialReference()
    target_srs.ImportFromWkt(target_proj)

    # 创建重投影后的文件
    dst_ds = gdal.Warp(
        output_file,
        src_ds,
        dstSRS=target_srs.ExportToWkt(),
        resampleAlg="bilinear"
    )

    src_ds = None
    dst_ds = None
    print(f"Reprojected {input_file} to {output_file} using target projection.")

def crop_raster_with_margin(input_file, output_file, xmin, ymin, xmax, ymax, margin):
    """
    裁剪 GeoTIFF 文件，并在四周预留边缘
    """
    # 计算预留边缘后的坐标
    xmin_new = xmin - margin
    ymin_new = ymin - margin
    xmax_new = xmax + margin
    ymax_new = ymax + margin

    gdal.Warp(
        output_file,
        input_file,
        outputBounds=(xmin_new, ymin_new, xmax_new, ymax_new),
        cropToCutline=True
    )
    print(f"Cropped {input_file} to {output_file} with bounds {xmin_new}, {ymin_new}, {xmax_new}, {ymax_new}.")

def get_raster_size(file_path):
    """
    获取栅格文件的尺寸
    """
    ds = gdal.Open(file_path)
    width = ds.RasterXSize
    height = ds.RasterYSize
    ds = None
    return width, height

# 文件路径
gis_input_dir = "../gis_input_data"
dsm_file = os.path.join(gis_input_dir, "dsm/Wadi_Birq_elevation_dsm.tif")
geotiff_file = os.path.join(gis_input_dir, "geotiff_RGB/Wadibirk_geotiff.tif")
input_data_dir = "../input_data"
csv_file = os.path.join(input_data_dir, "preplan_geom_lim_10m.csv")

# 重投影文件路径
geotiff_reprojected = "./Wadibirk_geotiff_reprojected.tif"

# 裁剪文件路径
dsm_cropped = "./Wadi_Birq_elevation_dsm_cropped.tif"
geotiff_cropped = "./Wadibirk_geotiff_reprojected_cropped.tif"

# 获取 DSM 文件的投影
dsm_ds = gdal.Open(dsm_file)
gt = dsm_ds.GetGeoTransform()
pixel_size = gt[1] 
dsm_proj = dsm_ds.GetProjection()
dsm_ds = None
# 增加一个像素大小的边缘
margin = 1

# 1. 重投影 GeoTIFF 文件
reproject_raster(geotiff_file, geotiff_reprojected, dsm_proj)

# 2. 读取 CSV 文件的坐标范围
df = pd.read_csv(csv_file)
xmin, ymin = df["X"].min(), df["Y"].min()
xmax, ymax = df["X"].max(), df["Y"].max()

# 3. 根据坐标范围裁剪 DSM 和重投影后的 GeoTIFF 文件
crop_raster_with_margin(dsm_file, dsm_cropped, xmin, ymin, xmax, ymax, margin)
crop_raster_with_margin(geotiff_reprojected, geotiff_cropped, xmin, ymin, xmax, ymax, margin)

# 4. 输出裁剪后文件的尺寸
for file_path, name in zip([dsm_cropped, geotiff_cropped], ["DSM", "Reprojected GeoTIFF"]):
    width, height = get_raster_size(file_path)
    print(f"{name} ({file_path}) Size: Width={width}, Height={height}")