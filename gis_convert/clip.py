import os
from osgeo import gdal

# 文件路径
dsm_cropped = "./Wadi_Birq_elevation_dsm_cropped.tif"
geotiff_cropped = "./Wadibirk_geotiff_reprojected_cropped.tif"

# 目标裁减尺寸
target_width = 256
target_height = 256

# 定义裁减函数
def resize_raster(input_file, output_file, width, height):
    gdal.Warp(
        output_file,
        input_file,
        width=width,
        height=height,
        resampleAlg="bilinear"
    )
    print(f"Resized {input_file} to {width}x{height} and saved as {output_file}")

# 输出文件路径
dsm_resized = "./Wadi_Birq_elevation_dsm_resized.tif"
geotiff_resized = "./Wadibirk_geotiff_reprojected_resized.tif"

# 执行裁减
resize_raster(dsm_cropped, dsm_resized, target_width, target_height)
# resize_raster(geotiff_cropped, geotiff_resized, target_width, target_height)
