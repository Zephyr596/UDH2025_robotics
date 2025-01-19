gdal_translate -scale -ot Byte -of PNG Wadi_Birq_elevation_dsm_resized.tif ./dsm/heightmap.png
# gdal_translate -scale -ot Byte -of PNG ./Wadibirk_geotiff_reprojected_resized.tif texture.png
gdal_translate -scale -ot Byte -of PNG ./Wadibirk_geotiff_reprojected_cropped.tif ./geotiff_RGB/texture_origin.png