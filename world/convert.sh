gdal_translate -scale -ot Byte -of PNG Wadi_Birq_elevation_dsm_resized_margin.tif ./dsm/heightmap_margin.png
# gdal_translate -scale -ot Byte -of PNG ./Wadibirk_geotiff_reprojected_resized.tif texture.png
# gdal_translate -scale -ot Byte -of PNG ./Wadibirk_geotiff_reprojected_cropped.tif ./geotiff_RGB/texture_origin.png