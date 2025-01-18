# Midterm Report: GPU Integration, Terrain Processing, and No-Fly Zone Filtering in Drone Simulations

## 1. Introduction
This project aims to integrate NVIDIA GPUs into Docker containers for real-time Gazebo simulations, process geospatial data (DEM and RGB), and configure simulation environments to enable drones to navigate realistic terrains. The report also focuses on implementing mathematical filtering methods to identify and restrict no-fly zones.

---

## 2. GPU Integration in Docker for Gazebo Simulation

### Problem
Gazebo simulations require high-performance rendering, which was initially not achievable due to the unavailability of NVIDIA GPUs within Docker containers.

### Solution
1. **NVIDIA Docker Toolkit Installation**:
   - Installed the `nvidia-docker-toolkit` to enable GPU support for Docker containers.
   
2. **Runtime Configuration**:
   - Modified the Docker runtime to use `nvidia` as the default runtime:
     ```bash
     "default-runtime": "nvidia"
     ```

3. **Environment Variable Adjustments**:
   - Set environment variables within the container for OpenGL compatibility:
     ```bash
     export DISPLAY=:0
     export LIBGL_ALWAYS_INDIRECT=1
     ```

4. **Verification**:
   - Used tools like `nvidia-smi` and `glxinfo` inside the container to confirm GPU usage and OpenGL rendering support.

### Outcome
The Docker containers now support GPU acceleration, enabling seamless Gazebo simulations with realistic rendering performance.

---

## 3. DEM and RGB Data Processing

### Steps

1. **Projection System Unification**:
   - Ensured the DEM and RGB datasets shared the same `EPSG` coordinate system using GDAL tools:
     ```bash
     gdalwarp -t_srs EPSG:4326 input_dem.tif output_dem.tif
     ```

2. **Clipping**:
   - Clipped the datasets to the region of interest based on CSV-provided bounds:
     ```bash
     gdal_translate -projwin xmin ymin xmax ymax input.tif output.tif
     ```

3. **Scaling DEM**:
   - Resampled the DEM data to reduce resolution while preserving critical terrain features:
     ```bash
     gdalwarp -tr new_x_resolution new_y_resolution input_dem.tif output_dem.tif
     ```

4. **SDF Generation**:
   - Converted the scaled DEM into a Gazebo-compatible `heightmap` and mapped the RGB dataset as a `texture`:
     ```xml
     <heightmap>
       <texture>path_to_texture.png</texture>
       <image>path_to_dem.png</image>
     </heightmap>
     ```

5. **Integration with PX4**:
   - Added the new world to PX4's Gazebo simulation configuration by updating the `Makefile`:
     ```makefile
     make px4_sitl gz_x500_wadibirk
     ```

---

## 4. Simulation World Configuration

### Start Area and Drone Setup
- Configured the starting area and drone initial position in the Gazebo world file:
  ```xml
  <model name="drone">
    <pose>start_x start_y start_z 0 0 0</pose>
  </model>
  ```
- Validated the drone's ability to:
  1. Spawn correctly in the simulation.
  2. Perform successful takeoff and landing.

---

## 5. Filtering No-Fly Zones

### Overview
To ensure the drone avoids no-fly zones such as buildings and farms, mathematical filtering techniques were applied to both **texture** (RGB data) and **elevation** (DEM data).

### 5.1 Texture-Based Filtering

#### Edge Detection
1. **Input**: RGB image as a texture.
2. **Grayscale Conversion**:
   - Converted RGB to grayscale for simplified processing:
     $$
     I(x, y) = 0.299 \cdot R(x, y) + 0.587 \cdot G(x, y) + 0.114 \cdot B(x, y)
     $$
3. **Gradient Calculation**:
   - Used the Sobel operator to calculate image gradients in \(x\) and \(y\) directions:
     $$
     G_x = \frac{\partial I}{\partial x}, \quad G_y = \frac{\partial I}{\partial y}
     $$
   - Combined gradients to compute the edge magnitude:
     $$
     G(x, y) = \sqrt{G_x^2 + G_y^2}
     $$
4. **Thresholding**:
   - Applied a threshold to detect strong edges:
     $$
     G(x, y) > T_{\text{edge}}
     $$
   - Marked these areas as potential no-fly zones.

#### Color-Based Filtering
1. **Color Masking**:
   - Defined color ranges for specific features (e.g., green for farms, red for buildings):
     $$
     M(x, y) = \begin{cases} 
     1, & \text{if } R(x, y) \text{ and } G(x, y) \text{ meet thresholds} \\
     0, & \text{otherwise}
     \end{cases}
     $$
2. **Morphological Operations**:
   - Performed dilation and erosion to refine masks and eliminate noise.

### 5.2 DEM-Based Filtering

#### Elevation Gradient Calculation
1. **Partial Derivatives**:
   - Estimated the gradient of the DEM using central differences:
     $$
     \frac{\partial z}{\partial x} \approx \frac{z(x+\Delta x, y) - z(x-\Delta x, y)}{2\Delta x}, \quad
     \frac{\partial z}{\partial y} \approx \frac{z(x, y+\Delta y) - z(x, y-\Delta y)}{2\Delta y}
     $$
2. **Gradient Magnitude**:
   - Combined partial derivatives to compute the slope:
     $$
     G(x, y) = \sqrt{\left(\frac{\partial z}{\partial x}\right)^2 + \left(\frac{\partial z}{\partial y}\right)^2}
     $$

#### Thresholding for Elevation
- Applied a threshold to identify steep slopes (>15°):
  $$
  G(x, y) > \tan(15^\circ) \approx 0.2679
  $$

#### Combined Filtering
- Combined texture and DEM filters using logical AND:
  $$
  F_{\text{final}}(x, y) = F_{\text{texture}}(x, y) \land F_{\text{DEM}}(x, y)
  $$

### Results
- Successfully identified buildings, farms, and steep terrains as no-fly zones.
- Marked these areas on the simulation map for testing.

---

## 6. Conclusion
The midterm progress achieved the following:
1. Enabled GPU-accelerated Gazebo simulations in Docker containers.
2. Processed DEM and RGB data for terrain representation.
3. Implemented robust mathematical filtering techniques for no-fly zone identification.

### Future Work
- Fine-tune filtering thresholds for more accurate results.
- Simulate complex flight paths avoiding the filtered no-fly zones.

---

## 7. Attachments
1. DEM filtering workflow image.
2. RGB edge detection code snippet.
3. Gazebo simulation screenshots.
