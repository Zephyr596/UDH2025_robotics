import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load local PNG image
image_path = "/home/zephyr/wep/UDH2025_robotics/world/geotiff_RGB/texture_origin.png"  # Replace with your image file path
image = cv2.imread(image_path)  # Load image using OpenCV
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for displaying with Matplotlib

# Convert image to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define color ranges for filtering
# Green range for farms (adjust based on observed farm colors)
green_lower = np.array([35, 40, 40])  # Lower bound for green in HSV
green_upper = np.array([85, 255, 255])  # Upper bound for green in HSV

# Gray/white range for houses (adjust based on observed house colors)
house_lower = np.array([0, 0, 180])  # Lower bound for white/gray in HSV
house_upper = np.array([180, 30, 255])  # Upper bound for white/gray in HSV

# Create masks for farms and houses
green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
house_mask = cv2.inRange(hsv_image, house_lower, house_upper)

# Combine masks
combined_mask = cv2.bitwise_or(green_mask, house_mask)

# Filter out noise with morphological operations
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

# Find contours in the combined mask
contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw contours on the original image
output_image_color_filtered = image_rgb.copy()
for contour in contours:
    if cv2.contourArea(contour) > 500:  # Adjust area threshold as needed
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(output_image_color_filtered, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue boxes

# Display results
fig, axes = plt.subplots(1, 3, figsize=(20, 10))
axes[0].imshow(image_rgb)
axes[0].set_title("Original Image")
axes[0].axis("off")

axes[1].imshow(cleaned_mask, cmap='gray')
axes[1].set_title("Filtered Mask (Green + Houses)")
axes[1].axis("off")

axes[2].imshow(output_image_color_filtered)
axes[2].set_title("Filtered Regions (Color-Based)")
axes[2].axis("off")

plt.tight_layout()
plt.show()
