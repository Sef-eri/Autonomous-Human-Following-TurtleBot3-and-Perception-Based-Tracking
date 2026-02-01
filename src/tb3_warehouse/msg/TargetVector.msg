# Target position of detected person relative to camera frame
float32 x          # Horizontal offset (-1.0 to 1.0, 0 = center)
float32 y          # Vertical offset (-1.0 to 1.0, 0 = center)
float32 distance   # Estimated distance (normalized 0-1 based on bbox size)
bool detected      # True if person is detected
