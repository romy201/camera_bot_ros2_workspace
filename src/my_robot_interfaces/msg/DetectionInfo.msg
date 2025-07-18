
bool detected          # True if a human was detected, false otherwise

# Bounding box details (in pixels, relative to top-left of image)
float32 x              # x-coordinate of bounding box top-left corner
float32 y              # y-coordinate of bounding box top-left corner
float32 width          # Width of bounding box
float32 height         # Height of bounding box

# Center of the detected object (in pixels, relative to top-left of image)
float32 center_x       # x-coordinate of bounding box center
float32 center_y       # y-coordinate of bounding box center