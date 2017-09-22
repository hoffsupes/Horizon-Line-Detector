# Horizon-Line-Detector
Horizon Line Detector which can detect uneven horizons


Preprocesses image using grayscale morphology, followed by image segmentation fitting of countours to the bindary image.

The contours shaping the uneven horizon are found and a "selective trace" is determined in separate segments of the image.

These portions of contours are used to draw lines in the separate segments and a new uneven polygonal horizon is traced.
