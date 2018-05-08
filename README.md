# block-detection-OpenCV
A specified color block/cube detection with the camera. The codes are implemented with OpenCV 2.4.13 in visual studio 2013.

This is the visual part codes for the Dobot to grab the block/cube in specified color and detet the black slot (where the block be put on).

## process steps:
1. preprocess the image get from the camera (thresholding, smoothing, and etc.)
2. detect the contours and find the minimum bounding boxes.
3. restrick the shape, area and perimeter for better accuracy.
4. select the most suitable block for grab.
