# Reserved extension, outside the current two benchmark tasks.
# sine_wave | crater | speed_bump | crosswalk
string class_id
float32 confidence
# Ordered polygon vertices; last vertex is implicitly joined to first.
geometry_msgs/Point[] polygon
# Optional estimate: positive height, negative depression; NaN if unavailable.
float32 height
