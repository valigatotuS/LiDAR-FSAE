The needed specifications for the Lidar camera can be determined by understanding the [[APPLICATION]] we will use it in.


## Desired specifications

Type of camera: a **solid-state LiDAR** may be preferred over a scanning LiDAR due to its compact size, real-time capabilities, less complexer and high-resolution data.

Horizontal and vertical field of view: The field of view (FOV) requirements can be determined based on the width of the track, which is a minimum of 3m, and the distance between the cones, which is a maximum of 5m. A LiDAR camera with a wide horizontal FOV and a narrow vertical FOV would be ideal for capturing the track and the cones accurately. (a rough estimate of the **HFOV** angle in the range of **60 to 120** degrees  and **VFOV of 30-40 degrees** could provide an adequate coverage of the track)

Framerate: Given the requirement for real-time processing of the data, it would be ideal to use a LiDAR camera with a high framerate, ideally in the range of **10-30 frames per second**.

Wavelength: The wavelength of the laser used in the LiDAR camera is an important consideration as it affects the range and accuracy of the camera. For high-precision mapping of the track, it would be ideal to use a LiDAR camera with a laser wavelength in the near-infrared (NIR) range, as it provides high accuracy and is less affected by weather conditions.

Range: Given the maximum distance between the cones and the maximum length of a straight, a LiDAR camera with **a range of at least 50-100 meters** would be ideal for mapping the entire circuit.

Precision and accuracy: For accurate mapping of the track, it is important to use a LiDAR camera with high precision and accuracy. This can be ensured by using a high-resolution camera, as well as using advanced algorithms for data processing and filtering.

Trade-offs must be made between the camera characteristics in order to meet cost restrictions and liability of the measurements.

(point density ?)
