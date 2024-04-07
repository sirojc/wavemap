#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_

#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/CameraInfo.h>

#include "wavemap/config/config_base.h"
#include "wavemap/integrator/projection_model/projector_base.h"

#include <iostream>

namespace wavemap {
/**
 * Config struct for the pinhole camera projection model.
 */
struct PinholeCameraProjectorConfig
    : ConfigBase<PinholeCameraProjectorConfig, 6> {
  //! The image's width in pixels.
  IndexElement width = -1;
  //! The image's height in pixels.
  IndexElement height = -1;
  //! Fx according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint fx = -1.f;
  //! Fy according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint fy = -1.f;
  //! Cx according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint cx = -1.f;
  //! Cy according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint cy = -1.f;

  static MemberMap memberMap;

  // Constructors
  PinholeCameraProjectorConfig() = default;
  PinholeCameraProjectorConfig(FloatingPoint fx, FloatingPoint fy,
                               FloatingPoint cx, FloatingPoint cy,
                               IndexElement height, IndexElement width)
      : width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy) {}

  bool isValid(bool verbose) const override;
};

class PinholeCameraProjector : public ProjectorBase {
 public:
  using Config = PinholeCameraProjectorConfig;

  explicit PinholeCameraProjector(const std::string& topic_name)
      : ProjectorBase(Vector2D::Ones(), Vector2D::Zero()),
        topic_name_(topic_name) {
    ros::NodeHandle nh;
    camera_info_subscriber_ = nh.subscribe(topic_name_, 1,
        &PinholeCameraProjector::cameraInfoCallback, this);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info) {
    ROS_INFO_ONCE("Received camera info for topic %s", topic_name_.c_str());

    // Update the configuration parameters
    config_.width = camera_info->width;
    config_.height = camera_info->height;
    config_.fx = camera_info->K[0];
    config_.fy = camera_info->K[4];
    config_.cx = camera_info->K[2];
    config_.cy = camera_info->K[5];

    // Update parameters that depend on them
    fxfy_ = config_.fx * config_.fy;
    fxfy_inv_ = 1.f / fxfy_;
    cxfy_ = config_.cx * config_.fy;
    cyfx_ = config_.cy * config_.fx;
  }

  bool isConfigInitialized() const {
    return config_.isValid(false);
  }

  void printConfig() const {
    std::cout << "Config for topic " << topic_name_ << ": " << std::endl 
              << "width: " << config_.width << std::endl 
              << "height: " << config_.height << std::endl 
              << "fx: " << config_.fx << std::endl 
              << "fy: " << config_.fx << std::endl 
              << "cx: " << config_.cx << std::endl 
              << "cy: " << config_.cy << std::endl;
  }

  IndexElement getNumRows() const final { return config_.width; }
  IndexElement getNumColumns() const final { return config_.height; }
  Vector2D getMinImageCoordinates() const final {
    return indexToImage(Index2D::Zero());
  }
  Vector2D getMaxImageCoordinates() const final {
    return {indexToImage({config_.width - 1, config_.height - 1})};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final {
    return {false, false, false};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {false, false, false};
  }
  SiUnit getImageCoordinatesUnit() const final { return SiUnit::kPixels; }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const final;
  Point3D sensorToCartesian(const Vector3D& coordinates) const final;
  Point3D sensorToCartesian(const Vector2D& image_coordinates,
                            FloatingPoint depth) const final;
  FloatingPoint imageOffsetToErrorNorm(const Vector2D& /*linearization_point*/,
                                       Vector2D offset) const final;
  std::array<FloatingPoint, 4> imageOffsetsToErrorNorms(
      const Vector2D& /*linearization_point*/,
      CellToBeamOffsetArray offsets) const final;

  // Projection from Cartesian space onto the sensor's image surface
  Vector2D cartesianToImage(const Point3D& C_point) const final {
    return cartesianToSensor(C_point).head<2>();
  }
  FloatingPoint cartesianToSensorZ(const Point3D& C_point) const final {
    return C_point.z();
  }

  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const final;

 private:
  const std::string topic_name_;
  PinholeCameraProjectorConfig config_;

  ros::Subscriber camera_info_subscriber_;

  FloatingPoint fxfy_ = config_.fx * config_.fy;
  FloatingPoint fxfy_inv_ = 1.f / fxfy_;
  FloatingPoint cxfy_ = config_.cx * config_.fy;
  FloatingPoint cyfx_ = config_.cy * config_.fx;
};



}  // namespace wavemap

#include "wavemap/integrator/projection_model/impl/pinhole_camera_projector_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
