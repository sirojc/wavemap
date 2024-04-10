#include "wavemap/integrator/projection_model/projector_factory.h"

#include "wavemap/integrator/projection_model/ouster_projector.h"
#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"

#include <stdlib.h>

namespace wavemap {
ProjectorBase::Ptr wavemap::ProjectorFactory::create(
    const param::Value& params,
    std::optional<ProjectorType> default_projector_type) {
  if (const auto type = ProjectorType::from(params, "projection_model"); type) {
    return create(type.value(), params);
  }

  if (default_projector_type.has_value()) {
    LOG(WARNING) << "Default type \"" << default_projector_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_projector_type.value(), params);
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

ProjectorBase::Ptr wavemap::ProjectorFactory::create(
    ProjectorType projector_type, const param::Value& params) {
  switch (projector_type.toTypeId()) {
    case ProjectorType::kSphericalProjector: {
      if (const auto config =
              SphericalProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_shared<SphericalProjector>(config.value());
      } else {
        LOG(ERROR) << "Spherical projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kOusterProjector: {
      if (const auto config =
              OusterProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_shared<OusterProjector>(config.value());
      } else {
        LOG(ERROR) << "Ouster projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kPinholeCameraProjector: {
      std::string topic_name_config = params.getChild("projection_model")->getChild("config_topic_name")->get<std::string>().value();      
      ROS_ERROR("%s", ("--------------------kPinholeCameraProjector Initialization for: " + topic_name_config).c_str());

      auto integrator = std::make_shared<PinholeCameraProjector>(topic_name_config);
    
      // return integrator;
      int num_tries = 0;
      while (num_tries < 300) {
        ros::spinOnce();

        if (integrator->isConfigInitialized()) {
          integrator->printConfig();
          ROS_ERROR("Pinhole projector config for topic %s is valid.", topic_name_config.c_str());
          return integrator;
        }
        sleep(1); // wait for 1s
        num_tries++;
      }
      ROS_ERROR("Pinhole projector config for topic %s is not valid after 300 tries.", topic_name_config.c_str());

      return nullptr;
    }
  }

  return nullptr;
}
}  // namespace wavemap
