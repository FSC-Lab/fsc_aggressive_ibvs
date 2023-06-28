#include "vision_common/visual_features.h"

namespace vision_common {

VisualFeatures::VisualFeatures() : timestamp(ros::Time::now()), features() {}

VisualFeatures::VisualFeatures(
    const ros::Time& timestamp,
    const std::vector<vision_common::VisualFeature>& features)
    : timestamp(timestamp), features(features) {}

VisualFeatures::VisualFeatures(const ros::Time& timestamp,
                               const vision_common::VisualFeature& feature)
    : timestamp(timestamp) {
  features.clear();
  features.push_back(feature);
}

VisualFeatures::VisualFeatures(
    const std::vector<vision_common::VisualFeature>& features)
    : VisualFeatures(ros::Time::now(), features) {}

VisualFeatures::VisualFeatures(const vision_common::VisualFeature& feature)
    : VisualFeatures(ros::Time::now(), feature) {}

VisualFeatures::VisualFeatures(const vision_msgs::VisualFeatures& msg) {
  timestamp = msg.header.stamp;

  if (msg.features.empty()) {
    features.clear();
  } else {
    int size = msg.features.size();
    features.resize(size);
    for (int i = 0; i < size; ++i) {
      features[i] = vision_common::VisualFeature(msg.features[i]);
    }
  }
}

VisualFeatures::~VisualFeatures() {}

bool VisualFeatures::findFeature(const std::string id,
                                 vision_common::VisualFeature& feature) {
  bool found = false;
  auto search = features.begin();
  for (; search != features.end(); ++search) {
    if (search->id == id) {
      found = true;
      break;
    }
  }

  if (!found) {
    return false;
  }

  feature = *search;
  return true;
}

vision_msgs::VisualFeatures VisualFeatures::toRosMessage() const {
  vision_msgs::VisualFeatures msg;

  msg.header.stamp = timestamp;
  msg.header.frame_id = "vision";

  for (int i = 0; i < features.size(); ++i) {
    msg.features.push_back(features[i].toRosMessage());
  }
  return msg;
}

bool VisualFeatures::isValid() const {
  for (int i = 0; i < features.size(); ++i) {
    if (!features[i].isValid()) {
      return false;
    }
  }
  return true;
}

}  // namespace vision_common