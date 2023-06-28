#include "vision_common/gate_feature_array.h"

namespace vision_common {

GateFeatureArray::GateFeatureArray() : timestamp(ros::Time::now()), gates() {}

GateFeatureArray::GateFeatureArray(const vision_msgs::GateFeatureArray& msg) {
  timestamp = msg.header.stamp;

  if (msg.gates.empty()) {
    gates.clear();
  } else {
    int size = msg.gates.size();
    gates.resize(size);
    for (int i = 0; i < size; ++i) {
      gates[i] = vision_common::GateFeature(msg.gates[i]);
    }
  }
}

GateFeatureArray::~GateFeatureArray() {}

bool GateFeatureArray::findGate(const std::string gate_id,
                                vision_common::GateFeature& feature) {
  bool found = false;
  auto search = gates.begin();
  for (; search != gates.end(); ++search) {
    if (search->gate_id == gate_id) {
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

vision_msgs::GateFeatureArray GateFeatureArray::toRosMessage() const {
  vision_msgs::GateFeatureArray msg;

  msg.header.stamp = timestamp;
  msg.header.frame_id = "vision";

  for (int i = 0; i < gates.size(); ++i) {
    msg.gates.push_back(gates[i].toRosMessage());
  }
  return msg;
}

}  // namespace vision_common