roslaunch fsc_autopilot run_autopilot_rotors_planning.launch
./trig_target.sh

how to change from our method to virtual method?
(1) in setState
  // virtual feature
  // est_state_((int)StateIndex::FOC_W) = gates.gates[1].center.normal_vector.orientation.w();
  // est_state_((int)StateIndex::FOC_X) = gates.gates[1].center.normal_vector.orientation.x();
  // est_state_((int)StateIndex::FOC_Y) = gates.gates[1].center.normal_vector.orientation.y();
  // est_state_((int)StateIndex::FOC_Z) = gates.gates[1].center.normal_vector.orientation.z();
  // est_state_((int)StateIndex::DISTANCE) = gates.gates[1].center.distance;

(2) replace model