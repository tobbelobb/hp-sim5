import { sensorForceProfileFor } from '../../../../../autocal/control/behaviors/sweep_data_collection.mjs';
const profiles = [
  {drive_anchor:1, sensor_anchor:2, fixed_deltas_mm:[390,0], force_n:[1,1], drive_mm:[-1,1]},
  {drive_anchor:1, sensor_anchor:2, fixed_deltas_mm:[390,-150], force_n:[2,2], drive_mm:[-1,1]},
];
console.log(JSON.stringify({requested_fixed_deltas_mm:profiles[1].fixed_deltas_mm, selected_fixed_deltas_mm:sensorForceProfileFor(profiles,1,2).fixed_deltas_mm}));
