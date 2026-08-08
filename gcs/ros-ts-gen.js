// Script to generate ros-ts-gen.json configuration file
// Must have ROS sourced

import fs from 'fs';
const distro = process.env.ROS_DISTRO;

if (!distro) {
  console.error('Error: ROS_DISTRO environment variable is not set.');
  console.error('Please source ROS 2 first');
  process.exit(1);
}

const config = {
  output: "./src/ros_interfaces.ts",
  rosVersion: 2,
  useNamespaces: false,
  smartEnums: true,
  input: [
    { namespace: "std_msgs", path: `/opt/ros/${distro}/share/std_msgs` },
    // { namespace: "geometry_msgs", path: `/opt/ros/${distro}/share/geometry_msgs` }
  ]
};

fs.writeFileSync('ros-ts-gen.json', JSON.stringify(config, null, 2));
console.log(`Wrote ros-ts-gen.json for ROS_DISTRO=${distro}`);
