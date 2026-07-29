/** Name of the systemd unit that runs the deployed ROS2 fleet launch --
 *  shared by deploy.ts, mission.ts, and logs.ts so they all target the same
 *  unit `provision-pi.sh`/`bootstrap-pi.sh` installs. */
export const SERVICE_NAME = process.env.MISSION_SERVICE_NAME ?? "pennair-autonomy.service";
