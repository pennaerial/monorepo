export interface DeployPaths {
  deployRoot: string;
  incomingDir: string;
  releasesDir: string;
  currentLink: string;
  previousLink: string;
  configDir: string;
  stateDir: string;
  missionsDir: string;
  runtimeFleet: string;
  fleetFile: string;
  overlayFile: string;
  runnerScript: string;
}

function defaultDeployRoot(): string {
  return (process.env.DEPLOY_ROOT ?? "/home/penn/pennair-deploy").replace(/\/$/, "");
}

// `root` is read lazily (not captured at module load) so tests can point
// this at a scratch directory via the parameter without needing to set
// process.env before this module is first imported.
export function deployPaths(root: string = defaultDeployRoot()): DeployPaths {
  return {
    deployRoot: root,
    incomingDir: `${root}/incoming`,
    releasesDir: `${root}/releases`,
    currentLink: `${root}/current`,
    previousLink: `${root}/previous`,
    configDir: `${root}/config`,
    stateDir: `${root}/state`,
    missionsDir: `${root}/config/missions`,
    runtimeFleet: `${root}/config/runtime_fleet.yaml`,
    fleetFile: `${root}/config/fleet.yaml`,
    overlayFile: `${root}/config/overlay.yaml`,
    runnerScript: `${root}/config/start-current.sh`,
  };
}
