use pennair_protocol::DEFAULT_AGENT_PORT;
use std::{
    env,
    error::Error,
    net::SocketAddr,
    path::{Path, PathBuf},
};

pub struct AgentConfig {
    pub bind_address: SocketAddr,
    pub agent_name: String,
    pub token: String,
    pub deployment_root: PathBuf,
    pub build_shell: PathBuf,
    pub ros_setup: PathBuf,
    pub colcon: PathBuf,
    pub sudo: PathBuf,
    pub systemctl: PathBuf,
    pub service_name: String,
}

impl AgentConfig {
    pub fn from_env() -> Result<Self, Box<dyn Error>> {
        let bind_address = env::var("PENNAIR_AGENT_BIND")
            .unwrap_or_else(|_| format!("0.0.0.0:{DEFAULT_AGENT_PORT}"))
            .parse()?;
        let agent_name = env::var("PENNAIR_AGENT_NAME")
            .or_else(|_| env::var("HOSTNAME"))
            .unwrap_or_else(|_| "pennair-agent".into());
        let token =
            env::var("PENNAIR_AGENT_TOKEN").map_err(|_| "PENNAIR_AGENT_TOKEN must be set")?;
        if token.len() < 32 {
            return Err("PENNAIR_AGENT_TOKEN must contain at least 32 characters".into());
        }
        let deployment_root = match env::var("PENNAIR_AGENT_ROOT") {
            Ok(path) => PathBuf::from(path),
            Err(_) => {
                let home = env::var("HOME").map_err(
                    |_| "HOME or PENNAIR_AGENT_ROOT must be set to locate the deployment root",
                )?;
                Path::new(&home).join("monorepo")
            }
        };
        let build_shell = env::var("PENNAIR_AGENT_BUILD_SHELL")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("/bin/bash"));
        let ros_setup = env::var("PENNAIR_AGENT_ROS_SETUP")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("/opt/ros/jazzy/setup.sh"));
        let colcon = env::var("PENNAIR_AGENT_COLCON")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("colcon"));
        let sudo = env::var("PENNAIR_AGENT_SUDO")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("sudo"));
        let systemctl = env::var("PENNAIR_AGENT_SYSTEMCTL")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("systemctl"));
        let service_name =
            env::var("PENNAIR_AGENT_SERVICE").unwrap_or_else(|_| "pennair-autonomy.service".into());

        Ok(Self {
            bind_address,
            agent_name,
            token,
            deployment_root,
            build_shell,
            ros_setup,
            colcon,
            sudo,
            systemctl,
            service_name,
        })
    }
}
