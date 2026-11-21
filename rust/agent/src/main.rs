use std::process::ExitCode;

use pennair_agent::{config::AgentConfig, server::AgentServer};

fn main() -> ExitCode {
    let result = AgentConfig::from_env().and_then(|config| AgentServer::new(config).run());
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
