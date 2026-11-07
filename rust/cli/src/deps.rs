use clap::{Args, Subcommand};
use std::error::Error;

pub fn run_deps(args: DepsArgs) -> Result<(), Box<dyn Error>> {
    match args.command {
        DepsCommand::Check => {
            println!("checking dependencies");
            Ok(())
        }
        DepsCommand::Sync => {
            println!("syncing dependencies");
            Ok(())
        }
    }
}

#[derive(Args, Debug)]
pub struct DepsArgs {
    #[command(subcommand)]
    pub command: DepsCommand,
}

#[derive(Subcommand, Debug)]
pub enum DepsCommand {
    /// Check installed dependencies against monorepo manifest
    Check,
    /// Sync system dependencies with monorepo.toml configuration
    Sync,
}
