use crate::config::{ConfigArgs, run_config};
use crate::deps::{DepsArgs, run_deps};
use clap::{Parser, Subcommand};
use std::error::Error;

#[derive(Debug, Parser)]
#[command(version, about)]
pub struct PennairCli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Subcommand for managing dependencies
    Deps(DepsArgs),
    /// Subcommand for managing monorepo.toml config
    Config(ConfigArgs),
}

impl PennairCli {
    pub fn run(self) -> Result<(), Box<dyn Error>> {
        match self.command {
            Commands::Deps(args) => run_deps(args),
            Commands::Config(args) => run_config(args),
        }
    }
}
