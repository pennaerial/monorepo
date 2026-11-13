use crate::config::{ConfigArgs, run_config};
use crate::deps::{DepsArgs, run_deps};
use clap::{Parser, Subcommand};
use std::error::Error;
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[command(version, about)]
pub struct PennairCli {
    /// Path to monorepo.toml (default: $PENNAIR_MONOREPO_PATH/monorepo.toml)
    #[arg(long, global = true)]
    pub manifest: Option<PathBuf>,

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
        let manifest = self.manifest.as_deref();
        match self.command {
            Commands::Deps(args) => run_deps(args, manifest),
            Commands::Config(args) => run_config(args, manifest),
        }
    }
}
