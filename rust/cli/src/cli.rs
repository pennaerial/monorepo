use crate::config::{ConfigArgs, run_config};
use crate::deps::{DepsArgs, run_deps};
use crate::gateway::{self, State};
use crate::{flash, push};
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
    /// Check or install the dependencies listed in monorepo.toml
    Deps(DepsArgs),
    /// Check monorepo.toml
    Config(ConfigArgs),
    /// Push source to Pis, build it there, and restart the autonomy service
    Push {
        /// ssh targets, e.g. penn@air-03.local
        #[arg(required = true)]
        hosts: Vec<String>,
        /// Build without restarting the service
        #[arg(long)]
        no_restart: bool,
    },
    /// Pull source edited on a Pi back into this checkout
    Pull {
        host: String,
        /// Overwrite uncommitted local changes
        #[arg(long)]
        force: bool,
    },
    /// Show what was last pushed to each Pi and what's been edited there since
    Status {
        #[arg(required = true)]
        hosts: Vec<String>,
    },
    /// Write the Pi image to an SD card and name it air-<ID>
    Flash {
        /// Pi number, e.g. 6 for air-06
        id: u8,
        /// SD card device, e.g. /dev/sda
        device: String,
        /// Use a local .img.xz instead of downloading the latest release
        #[arg(long)]
        image: Option<PathBuf>,
    },
    /// Share this laptop's internet with Pis over Ethernet and the mesh (Linux only)
    Gateway { state: State },
}

impl PennairCli {
    pub fn run(self) -> Result<(), Box<dyn Error>> {
        let manifest = self.manifest.as_deref();
        match self.command {
            Commands::Deps(args) => run_deps(args, manifest),
            Commands::Config(args) => run_config(args, manifest),
            Commands::Push { hosts, no_restart } => push::push(manifest, &hosts, !no_restart),
            Commands::Pull { host, force } => push::pull(manifest, &host, force),
            Commands::Status { hosts } => push::status(&hosts),
            Commands::Flash { id, device, image } => flash::flash(id, &device, image.as_deref()),
            Commands::Gateway { state } => gateway::gateway(state),
        }
    }
}
