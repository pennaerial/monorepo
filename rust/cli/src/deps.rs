use clap::{Args, Subcommand};
use std::error::Error;

pub fn run_deps(args: DepsArgs) -> Result<(), Box<dyn Error>> {
    match args.command {
        DepsCommand::Check { dev, prod }=> {
            let m = if dev {"dev"} else {"prod"};
            println!("checking {m} dependencies");
            Ok(())
        }
        DepsCommand::Sync { dev, prod }=> {
            let m = if dev {"dev"} else {"prod"};
            println!("syncing {m} dependencies");
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
    Check {
        #[arg(long, conflicts_with = "prod", required_unless_present = "prod")]
        dev: bool,

        #[arg(long)]
        prod: bool
    },
    /// Sync system dependencies with monorepo.toml configuration
    Sync {
        #[arg(long, conflicts_with = "prod", required_unless_present = "prod")]
        dev: bool,

        #[arg(long)]
        prod: bool
    },
}
