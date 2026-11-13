use crate::config::{MonorepoConfig, load_config};
use crate::{apt, python};
use clap::{Args, Subcommand};
use std::collections::BTreeSet;
use std::error::Error;
use std::path::Path;

pub fn run_deps(args: DepsArgs, manifest: Option<&Path>) -> Result<(), Box<dyn Error>> {
    match args.command {
        DepsCommand::Check { dev, .. } => {
            let (path, config) = load_config(manifest)?;
            let env = if dev { "dev" } else { "prod" };
            println!("checking {env} dependencies against {}", path.display());
            check(&config, env)
        }
        DepsCommand::Sync { dev, .. } => {
            let m = if dev { "dev" } else { "prod" };
            println!("syncing {m} dependencies");
            Ok(())
        }
    }
}

/// Reports apt and python packages from the environment's groups that aren't installed.
/// Versions aren't checked yet.
fn check(config: &MonorepoConfig, env_name: &str) -> Result<(), Box<dyn Error>> {
    let env = config
        .environments
        .get(env_name)
        .ok_or(format!("no [environments.{env_name}] in manifest"))?;

    let mut apt_wanted = BTreeSet::new();
    let mut python_wanted = BTreeSet::new();
    for name in &env.groups {
        let group = config.groups.get(name).ok_or(format!(
            "environment '{env_name}' uses unknown group '{name}'"
        ))?;
        apt_wanted.extend(group.packages.iter().map(String::as_str));
        python_wanted.extend(group.python_packages.iter().map(String::as_str));
    }

    let apt_installed = apt::installed_packages()?;
    let apt_missing: Vec<&str> = apt_wanted
        .iter()
        .filter(|p| !apt_installed.contains(**p))
        .copied()
        .collect();

    let python_installed = python::installed_packages()?;
    let python_missing: Vec<&str> = python_wanted
        .iter()
        .filter(|r| !python_installed.contains(&python::requirement_name(r)))
        .copied()
        .collect();

    report("apt", apt_wanted.len(), &apt_missing);
    report("python", python_wanted.len(), &python_missing);

    let missing = apt_missing.len() + python_missing.len();
    if missing > 0 {
        return Err(format!("{missing} missing dependencies").into());
    }
    println!("all dependencies installed");
    Ok(())
}

fn report(kind: &str, wanted: usize, missing: &[&str]) {
    println!("{kind}: {}/{wanted} installed", wanted - missing.len());
    for name in missing {
        println!("  missing: {name}");
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
        prod: bool,
    },
    /// Sync system dependencies with monorepo.toml configuration
    Sync {
        #[arg(long, conflicts_with = "prod", required_unless_present = "prod")]
        dev: bool,

        #[arg(long)]
        prod: bool,
    },
}
