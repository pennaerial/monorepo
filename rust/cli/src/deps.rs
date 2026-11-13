use crate::config::{MonorepoConfig, load_config};
use crate::installed;
use clap::{Args, Subcommand};
use std::collections::BTreeSet;
use std::error::Error;
use std::path::Path;

pub fn run_deps(args: DepsArgs, manifest: Option<&Path>) -> Result<(), Box<dyn Error>> {
    match args.command {
        DepsCommand::Check { dev, host, .. } => {
            let (_, config) = load_config(manifest)?;
            check(&config, if dev { "dev" } else { "prod" }, host.as_deref())
        }
        DepsCommand::Sync { dev, .. } => {
            let m = if dev { "dev" } else { "prod" };
            println!("syncing {m} dependencies");
            Ok(())
        }
    }
}

/// Reports packages and libraries from the environment's groups that aren't installed.
/// Versions aren't checked yet.
pub fn check(
    config: &MonorepoConfig,
    env_name: &str,
    host: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    println!(
        "checking {env_name} dependencies on {}",
        host.unwrap_or("this machine")
    );
    let env = config
        .environments
        .get(env_name)
        .ok_or(format!("no [environments.{env_name}] in manifest"))?;

    let (mut packages, mut python, mut libraries) =
        (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
    for name in &env.groups {
        let group = config.groups.get(name).ok_or(format!(
            "environment '{env_name}' uses unknown group '{name}'"
        ))?;
        packages.extend(&group.packages);
        python.extend(&group.python_packages);
        libraries.extend(&group.libraries);
    }

    let apt_installed = installed::apt(host)?;
    let python_installed = installed::python(host)?;
    let libraries_installed = installed::libraries(host)?;

    let missing = report("apt", &packages, |p| apt_installed.contains(p))
        + report("python", &python, |r| {
            python_installed.contains(&installed::requirement_name(r))
        })
        + report("libraries", &libraries, |l| libraries_installed.contains(l));

    if missing > 0 {
        return Err(format!("{missing} missing dependencies").into());
    }
    println!("all dependencies installed");
    Ok(())
}

/// Prints how many of `wanted` are installed and returns the number missing.
fn report(kind: &str, wanted: &BTreeSet<&String>, installed: impl Fn(&str) -> bool) -> usize {
    let missing: Vec<_> = wanted.iter().filter(|w| !installed(w)).collect();
    println!(
        "{kind}: {}/{} installed",
        wanted.len() - missing.len(),
        wanted.len()
    );
    for name in &missing {
        println!("  missing: {name}");
    }
    missing.len()
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

        /// Check a remote machine over ssh, e.g. penn@air-03.local
        #[arg(long)]
        host: Option<String>,
    },
    /// Sync system dependencies with monorepo.toml configuration
    Sync {
        #[arg(long, conflicts_with = "prod", required_unless_present = "prod")]
        dev: bool,

        #[arg(long)]
        prod: bool,
    },
}
