use crate::config::{MonorepoConfig, load_config};
use crate::{installed, sh};
use clap::{Args, Subcommand};
use std::collections::BTreeSet;
use std::error::Error;
use std::path::Path;

pub fn run_deps(args: DepsArgs, manifest: Option<&Path>) -> Result<(), Box<dyn Error>> {
    let (_, config) = load_config(manifest)?;
    match args.command {
        DepsCommand::Check { dev, host, .. } => {
            check(&config, if dev { "dev" } else { "prod" }, host.as_deref())
        }
        DepsCommand::Sync { dev, host, .. } => {
            sync(&config, if dev { "dev" } else { "prod" }, host.as_deref())
        }
    }
}

/// Fails if anything the environment needs isn't installed.
pub fn check(
    config: &MonorepoConfig,
    env_name: &str,
    host: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let missing = find_missing(config, env_name, host)?;
    if missing.count() > 0 {
        return Err(format!("{} missing dependencies", missing.count()).into());
    }
    println!("all dependencies installed");
    Ok(())
}

/// Installs missing apt and python packages, then checks again.
/// Needs internet and sudo only when something is missing.
pub fn sync(
    config: &MonorepoConfig,
    env_name: &str,
    host: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let missing = find_missing(config, env_name, host)?;
    if missing.count() == 0 {
        println!("all dependencies installed");
        return Ok(());
    }
    if !missing.libraries.is_empty() {
        return Err(format!(
            "install these by hand, they're built from source: {}",
            missing.libraries.join(", ")
        )
        .into());
    }

    if !missing.apt.is_empty() {
        sh::run(
            host,
            &format!(
                "sudo apt-get update && sudo apt-get install -y {}",
                missing.apt.join(" ")
            ),
        )?;
    }
    if !missing.python.is_empty() {
        // pin numpy to the apt version, since pip would otherwise pull numpy 2, which breaks ROS
        let requirements: Vec<String> = missing.python.iter().map(|r| format!("'{r}'")).collect();
        sh::run(
            host,
            &format!(
                "python3 -m pip install --user --break-system-packages {} \
                 \"numpy==$(python3 -c 'import numpy; print(numpy.__version__)')\"",
                requirements.join(" ")
            ),
        )?;
    }
    check(config, env_name, host)
}

/// What the environment needs that isn't installed.
struct Missing {
    apt: Vec<String>,
    python: Vec<String>,
    libraries: Vec<String>,
}

impl Missing {
    fn count(&self) -> usize {
        self.apt.len() + self.python.len() + self.libraries.len()
    }
}

/// Compares the environment's groups against what's installed and prints what's missing.
fn find_missing(
    config: &MonorepoConfig,
    env_name: &str,
    host: Option<&str>,
) -> Result<Missing, Box<dyn Error>> {
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

    Ok(Missing {
        apt: report("apt", &packages, |p| apt_installed.contains(p)),
        python: report("python", &python, |r| {
            python_installed.contains(&installed::requirement_name(r))
        }),
        libraries: report("libraries", &libraries, |l| libraries_installed.contains(l)),
    })
}

/// Prints how many of `wanted` are installed and returns the missing ones.
fn report(kind: &str, wanted: &BTreeSet<&String>, installed: impl Fn(&str) -> bool) -> Vec<String> {
    let missing: Vec<String> = wanted
        .iter()
        .filter(|w| !installed(w))
        .map(|w| w.to_string())
        .collect();
    println!(
        "{kind}: {}/{} installed",
        wanted.len() - missing.len(),
        wanted.len()
    );
    for name in &missing {
        println!("  missing: {name}");
    }
    missing
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
    /// Install missing apt and python packages listed in monorepo.toml
    Sync {
        #[arg(long, conflicts_with = "prod", required_unless_present = "prod")]
        dev: bool,

        #[arg(long)]
        prod: bool,

        /// Sync a remote machine over ssh, e.g. penn@air-03.local
        #[arg(long)]
        host: Option<String>,
    },
}
