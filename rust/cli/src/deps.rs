use crate::config::{Deb, MonorepoConfig, load_config};
use crate::{apt, python, sh};
use clap::{Args, Subcommand};
use pep440_rs::Version;
use pep508_rs::{PackageName, Requirement};
use std::collections::{BTreeMap, BTreeSet};
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
    if !missing.apt.is_empty() || !missing.debs.is_empty() {
        // download each release tarball and check its .deb, then install everything with one apt call
        let mut cmd = String::from("set -e; cd $(mktemp -d); ");
        for deb in &missing.debs {
            let url = &deb.url;
            cmd += &format!("curl -fsSL {url} | tar -xz; sha256sum -c SHA256SUMS; ");
        }
        let debs = if missing.debs.is_empty() {
            ""
        } else {
            "./*.deb"
        };
        cmd += &format!(
            "sudo apt-get update && sudo apt-get install -y {} {debs}",
            missing.apt.iter().map(String::as_str).collect::<Vec<_>>().join(" ")
        );
        sh::run(host, &cmd)?;
    }
    if !missing.python.is_empty() {
        // pin numpy to the apt version. Without the pin, pip installs numpy 2, which breaks ROS.
        let requirements: Vec<String> = missing.python.iter().map(|r| format!("'{r}'")).collect();
        sh::run(
            host,
            // TODO: Fix this numpy workaround
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
    apt: BTreeSet<String>,
    python: BTreeSet<Requirement>,
    debs: Vec<Deb>,
}

impl Missing {
    fn count(&self) -> usize {
        self.apt.len() + self.python.len() + self.debs.len()
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

    let mut apt_required_packages = BTreeSet::new();
    let mut python_requirement_strings = BTreeSet::new();
    let mut debs = Vec::new();
    for name in &env.groups {
        let group = config.groups.get(name).ok_or(format!(
            "environment '{env_name}' uses unknown group '{name}'"
        ))?;
        apt_required_packages.extend(group.packages.iter().cloned());
        python_requirement_strings.extend(group.python_packages.iter().cloned());
        debs.extend(group.debs.iter().cloned());
    }

    let apt_installed = apt::installed_packages(host)?;
    let missing_system_apt = apt::find_missing_system_packages(&apt_required_packages, &apt_installed);
    let unsatisfied_debs = apt::find_unsatisfied_custom_debs(&debs, &apt_installed);

    // Validate requirements before querying system
    let python_required = python::parse_requirements(&python_requirement_strings)?;
    let python_installed = python::installed_packages(host)?;
    let python_unsatisfied = python::find_unsatisfied_requirements(&python_required, &python_installed)?;

    // show status reports
    report_system_apt(&apt_required_packages, &missing_system_apt);
    report_custom_debs(&debs, &apt_installed, &unsatisfied_debs);
    report_python(&python_required, &python_installed, &python_unsatisfied);

    Ok(Missing {
        apt: missing_system_apt,
        python: python_unsatisfied,
        debs: unsatisfied_debs,
    })
}

/// Reports required system APT packages that are not installed.
fn report_system_apt(required: &BTreeSet<String>, missing: &BTreeSet<String>) {
    println!(
        "apt: {}/{} installed",
        required.len() - missing.len(),
        required.len()
    );

    for name in missing {
        println!("  missing: {name}");
    }
}

/// Reports missing or incompatible Python requirements.
fn report_python(
    required: &BTreeSet<Requirement>,
    installed: &BTreeMap<PackageName, Version>,
    unsatisfied: &BTreeSet<Requirement>,
) {
    println!(
        "python: {}/{} satisfied",
        required.len() - unsatisfied.len(),
        required.len()
    );

    for requirement in unsatisfied {
        match installed.get(&requirement.name) {
            Some(version) => {
                println!("  incompatible: {requirement} (installed: {version})");
            }
            None => println!("  missing: {requirement}"),
        }
    }
}

/// Reports missing or incompatible custom deb requirements.
fn report_custom_debs(
    required: &[Deb],
    installed: &BTreeMap<String, String>,
    unsatisfied: &[Deb],
) {
    println!(
        "debs: {}/{} satisfied",
        required.len() - unsatisfied.len(),
        required.len()
    );
    for deb in unsatisfied {
        match installed.get(&deb.package) {
            Some(version) => println!(
                "  incompatible: {} {} (installed: {version})",
                deb.package, deb.version
            ),
            None => println!("  missing: {} {}", deb.package, deb.version),
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
    /// Check that everything monorepo.toml lists is installed
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
