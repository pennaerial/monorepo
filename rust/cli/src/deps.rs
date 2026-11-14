use crate::config::{Deb, MonorepoConfig, load_config};
use crate::{installed, sh};
use clap::{Args, Subcommand};
use std::collections::{BTreeSet, HashMap};
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
        for url in &missing.debs {
            cmd += &format!("curl -fsSL {url} | tar -xz; sha256sum -c SHA256SUMS; ");
        }
        let debs = if missing.debs.is_empty() {
            ""
        } else {
            "./*.deb"
        };
        cmd += &format!(
            "sudo apt-get update && sudo apt-get install -y {} {debs}",
            missing.apt.join(" ")
        );
        sh::run(host, &cmd)?;
    }
    if !missing.python.is_empty() {
        // pip installs for the current user, or system-wide when run as root (the image build).
        // pin numpy to the apt version. Without the pin, pip installs numpy 2, which breaks ROS.
        let requirements: Vec<String> = missing.python.iter().map(|r| format!("'{r}'")).collect();
        sh::run(
            host,
            &format!(
                "python3 -m pip install --break-system-packages {} \
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
    /// release URLs
    debs: Vec<String>,
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

    let (mut packages, mut python, mut debs) = (BTreeSet::new(), BTreeSet::new(), Vec::new());
    for name in &env.groups {
        let group = config.groups.get(name).ok_or(format!(
            "environment '{env_name}' uses unknown group '{name}'"
        ))?;
        packages.extend(&group.packages);
        python.extend(&group.python_packages);
        debs.extend(&group.debs);
    }

    let apt_installed = installed::apt(host)?;
    let python_installed = installed::python(host)?;

    Ok(Missing {
        apt: report("apt", &packages, |p| apt_installed.contains_key(p)),
        python: report("python", &python, |r| {
            python_installed.contains(&installed::requirement_name(r))
        }),
        debs: report_debs(&debs, &apt_installed),
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

/// Prints how many debs are installed at the right version and returns the URLs of the rest.
fn report_debs(wanted: &[&Deb], installed: &HashMap<String, String>) -> Vec<String> {
    let missing: Vec<&&Deb> = wanted
        .iter()
        .filter(|d| {
            !installed
                .get(&d.package)
                .is_some_and(|v| v.starts_with(&d.version))
        })
        .collect();
    println!(
        "debs: {}/{} installed",
        wanted.len() - missing.len(),
        wanted.len()
    );
    for d in &missing {
        let have = installed
            .get(&d.package)
            .map_or("not installed", String::as_str);
        println!("  missing: {} {} (have {have})", d.package, d.version);
    }
    missing.iter().map(|d| d.url.clone()).collect()
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
