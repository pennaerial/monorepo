use clap::{Args, Subcommand};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::{env, error::Error, fs, path::Path};
use toml;

pub fn run_config(args: ConfigArgs) -> Result<(), Box<dyn Error>> {
    match args.command {
        ConfigCommand::Check => check_config(),
    }
}

#[derive(Args, Debug)]
pub struct ConfigArgs {
    #[command(subcommand)]
    pub command: ConfigCommand,
}

#[derive(Subcommand, Debug)]
pub enum ConfigCommand {
    /// Validates monorepo.toml manifest validation
    Check,
}

/// checks if monorepo.toml file validates correctly
pub fn check_config() -> Result<(), Box<dyn Error>> {
    let monorepo_path_str = env::var("PENNAIR_MONOREPO_PATH")
        .inspect_err(|_| eprintln!("PENNAIR_MONOREPO_PATH is not set!"))?;
    let toml_path = Path::new(&monorepo_path_str).join("monorepo.toml");

    if !toml_path.is_file() {
        return Err("monorepo.toml not found".into());
    }

    // try toml validation
    let contents = fs::read_to_string(toml_path)?;
    let _: MonorepoConfig = toml::from_str(&contents)?;

    println!("monorepo.toml loaded successfully");
    Ok(())
}

/// Target Ubuntu release and supported architectures from `[target]`.
#[derive(Debug, Deserialize)]
pub struct Target {
    pub ubuntu: String,
    pub architectures: Vec<String>,
}

/// Package and repository declarations from a `[groups.<name>]` table.
#[derive(Debug, Deserialize)]
pub struct DependencyGroup {
    pub repositories: Option<Vec<String>>,
    pub packages: Vec<String>,
    pub python_packages: Vec<String>,
}

/// Platform constraints and dependency selection from `[environments.<name>]`.
#[derive(Debug, Deserialize)]
pub struct Environment {
    pub os: Vec<String>,
    pub architectures: Vec<String>,
    pub groups: Vec<String>,
    pub repository_policy: String,
}

/// An architecture-specific entry from `[[apt.repositories.<name>.sources]]`.
#[derive(Debug, Deserialize)]
pub struct AptSource {
    pub architectures: Vec<String>,
    pub uri: String,
    pub suites: Vec<String>,
}

/// Shared settings and sources from `[apt.repositories.<name>]`.
#[derive(Debug, Deserialize)]
pub struct AptRepository {
    pub prod_snapshot: String,
    pub os: String,
    pub signed_by: String,
    pub components: Vec<String>,
    pub sources: Vec<AptSource>,
}

/// APT configuration from `[apt]` and its nested repository tables.
#[derive(Debug, Deserialize)]
pub struct Apt {
    pub repositories: BTreeMap<String, AptRepository>,
}

/// The complete `monorepo.toml` manifest.
#[derive(Debug, Deserialize)]
pub struct MonorepoConfig {
    pub target: Target,
    pub groups: BTreeMap<String, DependencyGroup>,
    pub environments: BTreeMap<String, Environment>,
    pub apt: Apt,
}
