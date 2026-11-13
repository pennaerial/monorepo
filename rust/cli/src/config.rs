use clap::{Args, Subcommand};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::{
    env,
    error::Error,
    fs,
    path::{Path, PathBuf},
};

pub fn run_config(args: ConfigArgs, manifest: Option<&Path>) -> Result<(), Box<dyn Error>> {
    match args.command {
        ConfigCommand::Check => check_config(manifest),
    }
}

#[derive(Args, Debug)]
pub struct ConfigArgs {
    #[command(subcommand)]
    pub command: ConfigCommand,
}

#[derive(Subcommand, Debug)]
pub enum ConfigCommand {
    /// Check that monorepo.toml parses and its [deploy] paths exist
    Check,
}

/// Fails if monorepo.toml doesn't parse or a [deploy] path doesn't exist.
pub fn check_config(manifest: Option<&Path>) -> Result<(), Box<dyn Error>> {
    let (path, config) = load_config(manifest)?;

    // fails if a package was moved or renamed without updating [deploy]
    let root = path.parent().unwrap();
    let missing: Vec<&str> = config
        .deploy
        .paths
        .iter()
        .filter(|p| !root.join(p).exists())
        .map(String::as_str)
        .collect();
    if !missing.is_empty() {
        return Err(format!("[deploy] paths not found: {}", missing.join(", ")).into());
    }

    println!("{} loaded successfully", path.display());
    Ok(())
}

/// Loads the manifest from `--manifest`, or `$PENNAIR_MONOREPO_PATH/monorepo.toml` if not given.
pub fn load_config(manifest: Option<&Path>) -> Result<(PathBuf, MonorepoConfig), Box<dyn Error>> {
    let path = match manifest {
        Some(p) => p.to_path_buf(),
        None => {
            let root = env::var("PENNAIR_MONOREPO_PATH")
                .map_err(|_| "PENNAIR_MONOREPO_PATH is not set; pass --manifest")?;
            Path::new(&root).join("monorepo.toml")
        }
    };
    // make it absolute, so push and pull can cd to the manifest's directory
    let path =
        fs::canonicalize(&path).map_err(|e| format!("failed to read {}: {e}", path.display()))?;
    let contents = fs::read_to_string(&path)?;
    let config = toml::from_str(&contents)
        .map_err(|e| format!("failed to parse {}: {e}", path.display()))?;
    Ok((path, config))
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
    /// Shared libraries installed outside apt, e.g. "libpigpiod_if2.so"
    #[serde(default)]
    pub libraries: Vec<String>,
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

/// Source that `pennair push` copies to Pis, from `[deploy]`.
#[derive(Debug, Deserialize)]
pub struct Deploy {
    /// Paths relative to the monorepo root
    pub paths: Vec<String>,
}

/// The complete `monorepo.toml` manifest.
#[derive(Debug, Deserialize)]
pub struct MonorepoConfig {
    pub target: Target,
    pub groups: BTreeMap<String, DependencyGroup>,
    pub environments: BTreeMap<String, Environment>,
    pub apt: Apt,
    pub deploy: Deploy,
}
