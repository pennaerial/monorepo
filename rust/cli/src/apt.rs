use crate::config::Deb;
use crate::sh;
use std::{collections::{BTreeMap, BTreeSet}, error::Error};

const LIST_INSTALLED_PACKAGES: &str =
    r"dpkg-query -W -f='${Package} ${Version} ${db:Status-Abbrev}\n'";

/// Fully installed APT packages and their versions.
pub fn installed_packages(
    host: Option<&str>,
) -> Result<BTreeMap<String, String>, Box<dyn Error>> {
    let out = sh::output(host, LIST_INSTALLED_PACKAGES)?;
    // "ii" means installed. Skip removed packages with leftover config files.
    Ok(out
        .lines()
        .filter_map(|line| {
            let mut fields = line.split(' ');
            let (name, version, status) = (fields.next()?, fields.next()?, fields.next()?);
            status
                .starts_with("ii")
                .then(|| (name.to_string(), version.to_string()))
        })
        .collect())
}

/// Returns required system package names that are not installed.
pub fn find_missing_system_packages(
    required: &BTreeSet<String>,
    installed: &BTreeMap<String, String>,
) -> BTreeSet<String> {
    required
        .iter()
        .filter(|name| !installed.contains_key(*name))
        .cloned()
        .collect()
}

/// Returns custom deb requirements that are missing or have an incompatible version.
pub fn find_unsatisfied_custom_debs(
    required: &[Deb],
    installed: &BTreeMap<String, String>,
) -> Vec<Deb> {
    required
        .iter()
        .filter(|deb| {
            !installed
                .get(&deb.package)
                .is_some_and(|version| version.starts_with(&deb.version))
        })
        .cloned()
        .collect()
}
