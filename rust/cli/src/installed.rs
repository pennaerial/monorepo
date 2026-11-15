//! What's installed on this machine, or on `host` over ssh.

use crate::sh;
use std::{
    collections::{HashMap, HashSet},
    error::Error,
};

/// Fully installed apt packages and their versions.
pub fn apt(host: Option<&str>) -> Result<HashMap<String, String>, Box<dyn Error>> {
    let out = sh::output(
        host,
        r"dpkg-query -W -f='${Package} ${Version} ${db:Status-Abbrev}\n'",
    )?;
    // "ii" means installed. Removed packages that left config files behind show "rc" and are skipped.
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

/// Normalized names of python packages visible to python3.
pub fn python(host: Option<&str>) -> Result<HashSet<String>, Box<dyn Error>> {
    let out = sh::output(
        host,
        r#"python3 -c 'import importlib.metadata as m; print(*(d.metadata["Name"] for d in m.distributions()), sep="\n")'"#,
    )?;
    Ok(out.lines().map(normalize).collect())
}

/// Package name from a requirement like "pydantic>=2,<3".
pub fn requirement_name(requirement: &str) -> String {
    let end = requirement
        .find(|c: char| "<>=!~;[ (".contains(c))
        .unwrap_or(requirement.len());
    normalize(&requirement[..end])
}

/// PEP 503 normalization, so "Pupil_Apriltags" matches "pupil-apriltags".
fn normalize(name: &str) -> String {
    let mut out = String::new();
    for c in name.trim().chars() {
        if "-_.".contains(c) {
            if !out.ends_with('-') {
                out.push('-');
            }
        } else {
            out.push(c.to_ascii_lowercase());
        }
    }
    out
}
