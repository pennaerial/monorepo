//! What's installed on this machine, or on `host` over ssh.

use crate::sh;
use std::{collections::HashSet, error::Error};

/// Names of fully installed apt packages.
pub fn apt(host: Option<&str>) -> Result<HashSet<String>, Box<dyn Error>> {
    let out = sh::output(host, r"dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n'")?;
    // "ii" means installed; skips removed packages that left config files behind
    Ok(out
        .lines()
        .filter_map(|line| line.split_once(' '))
        .filter(|(_, status)| status.starts_with("ii"))
        .map(|(name, _)| name.to_string())
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

/// Shared libraries known to the dynamic linker, e.g. "libfoo.so" and "libfoo.so.2.4".
pub fn libraries(host: Option<&str>) -> Result<HashSet<String>, Box<dyn Error>> {
    let out = sh::output(host, "ldconfig -p")?;
    // lines look like "\tlibfoo.so.2.4 (libc6,AArch64) => /usr/lib/libfoo.so.2.4"
    Ok(out
        .lines()
        .skip(1)
        .filter_map(|line| line.split_whitespace().next())
        .map(String::from)
        .collect())
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
