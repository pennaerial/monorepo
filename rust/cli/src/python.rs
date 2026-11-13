use std::{collections::HashSet, error::Error, process::Command};

const LIST_DISTRIBUTIONS: &str = "import importlib.metadata as m
for d in m.distributions():
    print(d.metadata['Name'])";

/// returns the normalized names of all python packages visible to python3
pub fn installed_packages() -> Result<HashSet<String>, Box<dyn Error>> {
    let output = Command::new("python3")
        .args(["-c", LIST_DISTRIBUTIONS])
        .output()
        .map_err(|e| format!("failed to run python3: {e}"))?;

    if !output.status.success() {
        return Err(format!("listing python packages failed: {}", output.status).into());
    }

    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(normalize)
        .collect())
}

/// gets the package name from a requirement like "pydantic>=2,<3"
pub fn requirement_name(requirement: &str) -> String {
    let end = requirement
        .find(|c: char| "<>=!~;[ (".contains(c))
        .unwrap_or(requirement.len());
    normalize(&requirement[..end])
}

/// PEP 503 name normalization, so "Pupil_Apriltags" matches "pupil-apriltags"
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
