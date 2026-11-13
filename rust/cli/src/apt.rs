use std::{collections::HashSet, error::Error, process::Command};

/// runs sudo apt-get update
pub fn apt_get_update() -> Result<(), Box<dyn Error>> {
    let update_status = Command::new("sudo").args(["apt-get", "update"]).status()?;

    if !update_status.success() {
        return Err(format!("apt-get update failed: {update_status}").into());
    }

    Ok(())
}

/// returns the names of all fully installed apt packages
pub fn installed_packages() -> Result<HashSet<String>, Box<dyn Error>> {
    let output = Command::new("dpkg-query")
        .args(["-W", "-f=${Package} ${db:Status-Abbrev}\n"])
        .output()
        .map_err(|e| format!("failed to run dpkg-query: {e}"))?;

    if !output.status.success() {
        return Err(format!("dpkg-query failed: {}", output.status).into());
    }

    // "ii" means installed; skips removed packages that left config files behind
    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| line.split_once(' '))
        .filter(|(_, status)| status.starts_with("ii"))
        .map(|(name, _)| name.to_string())
        .collect())
}
