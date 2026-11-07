use std::{error::Error, process::Command};


/// runs sudo apt-get update
pub fn apt_get_update() -> Result<(), Box<dyn Error>>{
    let update_status = Command::new("sudo")
        .args(["apt-get", "update"])
        .status()?;

    if !update_status.success() {
        return Err(format!("apt-get update failed: {update_status}").into());
    }

    Ok(())
}
