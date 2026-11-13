use std::{
    error::Error,
    process::{Command, Stdio},
};

/// Runs a shell command locally, or on `host` over ssh, and returns its stdout.
pub fn output(host: Option<&str>, cmd: &str) -> Result<String, Box<dyn Error>> {
    let out = command(host, cmd, false)
        .stderr(Stdio::inherit())
        .output()?;
    if !out.status.success() {
        return Err(format!("`{cmd}` failed ({})", out.status).into());
    }
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}

/// Like `output`, but prints straight to the terminal, and lets sudo ask for a password.
pub fn run(host: Option<&str>, cmd: &str) -> Result<(), Box<dyn Error>> {
    let status = command(host, cmd, true).status()?;
    if !status.success() {
        return Err(format!("`{cmd}` failed ({status})").into());
    }
    Ok(())
}

fn command(host: Option<&str>, cmd: &str, tty: bool) -> Command {
    let mut c = Command::new(if host.is_some() { "ssh" } else { "sh" });
    match host {
        Some(host) if tty => c.args(["-t", host, cmd]),
        Some(host) => c.args([host, cmd]),
        None => c.args(["-c", cmd]),
    };
    c
}
