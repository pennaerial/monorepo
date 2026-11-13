//! Push source to Pis and build it there, and pull edits made on a Pi back.

use crate::config::{MonorepoConfig, load_config};
use crate::{deps, sh};
use std::{
    env,
    error::Error,
    path::Path,
    time::{SystemTime, UNIX_EPOCH},
};

/// Where pushed source lives on a Pi, relative to the user's home.
const REMOTE_ROOT: &str = "monorepo";
const SERVICE: &str = "pennair-autonomy.service";
/// -R keeps paths relative to the monorepo root; -a keeps mtimes so colcon only rebuilds what changed.
const RSYNC: &str = "rsync -aR --delete --exclude=.git --exclude=__pycache__";

pub fn push(
    manifest: Option<&Path>,
    hosts: &[String],
    restart: bool,
) -> Result<(), Box<dyn Error>> {
    let config = load(manifest)?;
    let paths = config.deploy.paths.join(" ");
    let record = push_record(&paths)?;
    // only build pushed packages, even if the Pi has other source in ros_ws/src
    let packages: Vec<&str> = config
        .deploy
        .paths
        .iter()
        .filter_map(|p| p.strip_prefix("ros_ws/"))
        .collect();

    for h in hosts {
        println!("== {h}");
        deps::sync(&config, "prod", Some(h))?;

        // Pis have no RTC, and make needs sane timestamps
        let now = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
        sh::run(
            Some(h),
            &format!(
                "sudo -n date -s @{now} > /dev/null || \
                 echo 'warning: no passwordless sudo, so the clock was not set'"
            ),
        )?;

        sh::run(None, &format!("{RSYNC} {paths} {h}:{REMOTE_ROOT}/"))?;

        // save what was pushed, plus file hashes so `status` can spot edits made on the Pi
        sh::run(
            Some(h),
            &format!(
                "cd {REMOTE_ROOT} && mkdir -p .pennair && echo '{record}' > .pennair/push && \
                 find {paths} -type f -not -path '*/__pycache__/*' -print0 \
                 | xargs -0 sha256sum > .pennair/push.sha256"
            ),
        )?;

        sh::run(
            Some(h),
            &format!(
                "cd {REMOTE_ROOT}/ros_ws && . /opt/ros/jazzy/setup.sh && \
                 colcon build --base-paths {} --cmake-args -DBUILD_TESTING=OFF",
                packages.join(" ")
            ),
        )?;

        if restart {
            sh::run(Some(h), &format!("sudo -n systemctl restart {SERVICE}"))?;
        }
        println!("pushed to {h}");
    }
    Ok(())
}

pub fn pull(manifest: Option<&Path>, host: &str, force: bool) -> Result<(), Box<dyn Error>> {
    let config = load(manifest)?;
    let paths = config.deploy.paths.join(" ");

    // the pull overwrites these paths, so only git can get local edits back
    let changes = sh::output(None, &format!("git status --porcelain -- {paths}"))?;
    if !changes.is_empty() && !force {
        return Err(format!(
            "uncommitted changes in pushed paths; commit or stash them, or pass --force:\n{changes}"
        )
        .into());
    }

    let sources: Vec<String> = config
        .deploy
        .paths
        .iter()
        .map(|p| format!("{host}:{REMOTE_ROOT}/./{p}"))
        .collect();
    sh::run(None, &format!("{RSYNC} {} .", sources.join(" ")))?;

    println!("changes pulled from {host}:");
    sh::run(None, &format!("git status --short -- {paths}"))
}

pub fn status(hosts: &[String]) -> Result<(), Box<dyn Error>> {
    for h in hosts {
        println!("== {h}");
        // sha256sum lists pushed files that were edited or deleted on the Pi
        sh::run(
            Some(h),
            &format!(
                "cd {REMOTE_ROOT} && cat .pennair/push && \
                 (sha256sum --check --quiet .pennair/push.sha256 2>&1 && echo 'no edits since push' || true)"
            ),
        )?;
    }
    Ok(())
}

/// Loads the manifest and moves to the monorepo root, since deploy paths are relative to it.
fn load(manifest: Option<&Path>) -> Result<MonorepoConfig, Box<dyn Error>> {
    let (path, config) = load_config(manifest)?;
    env::set_current_dir(path.parent().unwrap())?;
    Ok(config)
}

/// Commit, author, and time of a push, saved on the Pi for `status`.
fn push_record(paths: &str) -> Result<String, Box<dyn Error>> {
    let commit = sh::output(None, "git rev-parse --short HEAD")?;
    let dirty = !sh::output(None, &format!("git status --porcelain -- {paths}"))?.is_empty();
    let user = sh::output(None, "git config user.name || whoami")?;
    let time = sh::output(None, "date -Is")?;
    Ok(format!(
        "commit: {}{}\npushed by: {}\npushed at: {}",
        commit.trim(),
        if dirty { " + uncommitted changes" } else { "" },
        user.trim(),
        time.trim()
    ))
}
