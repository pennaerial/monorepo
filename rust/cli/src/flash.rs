//! Writes the Pi base image to an SD card and names the Pi.

use crate::sh;
use std::{
    error::Error,
    fs,
    io::{self, Write},
    path::Path,
};

const RELEASE_URL: &str = "https://github.com/pennaerial/monorepo/releases/download/image-latest";
const USER_DATA: &str = include_str!("../../../image/user-data");
const NETWORK_CONFIG: &str = include_str!("../../../image/network-config");

/// Flashes `device` (e.g. /dev/sda) as air-<id>.
pub fn flash(id: u8, device: &str, image: Option<&Path>) -> Result<(), Box<dyn Error>> {
    if !(1..=99).contains(&id) {
        return Err("the Pi number has to be between 1 and 99".into());
    }
    let name = format!("air-{id:02}");

    // one .img.xz file, or the parts of one in order
    let image = match image {
        Some(path) => path.display().to_string(),
        None => download()?,
    };

    sh::run(
        None,
        &format!("lsblk -o NAME,SIZE,MODEL,MOUNTPOINTS {device}"),
    )?;
    print!("This erases {device} and writes {name} to it. Continue? [y/N] ");
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().read_line(&mut answer)?;
    if answer.trim() != "y" {
        return Err("cancelled".into());
    }

    // the desktop may have mounted the card's old partitions
    sh::run(None, &format!("sudo umount {device}?* 2>/dev/null || true"))?;
    sh::run(
        None,
        &format!("cat {image} | xz -dc | sudo dd of={device} bs=4M conv=fsync status=progress"),
    )?;

    // cloud-init reads these two files from the boot partition on first boot
    let dir = std::env::temp_dir();
    fs::write(dir.join("user-data"), USER_DATA.replace("@NAME@", &name))?;
    fs::write(
        dir.join("network-config"),
        NETWORK_CONFIG.replace("@ID@", &id.to_string()),
    )?;
    sh::run(
        None,
        &format!(
            "set -e; sudo blockdev --rereadpt {device}; sleep 2; \
             boot=$(lsblk -lnpo NAME {device} | sed -n 2p); mnt=$(mktemp -d); \
             sudo mount $boot $mnt; \
             sudo cp {0}/user-data {0}/network-config $mnt/; \
             sudo umount $mnt",
            dir.display()
        ),
    )?;

    println!("{device} is ready as {name} (mesh 10.10.0.{id}, ethernet 192.168.50.{id})");
    Ok(())
}

/// Downloads the latest image's parts into ~/.cache/pennair if they changed,
/// and returns their paths in order.
fn download() -> Result<String, Box<dyn Error>> {
    let cache = sh::output(None, "echo $HOME/.cache/pennair")?
        .trim()
        .to_string();
    // the .sha256 file lists the parts in order
    sh::run(
        None,
        &format!(
            "set -e; mkdir -p {cache}; cd {cache}; \
             curl -fsSL -o pennair-pi.img.xz.sha256 {RELEASE_URL}/pennair-pi.img.xz.sha256; \
             for part in $(awk '{{print $2}}' pennair-pi.img.xz.sha256); do \
                 curl -fL -z $part -o $part {RELEASE_URL}/$part; \
             done; \
             sha256sum -c pennair-pi.img.xz.sha256"
        ),
    )?;
    let parts = sh::output(
        None,
        &format!("cd {cache}; awk '{{print \"{cache}/\" $2}}' pennair-pi.img.xz.sha256"),
    )?;
    Ok(parts.split_whitespace().collect::<Vec<_>>().join(" "))
}
