//! Shares this laptop's internet with Pis over Ethernet and the mesh. Linux only.

use crate::sh;
use clap::ValueEnum;
use std::error::Error;

/// The Ethernet and mesh networks. Pis send internet traffic to .254 on each.
const NETWORKS: [&str; 2] = ["192.168.50.0/24", "10.10.0.0/24"];

#[derive(Clone, Debug, ValueEnum)]
pub enum State {
    On,
    Off,
}

pub fn gateway(state: State) -> Result<(), Box<dyn Error>> {
    match state {
        State::On => {
            sh::run(None, "sudo sysctl -qw net.ipv4.ip_forward=1")?;
            for net in NETWORKS {
                for rule in rules(net) {
                    // -C checks whether the rule exists, so running `on` twice adds it once
                    sh::run(
                        None,
                        &format!("sudo iptables -C {rule} 2>/dev/null || sudo iptables -A {rule}"),
                    )?;
                }
            }
        }
        State::Off => {
            for net in NETWORKS {
                for rule in rules(net) {
                    sh::run(
                        None,
                        &format!("sudo iptables -D {rule} 2>/dev/null || true"),
                    )?;
                }
            }
        }
    }
    Ok(())
}

/// NAT traffic from `net`, and let it through the firewall in both directions.
fn rules(net: &str) -> [String; 3] {
    [
        format!("POSTROUTING -t nat -s {net} ! -d {net} -j MASQUERADE"),
        format!("FORWARD -s {net} -j ACCEPT"),
        format!("FORWARD -d {net} -j ACCEPT"),
    ]
}
