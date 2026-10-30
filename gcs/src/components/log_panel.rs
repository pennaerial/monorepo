use crate::{
    components::Button,
    hooks::{use_ros_node, use_subscription},
};
use dioxus::prelude::*;
use ros_interfaces::rcl_interfaces::msg::Log;
use std::collections::VecDeque;

/// how many logs a panel can hold before it starts dropping
const MAX_LOG_MESSAGES_DISPLAY: usize = 10_000;

struct LogEntry {
    id: u64, // want stable indexing/ids to avoid re-rendering
    log: Log,
}

struct LogHistory {
    logs: VecDeque<LogEntry>,
    next_id: u64,
    cap: usize,
}

impl LogHistory {
    fn new() -> Self {
        LogHistory {
            logs: VecDeque::new(),
            next_id: 0,
            cap: MAX_LOG_MESSAGES_DISPLAY,
        }
    }

    fn add_log(&mut self, log: Log) {
        if self.logs.len() == self.cap {
            self.logs.pop_front();
        }
        self.logs.push_back(LogEntry {
            id: self.next_id,
            log,
        });
        self.next_id += 1;
    }

    fn logs_ref(&self) -> &VecDeque<LogEntry> {
        &self.logs
    }
}

#[component]
pub fn LogPanel() -> Element {
    let node = use_ros_node();
    let rosout_sub = use_subscription::<Log>(node.read().clone(), "/rosout", 10);

    // let mut log = use_signal_sync(|| None::<Log>);
    let mut logs = use_signal_sync(LogHistory::new);

    rsx! {
        Button {
            label: "see logs",
            onclick: move |_| {
                let mut sub = rosout_sub.clone();
                async move {
                    let _ = sub
                        .subscribe(move |msg| {
                            logs.write().add_log(msg);
                        })
                        .await;
                }
            },
        }
        div {
            if logs.read().logs_ref().is_empty() {
                p { "Waiting for log messages" }
            }
            for entry in logs.read().logs_ref().iter() {
                p { key: "{entry.id}", "[{entry.log.name}] {entry.log.msg} [{entry.id}]" }
            }
        }
        // if let Some(log) = log.read().as_ref() {
        //     p { "{log.msg}" }
        // } else {
        //     p { "waiting for rosout messages" }
        // }
    }
}
