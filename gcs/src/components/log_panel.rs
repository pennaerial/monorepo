use crate::hooks::{use_ros_node, use_subscription};
use chrono::{DateTime, Local};
use dioxus::prelude::*;
use ros_interfaces::rcl_interfaces::msg::Log;
use std::collections::VecDeque;

/// how many logs a panel can hold before it starts dropping
const MAX_LOG_MESSAGES_DISPLAY: usize = 10_000;

struct LogEntry {
    id: u64, // want stable indexing/ids to avoid re-rendering
    log: Log,
}

impl LogEntry {
    fn display_timestamp(&self) -> String {
        DateTime::from_timestamp(i64::from(self.log.stamp.sec), self.log.stamp.nanosec)
            .map(|timestamp| {
                timestamp
                    .with_timezone(&Local)
                    .format("%H:%M:%S%.3f")
                    .to_string()
            })
            .unwrap_or_else(|| format!("{}.{:09}", self.log.stamp.sec, self.log.stamp.nanosec))
    }

    fn level_color_class(&self) -> &'static str {
        match self.log.level {
            Log::DEBUG => "text-sky-300",
            Log::INFO => "text-zinc-100",
            Log::WARN => "text-amber-300",
            Log::ERROR => "text-red-400",
            Log::FATAL => "text-red-400 font-semibold",
            _ => "text-zinc-200",
        }
    }

    fn display_line(&self) -> String {
        let level = match self.log.level {
            Log::DEBUG => "DEBUG",
            Log::INFO => "INFO",
            Log::WARN => "WARN",
            Log::ERROR => "ERROR",
            Log::FATAL => "FATAL",
            _ => "UNKNOWN",
        };

        format!(
            "[{level}] [{}] [{}]: {}",
            self.display_timestamp(),
            self.log.name,
            self.log.msg
        )
    }
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
    let mut logs = use_signal_sync(LogHistory::new);

    use_future(move || {
        let mut sub = rosout_sub.clone();
        async move {
            let res = sub
                .subscribe(move |msg| {
                    logs.write().add_log(msg);
                })
                .await;

            if let Err(error) = res {
                error!("Failed to subscribe to /rosout: {error:?}");
            }
        }
    });

    rsx! {
        div { class: "flex h-96 min-h-32 min-w-64 resize flex-col overflow-hidden pb-3 rounded-md border border-zinc-800 bg-[#111827] font-mono text-xs leading-5 text-zinc-200",
            h2 { class: "shrink-0 border-b border-zinc-600 px-4 py-2 font-medium text-zinc-100",
                "/rosout"
            }
            div { class: "min-h-0 flex-1 overflow-auto p-4",
                if logs.read().logs_ref().is_empty() {
                    p { class: "text-zinc-500", "Waiting for log messages" }
                }
                for entry in logs.read().logs_ref().iter() {
                    p {
                        key: "{entry.id}",
                        class: "whitespace-pre-wrap break-words {entry.level_color_class()}",
                        "{entry.display_line()}"
                    }
                }
            }
        }
    }
}
