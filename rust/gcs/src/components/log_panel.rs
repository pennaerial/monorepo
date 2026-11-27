use crate::hooks::{use_subscription, UseSubscription};
use crate::ros::NodeHandle;
use chrono::{DateTime, Local};
use dioxus::prelude::*;
use ros_interfaces::rcl_interfaces::msg::Log;
use std::collections::VecDeque;

/// how many logs a panel can hold before it starts dropping
const MAX_LOG_MESSAGES_DISPLAY: usize = 10_000;

struct LogEntry {
    id: u64, // want stable indexing/ids to avoid re-rendering
    line: String,
    level_class: &'static str,
}

impl LogEntry {
    fn display_timestamp(log: &Log) -> String {
        DateTime::from_timestamp(i64::from(log.stamp.sec), log.stamp.nanosec)
            .map(|timestamp| {
                timestamp
                    .with_timezone(&Local)
                    .format("%H:%M:%S%.3f")
                    .to_string()
            })
            .unwrap_or_else(|| format!("{}.{:09}", log.stamp.sec, log.stamp.nanosec))
    }

    fn level_color_class(log: &Log) -> &'static str {
        match log.level {
            Log::DEBUG => "text-sky-300",
            Log::INFO => "text-zinc-100",
            Log::WARN => "text-amber-300",
            Log::ERROR => "text-red-400",
            Log::FATAL => "text-red-400 font-semibold",
            _ => "text-zinc-200",
        }
    }

    fn display_line(log: &Log) -> String {
        let level = match log.level {
            Log::DEBUG => "DEBUG",
            Log::INFO => "INFO",
            Log::WARN => "WARN",
            Log::ERROR => "ERROR",
            Log::FATAL => "FATAL",
            _ => "UNKNOWN",
        };

        format!(
            "[{level}] [{}] [{}]: {}",
            Self::display_timestamp(log),
            log.name,
            log.msg
        )
    }
}

pub(crate) struct LogHistory {
    logs: VecDeque<LogEntry>,
    next_id: u64,
    cap: usize,
}

impl LogHistory {
    pub(crate) fn new() -> Self {
        LogHistory {
            logs: VecDeque::new(),
            next_id: 0,
            cap: MAX_LOG_MESSAGES_DISPLAY,
        }
    }

    pub(crate) fn add_log(&mut self, log: Log) {
        if self.logs.len() == self.cap {
            self.logs.pop_front();
        }
        self.logs.push_back(LogEntry {
            id: self.next_id,
            line: LogEntry::display_line(&log),
            level_class: LogEntry::level_color_class(&log),
        });
        self.next_id += 1;
    }

    fn logs_ref(&self) -> &VecDeque<LogEntry> {
        &self.logs
    }
}

#[component]
pub fn LogPanel() -> Element {
    let logs = use_context::<SyncSignal<LogHistory>>();

    let mut scroll_top = use_signal(|| 0_usize);
    let mut viewport_height = use_signal(|| 0_usize);

    let scroll_top_px = scroll_top();
    let viewport_height_px = viewport_height();

    const ROW_HEIGHT_PX: usize = 20;
    const OVERSCAN_ROWS: usize = 10;

    let history = logs.read();
    let entries = history.logs_ref();
    let len = entries.len();

    let first_visible = scroll_top_px / ROW_HEIGHT_PX;
    let visible_end = scroll_top_px
        .saturating_add(viewport_height_px)
        .div_ceil(ROW_HEIGHT_PX);

    let start = first_visible.saturating_sub(OVERSCAN_ROWS).min(len);
    let end = visible_end.saturating_add(OVERSCAN_ROWS).clamp(start, len);
    let top_px = start * ROW_HEIGHT_PX;
    let bottom_px = (len - end) * ROW_HEIGHT_PX;

    rsx! {

        div { class: "flex h-96 min-h-32 min-w-64 resize flex-col overflow-hidden pb-3 rounded-md border border-zinc-800 bg-[#111827] font-mono text-xs leading-5 text-zinc-200",
            h2 { class: "shrink-0 border-b border-zinc-600 px-4 py-2 font-medium text-zinc-100",
                "/rosout"
            }

            div {
                class: "min-h-0 flex-1 overflow-auto px-4",
                onresize: move |event| {
                    if let Ok(size) = event.get_content_box_size() {
                        viewport_height.set(size.height.ceil() as usize);
                    }
                },
                onscroll: move |event| {
                    scroll_top.set(event.scroll_top().max(0.0).floor() as usize);
                    viewport_height.set(event.client_height().max(0) as usize);
                },

                if len == 0 {
                    p { class: "text-zinc-500", "Waiting for log messages" }
                }

                div { style: "height: {top_px}px;" }

                for entry in entries.range(start..end) {
                    p {
                        key: "{entry.id}",
                        class: "truncate {entry.level_class}",
                        style: "height: {ROW_HEIGHT_PX}px; line-height: {ROW_HEIGHT_PX}px; margin: 0;",
                        title: "{entry.line}",
                        {entry.line.as_str()}
                    }
                }

                div { style: "height: {bottom_px}px;" }

            }

        }
    }
}
