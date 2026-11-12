use dioxus::prelude::*;

#[component]
pub fn Button(label: String, onclick: EventHandler<MouseEvent>) -> Element {
    rsx! {
        button {
            class: "\
                inline-flex items-center justify-center \
                rounded-md \
                bg-zinc-900 px-4 py-2 \
                text-sm font-medium text-white \
                shadow-sm \
                transition-colors \
                hover:bg-zinc-800 \
                focus-visible:outline-none \
                focus-visible:ring-2 focus-visible:ring-zinc-950 \
                focus-visible:ring-offset-2 \
                disabled:pointer-events-none disabled:opacity-50 \
                dark:bg-zinc-50 dark:text-zinc-900 \
                dark:hover:bg-zinc-200",
            onclick,
            "{label}"
        }
    }
}
