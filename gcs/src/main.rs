mod components;
mod ros;
mod views;

use dioxus::prelude::*;

use views::MissionControl;

const TAILWIND_CSS: Asset = asset!("/assets/tailwind.css");

fn main() {
    dioxus::launch(App);
}

#[component]
fn App() -> Element {
    println!("Hello from PennAiR!");

    rsx! {
        document::Link { rel: "stylesheet", href: TAILWIND_CSS }
        MissionControl {}
    }
}
