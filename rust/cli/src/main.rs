use clap::Parser;
use std::process::ExitCode;

mod apt;
mod cli;
mod config;
mod deps;
use cli::PennairCli;

fn main() -> ExitCode {
    let cli = PennairCli::parse();
    match cli.run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
