// No console window behind the app in release builds.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Soft-CNC — G-code control")
            .with_inner_size([1040.0, 720.0])
            .with_min_inner_size([900.0, 600.0])
            .with_maximized(true), // fills the space above the taskbar on small screens
        ..Default::default()
    };
    // files given on the command line (or by "Open with") are loaded at start
    let files: Vec<std::path::PathBuf> = std::env::args_os().skip(1).map(Into::into).collect();
    eframe::run_native("Soft-CNC", options, Box::new(|cc| Ok(Box::new(soft_cnc::app::App::new(cc, files)))))
}
