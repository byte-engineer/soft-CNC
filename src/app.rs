//! The egui window: draws the controller's state and calls its methods.

mod pcb_setup;

use crate::controller::Controller;
use crate::pcb::{is_pcb_file, toolpath::Settings as PcbSettings};
use pcb_setup::{Choice, PcbSetup};
use crate::firmware::{BAUDS, FIRMWARES, SPINDLE_MAX, SPINDLE_STEP};
use crate::link::LogTag;
use crate::sim::{SIM_CIRQOID, SIM_GRBL};
use eframe::egui::{
    self, Align, Button, Color32, ComboBox, DragValue, FontId, Frame, Key, Layout, Margin, ProgressBar, RichText,
    ScrollArea, Stroke, TextEdit, TextStyle, Ui, Vec2,
};
use rfd::{MessageDialog, MessageLevel};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

const STOP_RED: Color32 = Color32::from_rgb(0xc6, 0x28, 0x28);
const GCODE_EXTENSIONS: [&str; 6] = ["gcode", "nc", "ngc", "tap", "cnc", "txt"];
const PCB_EXTENSIONS: [&str; 12] = ["gbr", "gtl", "gbl", "gko", "gm1", "gml", "ger", "pho", "drl", "xln", "exc", "drd"];
/// A jog pad button: (label, axis index, direction).
type JogButton = (&'static str, usize, f64);

const fn rgb(hex: u32) -> Color32 {
    Color32::from_rgb((hex >> 16) as u8, (hex >> 8) as u8, hex as u8)
}

/// "app" is the window background; "card" the sections drawn on it.
struct Palette {
    app: Color32,
    card: Color32,
    fg: Color32,
    muted: Color32,
    field: Color32,
    button: Color32,
    hover: Color32,
    border: Color32,
    select: Color32,
    accent: Color32,
    accent_fg: Color32,
    good: Color32,
    console_bg: Color32,
    console_fg: Color32,
    tx: Color32,
    err: Color32,
    info: Color32,
    ok: Color32,
}

const LIGHT: Palette = Palette {
    app: rgb(0xe4e7ec), card: rgb(0xffffff), fg: rgb(0x1f2328), muted: rgb(0x6b7280), field: rgb(0xf6f7f9),
    button: rgb(0xeceef2), hover: rgb(0xdde1e7), border: rgb(0xd0d5dd), select: rgb(0xd6e6fb),
    accent: rgb(0x1f6feb), accent_fg: rgb(0xffffff), good: rgb(0x1a7f37), console_bg: rgb(0xfafbfc),
    console_fg: rgb(0x2d333b), tx: rgb(0x1f5fbf), err: rgb(0xcf222e), info: rgb(0x1a7f37), ok: rgb(0x6a9a6a),
};

const DARK: Palette = Palette {
    app: rgb(0x141414), card: rgb(0x202124), fg: rgb(0xe6e6e6), muted: rgb(0x8b9099), field: rgb(0x2a2b2e),
    button: rgb(0x303134), hover: rgb(0x3c3d41), border: rgb(0x36373b), select: rgb(0x264f78),
    accent: rgb(0x2f81f7), accent_fg: rgb(0xffffff), good: rgb(0x57ab5a), console_bg: rgb(0x151618),
    console_fg: rgb(0xd4d4d4), tx: rgb(0x7fc7ff), err: rgb(0xff6b6b), info: rgb(0x9ccc65), ok: rgb(0x5f8f5f),
};

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Console,
    PcbJob,
}

pub struct App {
    c: Controller,
    ports: Vec<(String, String)>, // (device, label)
    dark: bool,
    tab: Tab,
    entry: String,
    /// The PCB setup dialog, while it is open.
    pcb: Option<PcbSetup>,
    /// Tool settings for PCB jobs, remembered between sessions.
    pcb_settings: PcbSettings,
    /// Where the theme and PCB settings are remembered (None in tests: nothing is read or written).
    settings_file: Option<PathBuf>,
}

fn settings_path() -> Option<PathBuf> {
    let home = std::env::var_os("USERPROFILE").or_else(|| std::env::var_os("HOME"))?;
    Some(PathBuf::from(home).join(".soft-cnc.json")) // remembers the theme and the PCB tool settings
}

/// (dark theme, PCB settings) from the settings file.
fn load_settings(file: &Option<PathBuf>) -> (bool, PcbSettings) {
    let saved = file
        .as_ref()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .unwrap_or_default();
    (saved["theme"] == "dark", pcb_setup::settings_from_json(&saved["pcb"]))
}

fn save_settings(file: &Option<PathBuf>, dark: bool, pcb: &PcbSettings) {
    if let Some(path) = file {
        let json = serde_json::json!({
            "theme": if dark { "dark" } else { "light" },
            "pcb": pcb_setup::settings_to_json(pcb),
        });
        let _ = std::fs::write(path, json.to_string());
    }
}

/// Use Windows' own fonts when available: Segoe UI for text, Consolas for numbers and the console,
/// and Segoe UI Symbol for arrows and symbols that egui's built-in fonts lack (↑ ↓ ⌂ ▾ ▶ ■).
fn install_fonts(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();
    let dir = std::env::var_os("WINDIR").map(PathBuf::from).unwrap_or_else(|| "C:\\Windows".into()).join("Fonts");
    let mut add = |name: &str, file: &str, families: &[(egui::FontFamily, bool)]| {
        let Ok(bytes) = std::fs::read(dir.join(file)) else { return };
        fonts.font_data.insert(name.into(), Arc::new(egui::FontData::from_owned(bytes)));
        for (family, first) in families {
            let list = fonts.families.entry(family.clone()).or_default();
            if *first { list.insert(0, name.into()) } else { list.push(name.into()) }
        }
    };
    use egui::FontFamily::{Monospace, Proportional};
    add("segoe", "segoeui.ttf", &[(Proportional, true)]);
    add("consolas", "consola.ttf", &[(Monospace, true)]);
    add("symbols", "seguisym.ttf", &[(Proportional, false), (Monospace, false)]);
    ctx.set_fonts(fonts);
}

fn error_box(title: &str, text: &str) {
    MessageDialog::new().set_level(MessageLevel::Error).set_title(title).set_description(text).show();
}

/// One G-code file, or the Gerber + drill files of a board (several at once).
fn pick_files() -> Option<Vec<PathBuf>> {
    let all: Vec<&str> = GCODE_EXTENSIONS.iter().chain(PCB_EXTENSIONS.iter()).copied().collect();
    rfd::FileDialog::new()
        .set_title("Load a G-code file, or a board's Gerber + drill files (select them together)")
        .add_filter("G-code and PCB files", &all)
        .add_filter("G-code", &GCODE_EXTENSIONS)
        .add_filter("Gerber and drill files", &PCB_EXTENSIONS)
        .add_filter("All files", &["*"])
        .pick_files()
}

impl App {
    /// `files`: opened at start, e.g. from the command line or Windows' "Open with".
    pub fn new(cc: &eframe::CreationContext<'_>, files: Vec<PathBuf>) -> Self {
        Self::build(&cc.egui_ctx, files, settings_path())
    }

    fn build(ctx: &egui::Context, files: Vec<PathBuf>, settings_file: Option<PathBuf>) -> Self {
        let repaint = ctx.clone();
        let waker: Arc<dyn Fn() + Send + Sync> = Arc::new(move || repaint.request_repaint());
        let (dark, pcb_settings) = load_settings(&settings_file);
        let mut app = Self {
            c: Controller::new(Some(waker)),
            ports: Vec::new(),
            dark,
            tab: Tab::Console,
            entry: String::new(),
            pcb: None,
            pcb_settings,
            settings_file,
        };
        app.refresh_ports();
        install_fonts(ctx);
        ctx.style_mut(|style| {
            style.text_styles.insert(TextStyle::Body, FontId::proportional(14.0));
            style.text_styles.insert(TextStyle::Button, FontId::proportional(14.0));
            style.text_styles.insert(TextStyle::Monospace, FontId::monospace(13.0));
            style.text_styles.insert(TextStyle::Small, FontId::proportional(12.0));
            style.spacing.item_spacing = Vec2::new(6.0, 6.0);
            style.spacing.button_padding = Vec2::new(8.0, 4.0);
        });
        app.apply_theme(ctx);
        app.open_paths(files);
        app
    }

    fn palette(&self) -> &'static Palette {
        if self.dark { &DARK } else { &LIGHT }
    }

    fn apply_theme(&self, ctx: &egui::Context) {
        let p = self.palette();
        let mut v = if self.dark { egui::Visuals::dark() } else { egui::Visuals::light() };
        v.panel_fill = p.app;
        v.window_fill = p.card;
        v.extreme_bg_color = p.field;
        v.faint_bg_color = p.field;
        v.override_text_color = Some(p.fg);
        v.selection.bg_fill = p.select;
        v.selection.stroke = Stroke::new(1.0_f32, p.fg);
        v.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, p.border);
        for (widget, fill) in [
            (&mut v.widgets.inactive, p.button),
            (&mut v.widgets.hovered, p.hover),
            (&mut v.widgets.active, p.hover),
            (&mut v.widgets.open, p.hover),
        ] {
            widget.bg_fill = fill;
            widget.weak_bg_fill = fill;
        }
        ctx.set_visuals(v);
    }

    fn refresh_ports(&mut self) {
        self.ports = serialport::available_ports()
            .unwrap_or_default()
            .into_iter()
            .map(|p| {
                let label = match &p.port_type {
                    serialport::SerialPortType::UsbPort(usb) => {
                        format!("{} — {}", p.port_name, usb.product.clone().unwrap_or_else(|| "USB serial".into()))
                    }
                    _ => p.port_name.clone(),
                };
                (p.port_name, label)
            })
            .collect();
        self.ports.push((SIM_CIRQOID.into(), format!("{SIM_CIRQOID} — Cirqoid-like")));
        self.ports.push((SIM_GRBL.into(), format!("{SIM_GRBL} — GRBL-like")));
        if !self.ports.iter().any(|(d, _)| *d == self.c.port) {
            self.c.port = self.ports[0].0.clone();
        }
    }

    // ---------- actions with dialogs ----------
    fn toggle_connect(&mut self) {
        if self.c.connected() {
            self.c.disconnect();
        } else if let Err(err) = self.c.connect() {
            error_box("Connect failed", &err);
        }
    }

    fn load(&mut self) {
        if let Some(paths) = pick_files() {
            self.open_paths(paths);
        }
    }

    /// From Load…, drag-and-drop or the command line. A G-code file is converted for the machine straight away;
    /// Gerber + drill files open the PCB setup.
    fn open_paths(&mut self, paths: Vec<PathBuf>) {
        if paths.is_empty() {
            return;
        }
        if self.c.job_running() {
            error_box("Load", "A job is running. Stop it before loading another file.");
            return;
        }
        self.tab = Tab::PcbJob;
        if paths.iter().any(|p| is_pcb_file(p)) {
            match PcbSetup::open(&paths, self.pcb_settings.clone()) {
                Ok(setup) => self.pcb = Some(setup),
                Err(err) => error_box("PCB files", &err),
            }
        } else if paths.len() > 1 {
            error_box("Load", "Pick one G-code file, or all the Gerber and drill files of one board.");
        } else if let Err(err) = self.c.load(&paths[0]) {
            error_box("Load", &err);
        }
    }

    /// Files dropped on the window are loaded like Load…; while dragging, say what will happen.
    fn drag_and_drop(&mut self, ctx: &egui::Context) {
        let hovering = ctx.input(|i| !i.raw.hovered_files.is_empty());
        if hovering {
            let p = self.palette();
            let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("drop")));
            let screen = ctx.screen_rect();
            painter.rect_filled(screen, 0.0, Color32::from_black_alpha(150));
            painter.text(screen.center(), egui::Align2::CENTER_CENTER,
                         "Drop to load: a G-code file, or a board's Gerber + drill files",
                         FontId::proportional(22.0), p.accent_fg);
        }
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().filter_map(|f| f.path.clone()).collect());
        if !dropped.is_empty() && self.pcb.is_none() {
            self.open_paths(dropped);
        }
    }

    fn pcb_dialog(&mut self, ctx: &egui::Context) {
        let palette = self.palette();
        let Some(setup) = &mut self.pcb else { return };
        match setup.show(ctx, palette) {
            Choice::Open => {}
            Choice::Cancel => self.pcb = None,
            Choice::Create => {
                let setup = self.pcb.take().unwrap();
                self.pcb_settings = setup.settings.clone();
                save_settings(&self.settings_file, self.dark, &self.pcb_settings);
                self.c.load_pcb_jobs(setup.name.clone(), setup.jobs(), &setup.notes());
                self.tab = Tab::PcbJob;
            }
        }
    }

    fn toggle_theme(&mut self, ctx: &egui::Context) {
        self.dark = !self.dark;
        save_settings(&self.settings_file, self.dark, &self.pcb_settings);
        self.apply_theme(ctx);
    }

    // ---------- keyboard ----------
    fn keyboard(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.c.estop();
        }
        if ctx.wants_keyboard_input() || !self.c.connected() || self.c.job_running() || self.pcb.is_some() {
            return; // typing in a box, or nothing to jog
        }
        let keys = [
            (Key::ArrowLeft, 0, -1.0),
            (Key::ArrowRight, 0, 1.0),
            (Key::ArrowUp, 1, 1.0),
            (Key::ArrowDown, 1, -1.0),
            (Key::PageUp, 2, 1.0),
            (Key::PageDown, 2, -1.0),
        ];
        let pressed: Vec<(usize, f64)> =
            ctx.input(|i| keys.iter().filter(|k| i.key_pressed(k.0)).map(|k| (k.1, k.2)).collect());
        for (axis, sign) in pressed {
            self.c.jog(axis, sign);
        }
    }

    // ---------- layout helpers ----------
    fn card(ui: &mut Ui, p: &Palette, title: &str, add: impl FnOnce(&mut Ui)) {
        Frame::new().fill(p.card).corner_radius(6.0).inner_margin(Margin::symmetric(12, 10)).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(title.to_uppercase()).size(11.0).strong().color(p.muted));
            ui.add_space(2.0);
            add(ui);
        });
        ui.add_space(8.0);
    }

    fn accent(p: &Palette, text: &str) -> Button<'static> {
        Button::new(RichText::new(text).strong().color(p.accent_fg)).fill(p.accent)
    }

    fn muted(p: &Palette, text: &str) -> RichText {
        RichText::new(text).color(p.muted).size(12.0)
    }

    // ---------- panels ----------
    fn top_bar(&mut self, ctx: &egui::Context) {
        let p = self.palette();
        let frame = Frame::new().fill(p.app).inner_margin(Margin { left: 10, right: 10, top: 10, bottom: 0 });
        egui::TopBottomPanel::top("top").frame(frame).show_separator_line(false).show(ctx, |ui| {
            Frame::new().fill(p.card).corner_radius(6.0).inner_margin(Margin::symmetric(12, 8)).show(ui, |ui| {
                ui.horizontal(|ui| {
                    let idle = !self.c.connected() && !self.c.detecting();
                    ui.add_enabled_ui(idle, |ui| {
                        ui.label("Port");
                        let current = self.ports.iter().find(|(d, _)| *d == self.c.port)
                            .map(|(_, l)| l.clone()).unwrap_or_else(|| self.c.port.clone());
                        ComboBox::from_id_salt("port").width(200.0).selected_text(current).show_ui(ui, |ui| {
                            for (device, label) in &self.ports {
                                ui.selectable_value(&mut self.c.port, device.clone(), label);
                            }
                        });
                        if ui.button("↻").on_hover_text("Refresh the port list").clicked() {
                            self.refresh_ports();
                        }
                        ui.add_space(2.0);
                        ui.label("Baud");
                        ui.add(TextEdit::singleline(&mut self.c.baud).desired_width(52.0));
                        ui.menu_button("▾", |ui| {
                            for baud in BAUDS {
                                if ui.button(baud.to_string()).clicked() {
                                    self.c.baud = baud.to_string();
                                    ui.close_menu();
                                }
                            }
                        });
                        ui.add_space(2.0);
                        ui.label("Firmware");
                        ComboBox::from_id_salt("fw").width(80.0).selected_text(self.c.fw.name).show_ui(ui, |ui| {
                            for fw in FIRMWARES {
                                if ui.selectable_label(self.c.fw.name == fw.name, fw.name).clicked() {
                                    self.c.set_firmware(fw.name);
                                }
                            }
                        });
                    });
                    ui.add_space(4.0);
                    let text = if self.c.connected() { "Disconnect" } else { "Connect" };
                    if ui.add_enabled(!self.c.detecting(), Self::accent(p, text).min_size(Vec2::new(96.0, 0.0)))
                        .clicked() {
                        self.toggle_connect();
                    }
                    let text = if self.c.detecting() { "Stop scan" } else { "Auto-detect" };
                    if ui.button(text).on_hover_text("Try every baud rate with info queries only (no motion)")
                        .clicked() {
                        self.c.auto_detect();
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let theme = if self.dark { "☀ Light" } else { "☾ Dark" };
                        if ui.button(theme).clicked() {
                            self.toggle_theme(ui.ctx());
                        }
                        ui.add_space(6.0);
                        let (dot, text, color) = if self.c.connected() {
                            ("●", "● Connected", p.good)
                        } else {
                            ("○", "○ Disconnected", p.muted)
                        };
                        // a narrow window shows just the dot, so it never draws over the buttons
                        let label = if ui.available_width() > 120.0 { text } else { dot };
                        ui.label(RichText::new(label).strong().color(color)).on_hover_text(&text[4..]);
                    });
                });
            });
        });
    }

    fn left_panel(&mut self, ctx: &egui::Context) {
        let p = self.palette();
        let frame = Frame::new().fill(p.app).inner_margin(Margin { left: 10, right: 0, top: 10, bottom: 10 });
        egui::SidePanel::left("left").resizable(false).exact_width(300.0).frame(frame).show_separator_line(false)
            .show(ctx, |ui| {
                let manual = self.c.connected() && !self.c.job_running();
                ui.with_layout(Layout::bottom_up(Align::Min), |ui| {
                    // E-STOP first, so it is never pushed out of view
                    let stop = Button::new(RichText::new("E-STOP   (Esc)").size(20.0).strong().color(Color32::WHITE))
                        .fill(STOP_RED).min_size(Vec2::new(ui.available_width(), 52.0));
                    if ui.add_enabled(self.c.connected(), stop).clicked() {
                        self.c.estop();
                    }
                    ui.add_space(8.0);
                    ui.with_layout(Layout::top_down(Align::Min), |ui| {
                        // scrolls only if the window is too short for all the cards
                        ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                            self.position_card(ui, p);
                            self.jog_card(ui, p, manual);
                            self.machine_card(ui, p, manual);
                            self.spindle_card(ui, p, manual);
                        });
                    });
                });
            });
    }

    fn position_card(&mut self, ui: &mut Ui, p: &Palette) {
        let (values, note) = self.c.readout();
        Self::card(ui, p, "Position · mm", |ui| {
            for (axis, value) in ["X", "Y", "Z"].iter().zip(values) {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(*axis).size(18.0).strong().color(p.muted));
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.label(RichText::new(value).monospace().size(26.0).strong());
                    });
                });
            }
            ui.label(Self::muted(p, &note));
        });
    }

    fn jog_card(&mut self, ui: &mut Ui, p: &Palette, manual: bool) {
        Self::card(ui, p, "Jog", |ui| {
            let size = Vec2::new(52.0, 30.0);
            let mut clicked = None;
            egui::Grid::new("jog").spacing([4.0, 4.0]).show(ui, |ui| {
                let layout: [[Option<JogButton>; 5]; 3] = [
                    [None, Some(("Y+", 1, 1.0)), None, None, Some(("Z↑", 2, 1.0))],
                    [Some(("X−", 0, -1.0)), None, Some(("X+", 0, 1.0)), None, None],
                    [None, Some(("Y−", 1, -1.0)), None, None, Some(("Z↓", 2, -1.0))],
                ];
                for row in layout {
                    for (column, cell) in row.into_iter().enumerate() {
                        match cell {
                            Some((text, axis, sign)) => {
                                if ui.add_enabled(manual, Button::new(text).min_size(size)).clicked() {
                                    clicked = Some((axis, sign));
                                }
                            }
                            None if column == 3 => { ui.allocate_space(Vec2::new(6.0, size.y)); }
                            None => { ui.allocate_space(size); }
                        }
                    }
                    ui.end_row();
                }
            });
            if let Some((axis, sign)) = clicked {
                self.c.jog(axis, sign);
            }
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Step");
                for value in ["0.1", "1", "5", "10", "50"] {
                    ui.selectable_value(&mut self.c.step, value.to_string(), value);
                }
                ui.label(Self::muted(p, "mm"));
            });
            ui.horizontal(|ui| {
                ui.label("Feed");
                ui.add(TextEdit::singleline(&mut self.c.feed).desired_width(56.0));
                ui.label(Self::muted(p, "mm/min"));
            });
            ui.label(Self::muted(p, "Keys  ← → ↑ ↓  PgUp/PgDn   ·   Esc stops"));
        });
    }

    fn machine_card(&mut self, ui: &mut Ui, p: &Palette, manual: bool) {
        let fw = self.c.fw;
        Self::card(ui, p, "Machine", |ui| {
            // Only the commands this firmware has (the Cirqoid has none of these extras).
            let mut extras: Vec<(&str, bool)> = Vec::new(); // (name, works mid-job)
            if fw.has(&fw.unlock) {
                extras.push(("Unlock", false));
            }
            if fw.has(&fw.position) {
                extras.push(("Position", false));
            }
            if fw.hold.is_some() {
                extras.push(("Feed hold", true));
            }
            if fw.resume.is_some() {
                extras.push(("Resume", true));
            }
            let width = (ui.available_width() - 6.0) / 2.0;
            egui::Grid::new("machine").spacing([6.0, 6.0]).show(ui, |ui| {
                if ui.add_enabled(manual, Self::accent(p, "⌂  Home").min_size(Vec2::new(width, 28.0))).clicked() {
                    self.c.home();
                }
                if ui.add_enabled(manual, Button::new("Zero XYZ (G92)").min_size(Vec2::new(width, 28.0))).clicked() {
                    self.c.send_line("G92 X0 Y0 Z0");
                }
                ui.end_row();
                for (index, (name, mid_job)) in extras.into_iter().enumerate() {
                    let enabled = if mid_job { self.c.connected() } else { manual };
                    if ui.add_enabled(enabled, Button::new(name).min_size(Vec2::new(width, 28.0))).clicked() {
                        match name {
                            "Unlock" => self.c.fw_command("unlock", &fw.unlock),
                            "Position" => self.c.fw_command("position", &fw.position),
                            "Feed hold" => self.c.realtime_command("hold", fw.hold.unwrap_or_default()),
                            _ => self.c.realtime_command("resume", fw.resume.unwrap_or_default()),
                        }
                    }
                    if index % 2 == 1 {
                        ui.end_row();
                    }
                }
            });
        });
    }

    fn spindle_card(&mut self, ui: &mut Ui, p: &Palette, manual: bool) {
        Self::card(ui, p, "Spindle", |ui| {
            ui.add_enabled_ui(manual, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Speed S");
                    if ui.small_button("−").clicked() {
                        self.c.speed = (self.c.speed - SPINDLE_STEP).max(SPINDLE_STEP);
                    }
                    ui.add(DragValue::new(&mut self.c.speed).range(SPINDLE_STEP..=SPINDLE_MAX).speed(10.0));
                    if ui.small_button("+").clicked() {
                        self.c.speed = (self.c.speed + SPINDLE_STEP).min(SPINDLE_MAX);
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let color = if self.c.spindle_on { p.good } else { p.muted };
                        ui.label(RichText::new(&self.c.spindle_text).strong().color(color));
                    });
                });
                let width = (ui.available_width() - 12.0) / 3.0;
                ui.horizontal(|ui| {
                    if ui.add(Button::new("Start M3").min_size(Vec2::new(width, 26.0))).clicked() {
                        self.c.spindle_start();
                    }
                    if ui.add(Button::new("Set speed").min_size(Vec2::new(width, 26.0))).clicked() {
                        self.c.spindle_set();
                    }
                    if ui.add(Button::new("Stop M5").min_size(Vec2::new(width, 26.0))).clicked() {
                        self.c.spindle_stop();
                    }
                });
            });
        });
    }

    fn central(&mut self, ctx: &egui::Context) {
        let p = self.palette();
        let frame = Frame::new().fill(p.app).inner_margin(Margin { left: 8, right: 10, top: 10, bottom: 10 });
        egui::CentralPanel::default().frame(frame).show(ctx, |ui| {
            ui.horizontal(|ui| {
                for (tab, text) in [(Tab::Console, "Console"), (Tab::PcbJob, "PCB job")] {
                    let selected = self.tab == tab;
                    let label = RichText::new(text).strong().color(if selected { p.fg } else { p.muted });
                    let button = Button::new(label).fill(if selected { p.card } else { p.app })
                        .stroke(Stroke::NONE).min_size(Vec2::new(90.0, 30.0));
                    if ui.add(button).clicked() {
                        self.tab = tab;
                    }
                }
            });
            Frame::new().fill(p.card).corner_radius(6.0).inner_margin(Margin::same(12)).show(ui, |ui| {
                ui.set_min_size(ui.available_size());
                match self.tab {
                    Tab::Console => self.console(ui, p),
                    Tab::PcbJob => self.pcb_job(ui, p),
                }
            });
        });
    }

    fn console(&mut self, ui: &mut Ui, p: &Palette) {
        let row_height = ui.text_style_height(&TextStyle::Monospace);
        let log_height = ui.available_height() - 40.0;
        Frame::new().fill(p.console_bg).corner_radius(4.0).inner_margin(Margin::same(8)).show(ui, |ui| {
            ScrollArea::both().auto_shrink([false, false]).stick_to_bottom(true).max_height(log_height - 16.0)
                .show_rows(ui, row_height, self.c.log.len(), |ui, rows| {
                    for line in &self.c.log[rows] {
                        let color = match line.tag {
                            LogTag::Tx => p.tx,
                            LogTag::Rx => p.console_fg,
                            LogTag::Err => p.err,
                            LogTag::Info => p.info,
                            LogTag::Ok => p.ok,
                        };
                        ui.horizontal(|ui| {
                            ui.add(egui::Label::new(RichText::new(&line.text).monospace().color(color)).extend());
                            if line.ok {
                                ui.label(RichText::new("✓ ok").monospace().color(p.ok));
                            }
                        });
                    }
                });
        });
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let manual = self.c.connected() && !self.c.job_running();
            let entry = ui.add_enabled(manual, TextEdit::singleline(&mut self.entry).font(TextStyle::Monospace)
                .hint_text("G-code command, Enter to send").desired_width(ui.available_width() - 130.0));
            if entry.has_focus() {
                let (up, down) = ui.input(|i| (i.key_pressed(Key::ArrowUp), i.key_pressed(Key::ArrowDown)));
                if let Some(text) = if up { self.c.history_step(-1) } else if down { self.c.history_step(1) } else { None } {
                    self.entry = text;
                }
            }
            let enter = entry.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
            if ui.add_enabled(manual, Button::new("Send")).clicked() || enter {
                let text = std::mem::take(&mut self.entry);
                self.c.send_entry(&text);
                entry.request_focus();
            }
            if ui.button("Clear").clicked() {
                self.c.clear_log();
            }
        });
    }

    fn pcb_job(&mut self, ui: &mut Ui, p: &Palette) {
        let manual = self.c.connected() && !self.c.job_running();
        let step = |ui: &mut Ui, title: &str, help: &str| {
            ui.label(RichText::new(title).size(11.0).strong().color(p.muted));
            if !help.is_empty() {
                ui.label(Self::muted(p, help));
            }
            ui.add_space(4.0);
        };

        step(ui, "1 · BOARD POSITION", "Home, jog the bit over the board's corner, then lower it until it just \
              touches the copper. Redo the surface Z after every bit change.");
        egui::Grid::new("board").spacing([8.0, 6.0]).show(ui, |ui| {
            ui.label("Origin  X");
            ui.add(TextEdit::singleline(&mut self.c.origin_x).desired_width(70.0));
            ui.label("Y");
            ui.add(TextEdit::singleline(&mut self.c.origin_y).desired_width(70.0));
            if ui.add_enabled(manual, Button::new("Set origin here")).clicked() {
                self.c.set_origin_here();
            }
            ui.end_row();
            ui.label("Surface  Z");
            ui.add(TextEdit::singleline(&mut self.c.surface).desired_width(70.0));
            ui.label("");
            ui.label("");
            if ui.add_enabled(manual, Button::new("Set surface Z here")).clicked() {
                self.c.set_surface_here();
            }
            ui.end_row();
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.c.air_test, "Air test: lift the whole job by");
            ui.add(TextEdit::singleline(&mut self.c.air_offset).desired_width(36.0));
            ui.label(Self::muted(p, "mm (dry run)"));
        });

        ui.add_space(6.0);
        ui.separator();
        step(ui, "2 · PROGRAM", "Load a G-code file, or a board's Gerber + drill files (select them together) \
              to make isolation, drilling and cutout jobs. Everything is turned into G-code the Cirqoid understands \
              and placed at the board position above.");
        ui.horizontal(|ui| {
            if ui.add_enabled(!self.c.job_running(), Button::new("Load…")).clicked() {
                self.load();
            }
            ui.add_space(6.0);
            ui.label(RichText::new(&self.c.file_label).strong());
        });
        let jobs = self.c.program.as_ref().filter(|p| p.jobs.len() > 1).map(|p| (p.jobs.clone(), p.loaded));
        if let Some((jobs, loaded)) = jobs {
            ui.add_space(4.0);
            ui.label(RichText::new("JOBS — load one, run it, then change the bit, set the surface Z and load the next")
                .size(11.0).strong().color(p.muted));
            egui::Grid::new("jobs").num_columns(3).spacing([8.0, 4.0]).show(ui, |ui| {
                for (index, job) in jobs.iter().enumerate() {
                    let current = loaded == Some(index);
                    let label = format!("{}{}. {}", if current { "▶ " } else { "" }, index + 1, job.title);
                    ui.label(if current { RichText::new(label).strong().color(p.accent) } else { RichText::new(label) });
                    let load = Button::new(if current { "Reload" } else { "Load" }).min_size(Vec2::new(64.0, 0.0));
                    if ui.add_enabled(!self.c.job_running(), load).clicked() {
                        if let Err(err) = self.c.load_job(index) {
                            error_box("Load job", &err);
                        }
                    }
                    ui.label(Self::muted(p, &job.hint));
                    ui.end_row();
                }
            });
        }

        ui.add_space(6.0);
        ui.separator();
        step(ui, "3 · RUN", "");
        ui.horizontal(|ui| {
            let running = self.c.job_running();
            let can_start = self.c.connected() && !running && !self.c.job_lines.is_empty();
            if ui.add_enabled(can_start, Self::accent(p, "▶  Start").min_size(Vec2::new(90.0, 28.0))).clicked() {
                self.c.start_job();
            }
            let pause = if self.c.paused() { "Resume" } else { "Pause" };
            if ui.add_enabled(running, Button::new(pause).min_size(Vec2::new(70.0, 28.0))).clicked() {
                self.c.pause_job();
            }
            if ui.add_enabled(running, Button::new("■  Stop").min_size(Vec2::new(70.0, 28.0))).clicked() {
                self.c.stop_job();
            }
            let (done, total) = (self.c.job_done, self.c.job_total);
            let fraction = if total > 0 { done as f32 / total as f32 } else { 0.0 };
            let text = if total > 0 { format!("{done}/{total}") } else { String::new() };
            ui.add(ProgressBar::new(fraction).text(text).fill(p.accent).desired_height(20.0));
        });

        ui.add_space(14.0);
        step(ui, "LAST MESSAGE", "");
        match &self.c.last_msg {
            Some((text, tag)) => {
                let color = if *tag == LogTag::Err { p.err } else { p.info };
                ui.label(RichText::new(text).color(color));
            }
            None => {
                ui.label(Self::muted(p, "—"));
            }
        }
    }

}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.frame(ctx);
    }
}

impl App {
    /// One frame of the whole window.
    fn frame(&mut self, ctx: &egui::Context) {
        self.c.poll();
        if self.c.connected() && self.c.fw.name == "GRBL" {
            ctx.request_repaint_after(Duration::from_millis(250)); // keeps the status poll going
        }
        self.keyboard(ctx);
        self.top_bar(ctx);
        self.left_panel(ctx);
        self.central(ctx);
        self.pcb_dialog(ctx);
        self.drag_and_drop(ctx);
    }
}

/// The window, driven headless: fake screen sizes, key presses and dropped files; no settings file is touched.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SIM_CIRQOID;
    use egui::{Event, Pos2, RawInput, Rect};
    use std::time::Instant;

    /// A small laptop screen, like the one this app is used on.
    const SMALL: Vec2 = Vec2::new(1000.0, 620.0);

    fn new_app(ctx: &egui::Context, files: Vec<PathBuf>) -> App {
        App::build(ctx, files, None)
    }

    fn frame(ctx: &egui::Context, app: &mut App, size: Vec2, events: Vec<Event>, dropped: Vec<PathBuf>) {
        let input = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, size)),
            events,
            dropped_files: dropped.into_iter().map(|path| egui::DroppedFile { path: Some(path), ..Default::default() }).collect(),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| app.frame(ctx));
    }

    fn key(key: Key) -> Event {
        Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }
    }

    /// Keep drawing frames until `done` (or fail after `secs`).
    fn wait(ctx: &egui::Context, app: &mut App, secs: u64, what: &str, done: impl Fn(&App) -> bool) {
        let end = Instant::now() + Duration::from_secs(secs);
        while !done(app) {
            assert!(Instant::now() < end, "timed out waiting for {what}");
            frame(ctx, app, SMALL, Vec::new(), Vec::new());
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn logged(app: &App, text: &str) -> bool {
        app.c.log.iter().any(|l| l.text.contains(text))
    }

    fn demo_board() -> Vec<PathBuf> {
        let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/pcb"));
        ["demo-F_Cu.gbr", "demo-B_Cu.gbr", "demo-Edge_Cuts.gbr", "demo.drl"].map(|f| dir.join(f)).to_vec()
    }

    fn homed(ctx: &egui::Context) -> App {
        let mut app = new_app(ctx, Vec::new());
        app.c.set_firmware("Cirqoid");
        app.c.port = SIM_CIRQOID.into();
        app.c.connect().unwrap();
        app.c.home();
        wait(ctx, &mut app, 5, "homing", |a| a.c.readout().1.starts_with("homed"));
        app
    }

    #[test]
    fn every_screen_draws_in_both_themes_at_several_sizes() {
        let ctx = egui::Context::default();
        let mut app = new_app(&ctx, Vec::new());
        for size in [SMALL, Vec2::new(900.0, 600.0), Vec2::new(1920.0, 1040.0)] {
            for dark in [false, true] {
                app.dark = dark;
                app.apply_theme(&ctx);
                for tab in [Tab::Console, Tab::PcbJob] {
                    app.tab = tab;
                    frame(&ctx, &mut app, size, Vec::new(), Vec::new());
                }
            }
        }
    }

    #[test]
    fn the_pcb_dialog_fits_a_small_screen_and_esc_closes_it() {
        let ctx = egui::Context::default();
        let mut app = new_app(&ctx, demo_board()); // files from the command line open the dialog
        assert!(app.pcb.is_some());
        // everything the dialog draws, not just its frame, must be on screen: small and very short screens
        for size in [SMALL, Vec2::new(900.0, 520.0)] {
            for _ in 0..3 {
                frame(&ctx, &mut app, size, Vec::new(), Vec::new());
            }
            let drawn = app.pcb.as_ref().unwrap().drawn.expect("the dialog was drawn");
            let screen = Rect::from_min_size(Pos2::ZERO, size);
            assert!(screen.contains_rect(drawn), "dialog contents {drawn:?} must fit the screen {screen:?}");
        }
        frame(&ctx, &mut app, SMALL, vec![key(Key::Escape)], Vec::new());
        assert!(app.pcb.is_none(), "Esc closes the dialog");
    }

    #[test]
    fn dropped_files_are_loaded() {
        let ctx = egui::Context::default();
        let mut app = new_app(&ctx, Vec::new());
        frame(&ctx, &mut app, SMALL, Vec::new(), demo_board());
        assert!(app.pcb.is_some(), "Gerber + drill files open the PCB setup");
        assert!(app.tab == Tab::PcbJob);

        let ctx = egui::Context::default();
        let mut app = new_app(&ctx, Vec::new());
        let gcode = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/cirqoid_square.gcode"));
        frame(&ctx, &mut app, SMALL, Vec::new(), vec![gcode]);
        assert!(app.c.file_label.starts_with("cirqoid_square.gcode"), "{}", app.c.file_label);
    }

    #[test]
    fn creating_jobs_from_the_dialog_fills_the_jobs_list() {
        let ctx = egui::Context::default();
        let mut app = new_app(&ctx, demo_board());
        let setup = app.pcb.take().unwrap();
        app.c.load_pcb_jobs(setup.name.clone(), setup.jobs(), &setup.notes());
        let program = app.c.program.as_ref().unwrap();
        assert_eq!(program.name, "demo");
        assert!(program.jobs.len() >= 4, "isolation, drills, cutout");
        app.tab = Tab::PcbJob;
        frame(&ctx, &mut app, SMALL, Vec::new(), Vec::new()); // the Jobs list draws
    }

    #[test]
    fn arrow_keys_jog_but_not_while_the_dialog_is_open() {
        let ctx = egui::Context::default();
        let mut app = homed(&ctx);
        frame(&ctx, &mut app, SMALL, vec![key(Key::ArrowRight)], Vec::new());
        wait(&ctx, &mut app, 3, "X jog", |a| logged(a, "G0 X1.0 F500.0"));
        frame(&ctx, &mut app, SMALL, vec![key(Key::PageDown)], Vec::new());
        wait(&ctx, &mut app, 3, "Z jog", |a| logged(a, "G0 Z-1.0 F300.0"));

        app.open_paths(demo_board());
        assert!(app.pcb.is_some());
        let before = app.c.log.len();
        frame(&ctx, &mut app, SMALL, vec![key(Key::ArrowUp)], Vec::new());
        std::thread::sleep(Duration::from_millis(200));
        frame(&ctx, &mut app, SMALL, Vec::new(), Vec::new());
        assert!(!app.c.log[before..].iter().any(|l| l.text.starts_with("G0 Y")), "no jog behind the dialog");
    }

    #[test]
    fn escape_is_the_stop_key() {
        let ctx = egui::Context::default();
        let mut app = homed(&ctx);
        frame(&ctx, &mut app, SMALL, vec![key(Key::Escape)], Vec::new());
        assert!(logged(&app, "*** STOP"));
    }
}
