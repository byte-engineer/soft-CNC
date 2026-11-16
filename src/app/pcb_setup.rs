//! The "PCB setup" dialog: Gerber + drill files → board preview, tool settings and the job list.

use super::Palette;
use crate::gcode::fmt;
use crate::pcb::toolpath::{generate, Job, JobKind, Settings};
use crate::pcb::{geom::rings, load_board, Board, Layer};
use eframe::egui::{self, Button, Color32, DragValue, Pos2, RichText, ScrollArea, Sense, Stroke, Ui, Vec2};
use geo::Coord;
use std::path::PathBuf;

const COPPER: Color32 = Color32::from_rgb(0xd0, 0x8a, 0x3c);
const BOTTOM_COPPER: Color32 = Color32::from_rgb(0x4f, 0x8f, 0xd8);

fn job_color(kind: JobKind) -> Color32 {
    match kind {
        JobKind::Isolation => Color32::from_rgb(0x3f, 0xb9, 0x50),
        JobKind::Drill => Color32::from_rgb(0xe5, 0x48, 0x48),
        JobKind::Cutout => Color32::from_rgb(0xf0, 0x9a, 0x2a),
        JobKind::Alignment => Color32::from_rgb(0xb0, 0x6a, 0xe8),
    }
}

pub(super) struct PcbSetup {
    pub name: String,
    board: Board,
    pub settings: Settings,
    generated: Settings,
    jobs: Vec<Job>,
    notes: Vec<String>,
    hovered: Option<usize>,
    /// Where the dialog's contents ended up on the last frame (checked by tests: all must be on screen).
    pub drawn: Option<egui::Rect>,
}

/// What the user chose in the dialog this frame.
pub(super) enum Choice {
    Open,
    Create,
    Cancel,
}

/// "demo-F_Cu.gbr", "demo-B_Cu.gbr", … → "demo" (KiCad names files <project>-<layer>).
fn project_name(paths: &[PathBuf]) -> String {
    let stem = paths[0].file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    match stem.rsplit_once('-') {
        Some((project, _)) if paths.len() > 1 => project.to_string(),
        _ => stem,
    }
}

impl PcbSetup {
    pub fn open(paths: &[PathBuf], settings: Settings) -> Result<Self, String> {
        let board = load_board(paths)?;
        let mut setup = Self {
            name: project_name(paths),
            board,
            generated: settings.clone(),
            settings,
            jobs: Vec::new(),
            notes: Vec::new(),
            hovered: None,
            drawn: None,
        };
        setup.regenerate();
        Ok(setup)
    }

    fn regenerate(&mut self) {
        let (jobs, notes) = generate(&self.board, &self.settings);
        self.jobs = jobs;
        self.notes = notes;
        self.generated = self.settings.clone();
    }

    pub fn jobs(&self) -> &[Job] {
        &self.jobs
    }

    pub fn notes(&self) -> Vec<String> {
        self.board.notes.iter().chain(&self.notes).cloned().collect()
    }

    pub fn show(&mut self, ctx: &egui::Context, p: &Palette) -> Choice {
        // Regenerate once a value stops changing (not on every frame of a drag).
        if self.settings != self.generated && !ctx.input(|i| i.pointer.any_down()) {
            self.regenerate();
        }
        let mut choice = Choice::Open;
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            return Choice::Cancel;
        }
        // Fit the screen: the whole dialog, including its buttons, must always be visible.
        let screen = ctx.screen_rect();
        let body_height = (screen.height() - 150.0).max(180.0);
        egui::Modal::new(egui::Id::new("pcb_setup")).show(ctx, |ui| {
            ui.set_width((screen.width() - 60.0).clamp(560.0, 1040.0));
            // Title and the buttons first, so they can never end up below the screen.
            ui.horizontal(|ui| {
                ui.heading(format!("PCB: {}", self.name));
                ui.label(
                    RichText::new(format!("board {} × {} mm", fmt(self.board.width), fmt(self.board.height)))
                        .color(p.muted),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Cancel").on_hover_text("Esc").clicked() {
                        choice = Choice::Cancel;
                    }
                    let create = Button::new(RichText::new("Create jobs").strong().color(p.accent_fg)).fill(p.accent);
                    if ui.add_enabled(!self.jobs.is_empty(), create).clicked() {
                        if self.settings != self.generated {
                            self.regenerate();
                        }
                        choice = Choice::Create;
                    }
                });
            });
            ui.label(
                RichText::new("X0 Y0 is the board's lower-left corner; Z0 is the copper surface.").color(p.muted).size(12.0),
            );
            ui.separator();
            ui.horizontal_top(|ui| {
                ui.vertical(|ui| {
                    ui.set_width(320.0);
                    ScrollArea::vertical().id_salt("pcb_settings").max_height(body_height).show(ui, |ui| {
                        self.files(ui, p);
                        self.settings_ui(ui, p);
                    });
                });
                ui.separator();
                ui.vertical(|ui| {
                    let list_height = ((self.jobs.len() as f32) * 22.0 + 26.0).min(body_height * 0.4);
                    let preview_height = (body_height - list_height - 8.0).max(120.0);
                    self.preview(ui, p, Vec2::new(ui.available_width(), preview_height));
                    ui.add_space(4.0);
                    ScrollArea::vertical().id_salt("pcb_jobs").max_height(list_height).show(ui, |ui| {
                        self.job_list(ui, p);
                    });
                });
            });
            self.drawn = Some(ui.min_rect());
        });
        choice
    }

    fn files(&self, ui: &mut Ui, p: &Palette) {
        ui.label(RichText::new("FILES").size(11.0).strong().color(p.muted));
        for file in &self.board.files {
            let color = match file.layer {
                Layer::CopperTop => COPPER,
                Layer::CopperBottom => BOTTOM_COPPER,
                Layer::Ignored => p.muted,
                _ => p.fg,
            };
            ui.horizontal(|ui| {
                ui.label(RichText::new(file.layer.label()).strong().color(color));
                ui.label(RichText::new(&file.name).color(p.muted).size(12.0)).on_hover_text(&file.reason);
            });
        }
        let notes = self.notes();
        if !notes.is_empty() {
            ui.add_space(4.0);
            for note in notes {
                ui.label(RichText::new(format!("• {note}")).color(p.muted).size(12.0));
            }
        }
        ui.add_space(6.0);
    }

    fn settings_ui(&mut self, ui: &mut Ui, p: &Palette) {
        let s = &mut self.settings;
        let section = |ui: &mut Ui, title: &str| {
            ui.add_space(6.0);
            ui.label(RichText::new(title).size(11.0).strong().color(p.muted));
        };
        fn row(ui: &mut Ui, label: &str, value: &mut f64, range: std::ops::RangeInclusive<f64>, step: f64, unit: &str) {
            ui.label(label);
            ui.add(DragValue::new(value).range(range).speed(step).max_decimals(3).suffix(unit));
            ui.end_row();
        }
        section(ui, "ISOLATION (V-BIT)");
        egui::Grid::new("iso").num_columns(2).spacing([10.0, 4.0]).show(ui, |ui| {
            row(ui, "Cut width", &mut s.iso_tool, 0.05..=2.0, 0.01, " mm");
            ui.label("Passes");
            ui.add(DragValue::new(&mut s.iso_passes).range(1..=10));
            ui.end_row();
            // shown in %, stored as a fraction; only written back when edited, so rounding can't change it
            let mut overlap = s.iso_overlap * 100.0;
            ui.label("Overlap");
            if ui.add(DragValue::new(&mut overlap).range(0.0..=90.0).speed(1.0).max_decimals(1).suffix(" %")).changed() {
                s.iso_overlap = overlap / 100.0;
            }
            ui.end_row();
            row(ui, "Depth", &mut s.iso_depth, 0.01..=1.0, 0.01, " mm");
            row(ui, "Feed", &mut s.iso_feed, 10.0..=1200.0, 5.0, " mm/min");
        });
        section(ui, "DRILLING");
        egui::Grid::new("drill").num_columns(2).spacing([10.0, 4.0]).show(ui, |ui| {
            row(ui, "Depth", &mut s.drill_depth, 0.2..=5.0, 0.05, " mm");
            row(ui, "Plunge feed", &mut s.drill_feed, 5.0..=300.0, 5.0, " mm/min");
        });
        section(ui, "CUTOUT (END MILL)");
        egui::Grid::new("cut").num_columns(2).spacing([10.0, 4.0]).show(ui, |ui| {
            row(ui, "Tool diameter", &mut s.cut_tool, 0.3..=3.2, 0.05, " mm");
            row(ui, "Depth", &mut s.cut_depth, 0.2..=5.0, 0.05, " mm");
            row(ui, "Depth per pass", &mut s.cut_step, 0.1..=2.0, 0.05, " mm");
            row(ui, "Feed", &mut s.cut_feed, 10.0..=1200.0, 5.0, " mm/min");
            ui.label("Tabs");
            ui.add(DragValue::new(&mut s.tabs).range(0..=12));
            ui.end_row();
            row(ui, "Tab width", &mut s.tab_width, 0.5..=10.0, 0.1, " mm");
            row(ui, "Tab height", &mut s.tab_height, 0.0..=2.0, 0.05, " mm");
        });
        section(ui, "GENERAL");
        egui::Grid::new("general").num_columns(2).spacing([10.0, 4.0]).show(ui, |ui| {
            row(ui, "Travel height", &mut s.travel_z, 0.5..=10.0, 0.1, " mm");
            row(ui, "Plunge feed", &mut s.plunge_feed, 5.0..=300.0, 5.0, " mm/min");
            ui.label("Spindle speed S");
            ui.add(DragValue::new(&mut s.spindle).range(50..=1400).speed(10.0));
            ui.end_row();
        });
        section(ui, "DOUBLE-SIDED");
        let both = self.board.top.is_some() && self.board.bottom.is_some();
        ui.add_enabled(both, egui::Checkbox::new(&mut s.double_sided, "Mirror the bottom layer, add alignment holes"))
            .on_disabled_hover_text("Needs both a top and a bottom copper layer");
        if s.double_sided {
            egui::Grid::new("double").num_columns(2).spacing([10.0, 4.0]).show(ui, |ui| {
                row(ui, "Alignment hole", &mut s.align_diameter, 0.5..=3.2, 0.1, " mm");
                row(ui, "Distance from board", &mut s.align_margin, 2.0..=20.0, 0.5, " mm");
            });
        }
    }

    fn job_list(&mut self, ui: &mut Ui, p: &Palette) {
        ui.label(RichText::new("JOBS, IN ORDER").size(11.0).strong().color(p.muted));
        self.hovered = None;
        for (index, job) in self.jobs.iter().enumerate() {
            let response = ui
                .horizontal(|ui| {
                    ui.label(RichText::new("●").color(job_color(job.kind)));
                    ui.label(format!("{}. {}", index + 1, job.title));
                    if !job.hint.is_empty() {
                        ui.label(RichText::new(&job.hint).color(p.muted).size(12.0));
                    }
                })
                .response;
            if response.hovered() {
                self.hovered = Some(index);
            }
        }
        if self.jobs.is_empty() {
            ui.label(RichText::new("Nothing to make from these files.").color(p.muted));
        }
    }

    /// Board outline, copper and toolpaths, scaled to fit. The hovered job is drawn thicker.
    fn preview(&self, ui: &mut Ui, p: &Palette, size: Vec2) {
        let (response, painter) = ui.allocate_painter(size, Sense::hover());
        let area = response.rect;
        painter.rect_filled(area, 6.0, p.console_bg);
        let mut points: Vec<Coord<f64>> = vec![Coord { x: 0.0, y: 0.0 }, Coord { x: self.board.width, y: self.board.height }];
        points.extend(self.jobs.iter().flat_map(|j| j.paths.iter().flatten().copied()));
        let (min_x, max_x) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), c| (lo.min(c.x), hi.max(c.x)));
        let (min_y, max_y) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), c| (lo.min(c.y), hi.max(c.y)));
        let margin = 14.0;
        let scale = ((area.width() as f64 - 2.0 * margin) / (max_x - min_x).max(1.0))
            .min((area.height() as f64 - 2.0 * margin) / (max_y - min_y).max(1.0));
        let (used_w, used_h) = ((max_x - min_x) * scale, (max_y - min_y) * scale);
        let left = area.left() as f64 + (area.width() as f64 - used_w) / 2.0;
        let bottom = area.bottom() as f64 - (area.height() as f64 - used_h) / 2.0;
        let to_screen = |c: Coord<f64>| Pos2::new((left + (c.x - min_x) * scale) as f32, (bottom - (c.y - min_y) * scale) as f32);
        let line = |ring: &[Coord<f64>], stroke: Stroke| {
            painter.add(egui::Shape::line(ring.iter().map(|c| to_screen(*c)).collect(), stroke));
        };

        if let Some(outline) = &self.board.outline {
            line(&outline.exterior().0, Stroke::new(1.5_f32, p.muted));
        }
        let show_bottom = self.board.top.is_none() || self.settings.double_sided;
        if show_bottom {
            if let Some(bottom) = &self.board.bottom {
                for ring in rings(bottom) {
                    line(&ring, Stroke::new(1.0_f32, BOTTOM_COPPER.gamma_multiply(0.6)));
                }
            }
        }
        if let Some(top) = &self.board.top {
            for ring in rings(top) {
                line(&ring, Stroke::new(1.2_f32, COPPER));
            }
        }
        for (index, job) in self.jobs.iter().enumerate() {
            let color = job_color(job.kind);
            let width = if self.hovered == Some(index) { 2.5_f32 } else { 1.0 };
            let color = if self.hovered.is_some() && self.hovered != Some(index) { color.gamma_multiply(0.35) } else { color };
            for path in &job.paths {
                if path.len() == 1 {
                    let radius = if job.kind == JobKind::Alignment { 4.0_f32 } else { 2.5 };
                    painter.circle_stroke(to_screen(path[0]), radius, Stroke::new(width, color));
                } else {
                    line(path, Stroke::new(width, color));
                }
            }
        }
    }
}

// ---------- remembered settings ----------
pub(super) fn settings_to_json(s: &Settings) -> serde_json::Value {
    serde_json::json!({
        "travel_z": s.travel_z, "spindle": s.spindle, "plunge_feed": s.plunge_feed,
        "iso_tool": s.iso_tool, "iso_passes": s.iso_passes, "iso_overlap": s.iso_overlap,
        "iso_depth": s.iso_depth, "iso_feed": s.iso_feed,
        "drill_depth": s.drill_depth, "drill_feed": s.drill_feed,
        "cut_tool": s.cut_tool, "cut_depth": s.cut_depth, "cut_step": s.cut_step, "cut_feed": s.cut_feed,
        "tabs": s.tabs, "tab_width": s.tab_width, "tab_height": s.tab_height,
        "double_sided": s.double_sided, "align_diameter": s.align_diameter, "align_margin": s.align_margin,
    })
}

pub(super) fn settings_from_json(v: &serde_json::Value) -> Settings {
    let d = Settings::default();
    let f = |key: &str, default: f64| v[key].as_f64().unwrap_or(default);
    let u = |key: &str, default: u32| v[key].as_u64().map(|n| n as u32).unwrap_or(default);
    Settings {
        travel_z: f("travel_z", d.travel_z),
        spindle: v["spindle"].as_i64().unwrap_or(d.spindle),
        plunge_feed: f("plunge_feed", d.plunge_feed),
        iso_tool: f("iso_tool", d.iso_tool),
        iso_passes: u("iso_passes", d.iso_passes),
        iso_overlap: f("iso_overlap", d.iso_overlap),
        iso_depth: f("iso_depth", d.iso_depth),
        iso_feed: f("iso_feed", d.iso_feed),
        drill_depth: f("drill_depth", d.drill_depth),
        drill_feed: f("drill_feed", d.drill_feed),
        cut_tool: f("cut_tool", d.cut_tool),
        cut_depth: f("cut_depth", d.cut_depth),
        cut_step: f("cut_step", d.cut_step),
        cut_feed: f("cut_feed", d.cut_feed),
        tabs: u("tabs", d.tabs),
        tab_width: f("tab_width", d.tab_width),
        tab_height: f("tab_height", d.tab_height),
        double_sided: v["double_sided"].as_bool().unwrap_or(d.double_sided),
        align_diameter: f("align_diameter", d.align_diameter),
        align_margin: f("align_margin", d.align_margin),
    }
}
