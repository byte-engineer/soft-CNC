//! Gerber (RS-274X / X2) reader: turns a layer into filled copper polygons, the way FlatCAM does.
//!
//! Supports what PCB tools (KiCad, EasyEDA, Eagle…) write: standard apertures (C, R, O, P), aperture macros
//! (circle, vector/center line, outline, polygon, thermal) with $ variables and arithmetic, linear and arc
//! draws (G01/G02/G03, G74/G75), flashes, regions (G36/G37), dark/clear polarity, mm and inch.

use super::geom::{circle, rotate, union_all};
use geo::{BooleanOps, Buffer, ConvexHull, Coord, LineString, MultiPoint, MultiPolygon, Point, Polygon};
use regex::Regex;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::LazyLock;

static WORD_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"([GDMXYIJ])([+-]?[\d.]+)").unwrap());

/// A parsed layer.
#[derive(Debug, Clone)]
pub struct GerberLayer {
    /// Everything dark (copper), with clear areas already removed.
    pub image: MultiPolygon<f64>,
    /// Widest aperture used for draws (for outlines drawn as lines: the line width).
    pub stroke_width: f64,
    /// X2 file function, e.g. "Copper,L1,Top" or "Profile,NP".
    pub function: Option<String>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone)]
enum Aperture {
    Circle(f64),
    Rect(f64, f64),
    Obround(f64, f64),
    Polygon { diameter: f64, vertices: usize, rotation: f64 },
    /// A macro instance, already evaluated into a shape around (0, 0).
    Shape(MultiPolygon<f64>),
}

#[derive(Clone, Copy, PartialEq)]
enum Interp {
    Linear,
    Clockwise,
    CounterClockwise,
}

struct Parser {
    int_digits: u32,
    dec_digits: u32,
    trailing_zeros: bool, // FS ..T..: trailing zeros omitted (deprecated)
    unit: f64,            // 1.0 for mm, 25.4 for inch
    macros: HashMap<String, Vec<String>>,
    apertures: HashMap<u32, Aperture>,
    current: Option<u32>,
    pos: Coord<f64>,
    interp: Interp,
    multi_quadrant: bool,
    region: Option<Vec<Vec<Coord<f64>>>>, // contours of the current G36 region
    last_op: u32,
    dark: bool,
    pending: Vec<MultiPolygon<f64>>, // objects of the current polarity, not yet merged
    image: MultiPolygon<f64>,
    stroke_width: f64,
    function: Option<String>,
    warnings: Vec<String>,
}

pub fn parse(text: &str) -> Result<GerberLayer, String> {
    let mut p = Parser {
        int_digits: 3,
        dec_digits: 6,
        trailing_zeros: false,
        unit: 1.0,
        macros: HashMap::new(),
        apertures: HashMap::new(),
        current: None,
        pos: Coord { x: 0.0, y: 0.0 },
        interp: Interp::Linear,
        multi_quadrant: true,
        region: None,
        last_op: 2,
        dark: true,
        pending: Vec::new(),
        image: MultiPolygon::new(vec![]),
        stroke_width: 0.0,
        function: None,
        warnings: Vec::new(),
    };
    let mut rest = text;
    while let Some(start) = rest.find(|c: char| !c.is_whitespace()) {
        rest = &rest[start..];
        if let Some(body) = rest.strip_prefix('%') {
            let end = body.find('%').ok_or("unterminated % block")?;
            let block = body[..end].replace(['\r', '\n'], "");
            if block.starts_with("AM") {
                p.extended(&block)?; // a macro: its '*'-separated statements belong together
            } else {
                // older files put several commands in one block: %FSLAX24Y24*MOIN*%
                for command in block.split('*').filter(|c| !c.trim().is_empty()) {
                    p.extended(command.trim())?;
                }
            }
            rest = &body[end + 1..];
        } else {
            let end = rest.find('*').unwrap_or(rest.len());
            let word = rest[..end].replace(['\r', '\n'], "");
            rest = rest.get(end + 1..).unwrap_or("");
            if p.word(word.trim())? {
                break; // M02: end of file
            }
        }
    }
    p.flush();
    Ok(GerberLayer { image: p.image, stroke_width: p.stroke_width, function: p.function, warnings: p.warnings })
}

impl Parser {
    fn warn(&mut self, message: impl Into<String>) {
        let message = message.into();
        if !self.warnings.contains(&message) {
            self.warnings.push(message);
        }
    }

    // ---------- extended (%...%) commands ----------
    fn extended(&mut self, block: &str) -> Result<(), String> {
        let code = block.get(..2).unwrap_or("");
        let body = block.get(2..).unwrap_or("").trim_end_matches('*');
        match code {
            "FS" => {
                // e.g. LAX46Y46: zero omission (L/T), absolute (A), X and Y digit counts
                self.trailing_zeros = body.starts_with('T');
                let x = body.find('X').ok_or("bad FS")?;
                let digits: Vec<u32> = body[x + 1..].chars().take(2).filter_map(|c| c.to_digit(10)).collect();
                if digits.len() == 2 {
                    self.int_digits = digits[0];
                    self.dec_digits = digits[1];
                }
                if body.contains('I') && !body.starts_with('I') {
                    self.warn("incremental coordinates (FS..I) are not supported; treated as absolute");
                }
            }
            "MO" => self.unit = if body.starts_with("IN") { 25.4 } else { 1.0 },
            "AD" => self.define_aperture(body)?,
            "AM" => {
                let mut parts = block[2..].split('*');
                let name = parts.next().unwrap_or("").to_string();
                let statements = parts.map(str::trim).filter(|s| !s.is_empty()).map(String::from).collect();
                self.macros.insert(name, statements);
            }
            "LP" => {
                let dark = body.starts_with('D');
                if dark != self.dark {
                    self.flush();
                    self.dark = dark;
                }
            }
            "TF" => {
                if let Some(function) = body.strip_prefix(".FileFunction,") {
                    self.function = Some(function.to_string());
                }
            }
            "SR" => {
                if body.contains('X') && !body.starts_with("X1Y1") {
                    self.warn("step-and-repeat (SR) is not supported; only the first copy is used");
                }
            }
            "AB" => return Err("block apertures (AB) are not supported".into()),
            "LM" | "LR" | "LS" => self.warn("mirror/rotate/scale (LM/LR/LS) commands are ignored"),
            "IP" if body.starts_with("NEG") => self.warn("negative image polarity (IPNEG) is ignored"),
            _ => {} // TA, TO, TD, IN, OF, ... : attributes and no-ops
        }
        Ok(())
    }

    fn define_aperture(&mut self, body: &str) -> Result<(), String> {
        // D10C,0.2  /  D11RoundRect,0.25X-0.6X...
        let body = body.strip_prefix('D').ok_or("bad AD")?;
        let digits = body.find(|c: char| !c.is_ascii_digit()).ok_or("bad AD")?;
        let code: u32 = body[..digits].parse().map_err(|_| "bad AD code")?;
        let (name, params) = match body[digits..].split_once(',') {
            Some((n, p)) => (n, p.split('X').filter_map(|v| v.trim().parse::<f64>().ok()).collect::<Vec<_>>()),
            None => (&body[digits..], Vec::new()),
        };
        let u = self.unit;
        let p = |i: usize| params.get(i).copied().unwrap_or(0.0);
        let aperture = match name {
            "C" => Aperture::Circle(p(0) * u),
            "R" => Aperture::Rect(p(0) * u, p(1) * u),
            "O" => Aperture::Obround(p(0) * u, p(1) * u),
            "P" => Aperture::Polygon { diameter: p(0) * u, vertices: p(1).max(3.0) as usize, rotation: p(2) },
            macro_name => {
                let statements = self.macros.get(macro_name).cloned()
                    .ok_or_else(|| format!("aperture D{code} uses unknown macro '{macro_name}'"))?;
                Aperture::Shape(self.evaluate_macro(&statements, &params)?)
            }
        };
        self.apertures.insert(code, aperture);
        Ok(())
    }

    // ---------- aperture macros ----------
    fn evaluate_macro(&mut self, statements: &[String], params: &[f64]) -> Result<MultiPolygon<f64>, String> {
        let mut vars: HashMap<u32, f64> = params.iter().enumerate().map(|(i, v)| (i as u32 + 1, *v)).collect();
        let mut shape = MultiPolygon::new(vec![]);
        for statement in statements {
            if statement.starts_with('0') && !statement.starts_with("0,") {
                continue; // comment primitive
            }
            if let Some(assign) = statement.strip_prefix('$') {
                let (var, expr) = assign.split_once('=').ok_or("bad macro variable")?;
                let value = eval(expr, &vars)?;
                vars.insert(var.trim().parse().map_err(|_| "bad macro variable")?, value);
                continue;
            }
            let fields: Vec<f64> = statement.split(',').map(|f| eval(f, &vars)).collect::<Result<_, _>>()?;
            let code = fields[0] as i64;
            let a = |i: usize| fields.get(i).copied().unwrap_or(0.0);
            let u = self.unit;
            let (exposure, primitive) = match code {
                1 => (a(1), rotate(&circle(Coord { x: a(3) * u, y: a(4) * u }, a(2) * u), a(5))),
                2 | 20 => {
                    let (s, e) = (Coord { x: a(3) * u, y: a(4) * u }, Coord { x: a(5) * u, y: a(6) * u });
                    (a(1), rotate(&vector_line(s, e, a(2) * u), a(7)))
                }
                21 => (a(1), rotate(&rect(Coord { x: a(4) * u, y: a(5) * u }, a(2) * u, a(3) * u), a(6))),
                22 => {
                    let (w, h) = (a(2) * u, a(3) * u);
                    (a(1), rotate(&rect(Coord { x: a(4) * u + w / 2.0, y: a(5) * u + h / 2.0 }, w, h), a(6)))
                }
                4 => {
                    let n = a(2) as usize;
                    let points: Vec<Coord<f64>> =
                        (0..=n).map(|k| Coord { x: a(3 + 2 * k) * u, y: a(4 + 2 * k) * u }).collect();
                    let outline = union_all([MultiPolygon::new(vec![Polygon::new(LineString::new(points), vec![])])]);
                    (a(1), rotate(&outline, a(5 + 2 * n)))
                }
                5 => {
                    let centre = Coord { x: a(3) * u, y: a(4) * u };
                    (a(1), rotate(&regular_polygon(centre, a(5) * u, a(2) as usize, 0.0), a(6)))
                }
                7 => {
                    let centre = Coord { x: a(1) * u, y: a(2) * u };
                    (1.0, rotate(&thermal(centre, a(3) * u, a(4) * u, a(5) * u), a(6)))
                }
                6 => {
                    self.warn("moiré macro primitives are drawn as their outer circle only");
                    (1.0, circle(Coord { x: a(1) * u, y: a(2) * u }, a(3) * u))
                }
                other => return Err(format!("unknown aperture macro primitive {other}")),
            };
            shape = if exposure == 0.0 { shape.difference(&primitive) } else { shape.union(&primitive) };
        }
        Ok(shape)
    }

    // ---------- word commands ----------
    /// Returns true at the end of the file (M02).
    fn word(&mut self, word: &str) -> Result<bool, String> {
        if word.is_empty() || word.starts_with("G04") || word.starts_with("G4 ") {
            return Ok(false); // comment
        }
        let mut x = None;
        let mut y = None;
        let (mut i, mut j) = (0.0, 0.0);
        let mut op = None;
        for c in WORD_RE.captures_iter(word) {
            let value = &c[2];
            match &c[1] {
                "G" => match value.parse::<u32>().unwrap_or(0) {
                    1 => self.interp = Interp::Linear,
                    2 => self.interp = Interp::Clockwise,
                    3 => self.interp = Interp::CounterClockwise,
                    74 => self.multi_quadrant = false,
                    75 => self.multi_quadrant = true,
                    36 => self.region = Some(vec![vec![self.pos]]),
                    37 => self.end_region(),
                    70 => self.unit = 25.4,
                    71 => self.unit = 1.0,
                    91 => self.warn("incremental coordinates (G91) are not supported; treated as absolute"),
                    _ => {} // G54 (select aperture prefix), G90, ...
                },
                "D" => {
                    let d: u32 = value.parse().map_err(|_| format!("bad D code in '{word}'"))?;
                    if d >= 10 {
                        self.current = Some(d);
                    } else {
                        op = Some(d);
                    }
                }
                "M" => {
                    if value.trim_start_matches('0') == "2" || value == "00" || value == "0" {
                        return Ok(true);
                    }
                }
                "X" => x = Some(self.coord(value)?),
                "Y" => y = Some(self.coord(value)?),
                "I" => i = self.coord(value)?,
                "J" => j = self.coord(value)?,
                _ => {}
            }
        }
        let has_coords = x.is_some() || y.is_some();
        let op = match op {
            Some(op) => op,
            None if has_coords => self.last_op, // deprecated modal operation
            None => return Ok(false),
        };
        self.last_op = op;
        let target = Coord { x: x.unwrap_or(self.pos.x), y: y.unwrap_or(self.pos.y) };
        match op {
            1 => {
                let points = self.interpolate(self.pos, target, Coord { x: i, y: j });
                if let Some(region) = &mut self.region {
                    region.last_mut().unwrap().extend(points.into_iter().skip(1));
                } else {
                    self.stroke(&points)?;
                }
            }
            2 => {
                if let Some(region) = &mut self.region {
                    region.push(vec![target]);
                }
            }
            3 => {
                let shape = self.aperture_shape(target)?;
                self.pending.push(shape);
            }
            other => return Err(format!("unknown operation D0{other}")),
        }
        self.pos = target;
        Ok(false)
    }

    fn coord(&self, text: &str) -> Result<f64, String> {
        if text.contains('.') {
            return text.parse::<f64>().map(|v| v * self.unit).map_err(|_| format!("bad number '{text}'"));
        }
        let negative = text.starts_with('-');
        let mut digits = text.trim_start_matches(['+', '-']).to_string();
        if self.trailing_zeros {
            let total = (self.int_digits + self.dec_digits) as usize;
            while digits.len() < total {
                digits.push('0');
            }
        }
        let value: f64 = digits.parse::<f64>().map_err(|_| format!("bad number '{text}'"))?
            / 10f64.powi(self.dec_digits as i32);
        Ok(if negative { -value } else { value } * self.unit)
    }

    /// Points from `from` to `to` along the current interpolation (arcs are split into short lines).
    fn interpolate(&mut self, from: Coord<f64>, to: Coord<f64>, offset: Coord<f64>) -> Vec<Coord<f64>> {
        if self.interp == Interp::Linear {
            return vec![from, to];
        }
        let clockwise = self.interp == Interp::Clockwise;
        let centre = if self.multi_quadrant {
            from + offset
        } else {
            // single-quadrant: I/J are unsigned; pick the centre that is equally far from both ends
            let candidates = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
                .map(|(sx, sy)| Coord { x: from.x + sx * offset.x, y: from.y + sy * offset.y });
            *candidates.iter().min_by(|a, b| {
                let score = |c: &Coord<f64>| (dist(*c, from) - dist(*c, to)).abs();
                score(a).total_cmp(&score(b))
            }).unwrap()
        };
        let radius = dist(centre, from);
        let a0 = (from.y - centre.y).atan2(from.x - centre.x);
        let a1 = (to.y - centre.y).atan2(to.x - centre.x);
        let mut sweep = if clockwise { a0 - a1 } else { a1 - a0 };
        sweep = sweep.rem_euclid(2.0 * PI);
        if sweep < 1e-9 {
            sweep = if self.multi_quadrant { 2.0 * PI } else { 0.0 };
        }
        let segments = ((sweep * radius / 0.1).ceil() as usize).clamp(2, 720); // ~0.1 mm per segment
        let direction = if clockwise { -1.0 } else { 1.0 };
        let mut points: Vec<Coord<f64>> = (0..segments)
            .map(|k| {
                let angle = a0 + direction * sweep * k as f64 / segments as f64;
                Coord { x: centre.x + radius * angle.cos(), y: centre.y + radius * angle.sin() }
            })
            .collect();
        points.push(to);
        points
    }

    fn stroke(&mut self, points: &[Coord<f64>]) -> Result<(), String> {
        let code = self.current.ok_or("draw before any aperture was selected")?;
        let aperture = self.apertures.get(&code).cloned().ok_or_else(|| format!("aperture D{code} is not defined"))?;
        let zero_length = points.windows(2).all(|w| dist(w[0], w[1]) < 1e-9);
        if zero_length {
            let shape = self.aperture_shape(points[0])?;
            self.pending.push(shape);
            return Ok(());
        }
        let shape = match aperture {
            Aperture::Rect(w, h) if points.len() == 2 => {
                // a rectangle dragged in a straight line: hull of the rectangle at both ends
                let corners: Vec<Point<f64>> = points
                    .iter()
                    .flat_map(|p| {
                        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                            .map(|(sx, sy)| Point::new(p.x + sx * w / 2.0, p.y + sy * h / 2.0))
                    })
                    .collect();
                self.stroke_width = self.stroke_width.max(w.min(h));
                MultiPolygon::new(vec![MultiPoint::new(corners).convex_hull()])
            }
            other => {
                let width = match other {
                    Aperture::Circle(d) => d,
                    Aperture::Rect(w, h) | Aperture::Obround(w, h) => {
                        self.warn("an arc drawn with a non-round aperture is approximated with a round one");
                        w.min(h)
                    }
                    Aperture::Polygon { diameter, .. } => diameter,
                    Aperture::Shape(_) => return Err("drawing with a macro aperture is not allowed".into()),
                };
                self.stroke_width = self.stroke_width.max(width);
                if width <= 0.0 {
                    return Ok(()); // zero-width line: nothing to fill
                }
                LineString::new(points.to_vec()).buffer(width / 2.0)
            }
        };
        self.pending.push(shape);
        Ok(())
    }

    fn aperture_shape(&self, at: Coord<f64>) -> Result<MultiPolygon<f64>, String> {
        let code = self.current.ok_or("flash before any aperture was selected")?;
        let aperture = self.apertures.get(&code).ok_or_else(|| format!("aperture D{code} is not defined"))?;
        Ok(match aperture {
            Aperture::Circle(d) => circle(at, *d),
            Aperture::Rect(w, h) => rect(at, *w, *h),
            Aperture::Obround(w, h) => obround(at, *w, *h),
            Aperture::Polygon { diameter, vertices, rotation } => regular_polygon(at, *diameter, *vertices, *rotation),
            Aperture::Shape(shape) => super::geom::translate(shape, at.x, at.y),
        })
    }

    fn end_region(&mut self) {
        if let Some(contours) = self.region.take() {
            let polygons: Vec<MultiPolygon<f64>> = contours
                .into_iter()
                .filter(|c| c.len() >= 3)
                .map(|c| MultiPolygon::new(vec![Polygon::new(LineString::new(c), vec![])]))
                .collect();
            self.pending.push(union_all(polygons));
        }
    }

    /// Merge the pending objects into the image (dark: add, clear: cut away).
    fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let objects = union_all(std::mem::take(&mut self.pending));
        self.image = if self.dark { self.image.union(&objects) } else { self.image.difference(&objects) };
    }
}

// ---------- shapes ----------
fn dist(a: Coord<f64>, b: Coord<f64>) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn polygon(points: Vec<Coord<f64>>) -> MultiPolygon<f64> {
    MultiPolygon::new(vec![Polygon::new(LineString::new(points), vec![])])
}

fn rect(centre: Coord<f64>, w: f64, h: f64) -> MultiPolygon<f64> {
    let (x, y) = (w / 2.0, h / 2.0);
    polygon(vec![
        Coord { x: centre.x - x, y: centre.y - y },
        Coord { x: centre.x + x, y: centre.y - y },
        Coord { x: centre.x + x, y: centre.y + y },
        Coord { x: centre.x - x, y: centre.y + y },
    ])
}

fn obround(centre: Coord<f64>, w: f64, h: f64) -> MultiPolygon<f64> {
    if (w - h).abs() < 1e-9 {
        return circle(centre, w);
    }
    let (half, radius) = if w > h { (Coord { x: (w - h) / 2.0, y: 0.0 }, h / 2.0) } else { (Coord { x: 0.0, y: (h - w) / 2.0 }, w / 2.0) };
    LineString::new(vec![centre - half, centre + half]).buffer(radius)
}

fn regular_polygon(centre: Coord<f64>, diameter: f64, vertices: usize, rotation_deg: f64) -> MultiPolygon<f64> {
    let r = diameter / 2.0;
    polygon(
        (0..vertices.max(3))
            .map(|k| {
                let angle = rotation_deg.to_radians() + 2.0 * PI * k as f64 / vertices.max(3) as f64;
                Coord { x: centre.x + r * angle.cos(), y: centre.y + r * angle.sin() }
            })
            .collect(),
    )
}

/// A line of the given width with square (butt) ends: macro primitive 20.
fn vector_line(start: Coord<f64>, end: Coord<f64>, width: f64) -> MultiPolygon<f64> {
    let length = dist(start, end);
    if length < 1e-12 {
        return MultiPolygon::new(vec![]);
    }
    let n = Coord { x: -(end.y - start.y) / length * width / 2.0, y: (end.x - start.x) / length * width / 2.0 };
    polygon(vec![start + n, end + n, end - n, start - n])
}

/// Thermal relief: a ring with a cross-shaped gap (macro primitive 7).
fn thermal(centre: Coord<f64>, outer: f64, inner: f64, gap: f64) -> MultiPolygon<f64> {
    let ring = circle(centre, outer).difference(&circle(centre, inner));
    let cross = rect(centre, outer * 2.0, gap).union(&rect(centre, gap, outer * 2.0));
    ring.difference(&cross)
}

// ---------- macro arithmetic ----------
/// Evaluate a macro expression: numbers, $n variables, + - x / and parentheses.
fn eval(expr: &str, vars: &HashMap<u32, f64>) -> Result<f64, String> {
    let tokens: Vec<char> = expr.chars().filter(|c| !c.is_whitespace()).collect();
    let mut pos = 0;
    let value = sum(&tokens, &mut pos, vars)?;
    if pos != tokens.len() {
        return Err(format!("bad macro expression '{expr}'"));
    }
    Ok(value)
}

fn sum(t: &[char], pos: &mut usize, vars: &HashMap<u32, f64>) -> Result<f64, String> {
    let mut value = product(t, pos, vars)?;
    while *pos < t.len() && (t[*pos] == '+' || t[*pos] == '-') {
        let op = t[*pos];
        *pos += 1;
        let rhs = product(t, pos, vars)?;
        value = if op == '+' { value + rhs } else { value - rhs };
    }
    Ok(value)
}

fn product(t: &[char], pos: &mut usize, vars: &HashMap<u32, f64>) -> Result<f64, String> {
    let mut value = factor(t, pos, vars)?;
    while *pos < t.len() && matches!(t[*pos], 'x' | 'X' | '/') {
        let op = t[*pos];
        *pos += 1;
        let rhs = factor(t, pos, vars)?;
        value = if op == '/' { value / rhs } else { value * rhs };
    }
    Ok(value)
}

fn factor(t: &[char], pos: &mut usize, vars: &HashMap<u32, f64>) -> Result<f64, String> {
    match t.get(*pos) {
        Some('-') => {
            *pos += 1;
            Ok(-factor(t, pos, vars)?)
        }
        Some('+') => {
            *pos += 1;
            factor(t, pos, vars)
        }
        Some('(') => {
            *pos += 1;
            let value = sum(t, pos, vars)?;
            if t.get(*pos) != Some(&')') {
                return Err("missing ')' in macro expression".into());
            }
            *pos += 1;
            Ok(value)
        }
        Some('$') => {
            *pos += 1;
            let start = *pos;
            while *pos < t.len() && t[*pos].is_ascii_digit() {
                *pos += 1;
            }
            let n: u32 = t[start..*pos].iter().collect::<String>().parse().map_err(|_| "bad $ variable")?;
            Ok(vars.get(&n).copied().unwrap_or(0.0))
        }
        Some(c) if c.is_ascii_digit() || *c == '.' => {
            let start = *pos;
            while *pos < t.len() && (t[*pos].is_ascii_digit() || t[*pos] == '.') {
                *pos += 1;
            }
            t[start..*pos].iter().collect::<String>().parse().map_err(|_| "bad number in macro".into())
        }
        _ => Err("bad macro expression".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{Area, BoundingRect};

    fn area(layer: &GerberLayer) -> f64 {
        layer.image.unsigned_area()
    }

    const HEADER: &str = "%FSLAX46Y46*%\n%MOMM*%\n%LPD*%\n";

    #[test]
    fn macro_expressions() {
        let vars: HashMap<u32, f64> = [(1, 0.25), (2, -0.6)].into();
        assert_eq!(eval("$1+$1", &vars).unwrap(), 0.5);
        assert_eq!(eval("$2x2", &vars).unwrap(), -1.2);
        assert_eq!(eval("(1+2)x3/-3", &vars).unwrap(), -3.0);
    }

    #[test]
    fn flashes_lines_and_units() {
        let g = format!("{HEADER}%ADD10C,1.000000*%\n%ADD11R,2.000000X1.000000*%\nD10*\nX0Y0D03*\nD11*\nX10000000Y0D03*\nM02*\n");
        let layer = parse(&g).unwrap();
        assert!((area(&layer) - (PI * 0.25 + 2.0)).abs() < 0.01, "{}", area(&layer));
        // the same 1 mm circle in inches: 0.03937 in
        let inch = "%FSLAX24Y24*%\n%MOIN*%\n%ADD10C,0.03937*%\nD10*\nX0Y0D03*\nM02*\n";
        assert!((area(&parse(inch).unwrap()) - PI * 0.25).abs() < 0.01);
    }

    #[test]
    fn tracks_regions_and_clear_polarity() {
        // a 10 mm x 0.5 mm track, a 4x4 region with a 2x2 clear square cut out of it
        let g = format!(
            "{HEADER}%ADD10C,0.500000*%\nD10*\nX0Y0D02*\nX10000000Y0D01*\n\
             G36*\nX20000000Y0D02*\nX24000000Y0D01*\nX24000000Y4000000D01*\nX20000000Y4000000D01*\nX20000000Y0D01*\nG37*\n\
             %LPC*%\nG36*\nX21000000Y1000000D02*\nX23000000Y1000000D01*\nX23000000Y3000000D01*\nX21000000Y3000000D01*\nG37*\nM02*\n"
        );
        let layer = parse(&g).unwrap();
        let expected = 10.0 * 0.5 + PI * 0.0625 + 16.0 - 4.0;
        assert!((area(&layer) - expected).abs() < 0.02, "{} vs {expected}", area(&layer));
        assert_eq!(layer.stroke_width, 0.5);
    }

    #[test]
    fn arcs_follow_the_circle() {
        // a full circle of radius 5 drawn with a 0.2 mm line, multi-quadrant
        let g = format!("{HEADER}%ADD10C,0.200000*%\nD10*\nG75*\nX5000000Y0D02*\nG03X5000000Y0I-5000000J0D01*\nM02*\n");
        let layer = parse(&g).unwrap();
        let ring = 2.0 * PI * 5.0 * 0.2;
        assert!((area(&layer) - ring).abs() < 0.05, "{} vs {ring}", area(&layer));
        let b = layer.image.bounding_rect().unwrap();
        assert!((b.max().x - 5.1).abs() < 0.01 && (b.min().y + 5.1).abs() < 0.01);
    }

    #[test]
    fn kicad_roundrect_macro() {
        let g = format!(
            "{HEADER}%AMRoundRect*\n0 Rectangle with rounded corners*\n0 $1 Rounding radius*\n\
             4,1,4,$2,$3,$4,$5,$6,$7,$8,$9,$2,$3,0*\n1,1,$1+$1,$2,$3*\n1,1,$1+$1,$4,$5*\n1,1,$1+$1,$6,$7*\n1,1,$1+$1,$8,$9*\n\
             20,1,$1+$1,$2,$3,$4,$5,0*\n20,1,$1+$1,$4,$5,$6,$7,0*\n20,1,$1+$1,$6,$7,$8,$9,0*\n20,1,$1+$1,$8,$9,$2,$3,0*%\n\
             %ADD10RoundRect,0.250000X-0.750000X-0.500000X0.750000X-0.500000X0.750000X0.500000X-0.750000X0.500000X0*%\n\
             D10*\nX5000000Y5000000D03*\nM02*\n"
        );
        let layer = parse(&g).unwrap();
        // a 2 x 1.5 pad with 0.25 corner radius
        let expected = 2.0 * 1.5 - (4.0 - PI) * 0.25 * 0.25;
        assert!((area(&layer) - expected).abs() < 0.02, "{} vs {expected}", area(&layer));
        let b = layer.image.bounding_rect().unwrap();
        assert!((b.min().x - 4.0).abs() < 0.01 && (b.max().y - 5.75).abs() < 0.01);
    }

    #[test]
    fn file_function_is_read() {
        let g = format!("%TF.FileFunction,Copper,L1,Top*%\n{HEADER}M02*\n");
        assert_eq!(parse(&g).unwrap().function.as_deref(), Some("Copper,L1,Top"));
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;
    use geo::{Area, BoundingRect};

    const MM: &str = "%FSLAX46Y46*%\n%MOMM*%\n";

    fn area(text: &str) -> f64 {
        parse(text).unwrap().image.unsigned_area()
    }

    fn close(a: f64, b: f64, tolerance: f64) -> bool {
        (a - b).abs() < tolerance
    }

    #[test]
    fn several_commands_in_one_block_old_style() {
        // FS and MO in one % block: the inch unit must not be lost (a 0.04 in = 1.016 mm pad)
        let g = "%FSLAX24Y24*MOIN*%\n%ADD10C,0.04*%\nD10*\nX0Y0D03*\nM02*\n";
        // circles are polygons accurate to 5 µm: ~1 % less area on a 1 mm pad (in mm units it would be ~0.001)
        assert!(close(area(g), PI * 0.508 * 0.508, 0.015), "{}", area(g));
    }

    #[test]
    fn trailing_zero_format_and_decimal_coordinates() {
        // FS..T: trailing zeros dropped: X15 with 2.4 digits means 15xxxx → 15.0000 mm
        let g = "%FSTAX24Y24*%\n%MOMM*%\n%ADD10C,1*%\nD10*\nX15Y15D03*\nM02*\n";
        let b = parse(g).unwrap().image.bounding_rect().unwrap();
        assert!(close(b.center().x, 15.0, 0.005), "{b:?}"); // circle polygons: centre within a few µm
        let g = format!("{MM}%ADD10C,1*%\nD10*\nX2.5Y-1.5D03*\nM02*\n");
        let b = parse(&g).unwrap().image.bounding_rect().unwrap();
        assert!(close(b.center().x, 2.5, 0.005) && close(b.center().y, -1.5, 0.005));
    }

    #[test]
    fn g70_g71_switch_units_mid_file() {
        let g = "%FSLAX24Y24*%\nG70*\n%ADD10C,0.04*%\nD10*\nX0Y0D03*\nM02*\n";
        assert!(close(area(g), PI * 0.508 * 0.508, 0.015));
    }

    #[test]
    fn standard_aperture_shapes() {
        // obround 3 x 1 = 2 x 1 rectangle + 1 mm circle; hexagon of 2 mm across corners
        assert!(close(area(&format!("{MM}%ADD10O,3X1*%\nD10*\nX0Y0D03*\nM02*\n")), 2.0 + PI * 0.25, 0.01));
        let hexagon = 3.0 * 3f64.sqrt() / 2.0;
        assert!(close(area(&format!("{MM}%ADD10P,2X6*%\nD10*\nX0Y0D03*\nM02*\n")), hexagon, 0.01));
        // a circle with a hole parameter is still filled (copper; the hole is drilled)
        assert!(close(area(&format!("{MM}%ADD10C,1X0.5*%\nD10*\nX0Y0D03*\nM02*\n")), PI * 0.25, 0.01));
    }

    #[test]
    fn rectangle_dragged_along_a_line() {
        // 1 x 0.5 rectangle dragged 10 mm along X: 11 x 0.5
        let g = format!("{MM}%ADD10R,1X0.5*%\nD10*\nX0Y0D02*\nX10000000Y0D01*\nM02*\n");
        assert!(close(area(&g), 11.0 * 0.5, 1e-6));
    }

    #[test]
    fn clear_polarity_flash_makes_a_hole() {
        let g = format!("{MM}%ADD10C,2*%\n%ADD11C,1*%\nD10*\nX0Y0D03*\n%LPC*%\nD11*\nX0Y0D03*\nM02*\n");
        assert!(close(area(&g), PI * (1.0 - 0.25), 0.02));
    }

    #[test]
    fn single_quadrant_arc_g74() {
        // quarter circle radius 5 from (5,0) to (0,5), I/J unsigned
        let g = format!("{MM}%ADD10C,0.1*%\nD10*\nG74*\nX5000000Y0D02*\nG03X0Y5000000I5000000J0D01*\nM02*\n");
        let ring = PI * 5.0 / 2.0 * 0.1;
        assert!(close(area(&g), ring, 0.01), "{} vs {ring}", area(&g));
    }

    #[test]
    fn modal_operation_and_zero_length_draw() {
        // coordinates with no D code repeat the last operation (D01): 3 segments of 1 mm
        let g = format!("{MM}%ADD10C,0.2*%\nD10*\nX0Y0D02*\nX1000000Y0D01*\nX2000000Y0*\nX3000000Y0*\nM02*\n");
        assert!(close(area(&g), 3.0 * 0.2 + PI * 0.01, 0.01));
        // a draw that doesn't move is a dot
        let g = format!("{MM}%ADD10C,1*%\nD10*\nX0Y0D02*\nX0Y0D01*\nM02*\n");
        assert!(close(area(&g), PI * 0.25, 0.01));
    }

    #[test]
    fn macro_primitives_and_variables() {
        let g = format!(
            "{MM}%AMBOX*$3=$1x2*21,1,$3,$2,0,0,0*%\n%ADD10BOX,1X1*%\nD10*\nX0Y0D03*\nM02*\n"
        );
        assert!(close(area(&g), 2.0, 1e-6), "21 centre line, width from a $ variable");
        let g = format!("{MM}%AMLL*22,1,2,1,0,0,0*%\n%ADD10LL*%\nD10*\nX0Y0D03*\nM02*\n");
        let b = parse(&g).unwrap().image.bounding_rect().unwrap();
        assert!(close(b.min().x, 0.0, 1e-9) && close(b.max().y, 1.0, 1e-9), "22 lower-left line");
        let g = format!("{MM}%AMHOLE*1,1,2,0,0*1,0,1,0,0*%\n%ADD10HOLE*%\nD10*\nX0Y0D03*\nM02*\n");
        assert!(close(area(&g), PI * 0.75, 0.02), "exposure 0 cuts a hole");
        let g = format!("{MM}%AMPENT*5,1,5,0,0,2,0*%\n%ADD10PENT*%\nD10*\nX0Y0D03*\nM02*\n");
        let pentagon = 5.0 / 2.0 * (2.0 * PI / 5.0).sin();
        assert!(close(area(&g), pentagon, 0.01), "5 regular polygon");
        let g = format!("{MM}%AMTH*7,0,0,2,1.5,0.2,0*%\n%ADD10TH*%\nD10*\nX0Y0D03*\nM02*\n");
        let ring = PI * (1.0 - 0.5625);
        let a = area(&g);
        assert!(a < ring && a > ring - 4.0 * 0.2 * 0.25 - 0.02, "7 thermal: ring minus 4 gaps: {a}");
    }

    #[test]
    fn rotated_macro() {
        // a 2 x 0.5 centre line rotated 90° stands upright
        let g = format!("{MM}%AMROT*21,1,2,0.5,0,0,90*%\n%ADD10ROT*%\nD10*\nX0Y0D03*\nM02*\n");
        let b = parse(&g).unwrap().image.bounding_rect().unwrap();
        assert!(close(b.height(), 2.0, 1e-6) && close(b.width(), 0.5, 1e-6));
    }

    #[test]
    fn everything_after_m02_is_ignored_and_an_empty_file_is_empty() {
        let g = format!("{MM}%ADD10C,1*%\nD10*\nX0Y0D03*\nM02*\nX5000000Y0D03*\n");
        assert!(close(area(&g), PI * 0.25, 0.01));
        assert!(parse(&format!("{MM}M02*\n")).unwrap().image.0.is_empty());
    }

    #[test]
    fn errors_are_reported() {
        assert!(parse(&format!("{MM}%ADD10NOPE,1*%\nM02*\n")).unwrap_err().contains("unknown macro 'NOPE'"));
        assert!(parse(&format!("{MM}D10*\nX0Y0D03*\n")).unwrap_err().contains("D10 is not defined"));
        assert!(parse(&format!("{MM}X0Y0D03*\n")).unwrap_err().contains("before any aperture"));
        assert!(parse(&format!("{MM}%ABD12*%\n")).unwrap_err().contains("block apertures"));
        assert!(parse(&format!("{MM}%ADD10C,1*\n")).unwrap_err().contains("unterminated"));
        assert!(parse(&format!("{MM}%AMX*9,1*%\n%ADD10X*%\n")).unwrap_err().contains("primitive 9"));
    }

    #[test]
    fn unsupported_features_warn_instead_of_failing() {
        let layer = parse(&format!("{MM}%SRX2Y2I5J5*%\n%LMX*%\nM02*\n")).unwrap();
        assert!(layer.warnings.iter().any(|w| w.contains("step-and-repeat")));
        assert!(layer.warnings.iter().any(|w| w.contains("mirror/rotate/scale")));
    }
}
