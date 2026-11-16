//! Small geometry helpers on top of the `geo` crate.

use geo::{unary_union, BoundingRect, Coord, LineString, MapCoords, MultiPolygon, Polygon, Rect};
use std::f64::consts::PI;

/// Largest gap between a true circle and the polygon drawn for it (mm).
const CIRCLE_TOLERANCE: f64 = 0.005;

/// A filled circle as a polygon, accurate to CIRCLE_TOLERANCE.
pub fn circle(centre: Coord<f64>, diameter: f64) -> MultiPolygon<f64> {
    let r = diameter / 2.0;
    if r <= 0.0 {
        return MultiPolygon::new(vec![]);
    }
    let step = 2.0 * (1.0 - CIRCLE_TOLERANCE / r).max(-1.0).acos();
    let n = ((2.0 * PI / step).ceil() as usize).clamp(12, 256);
    let points = (0..n)
        .map(|k| {
            let a = 2.0 * PI * k as f64 / n as f64;
            Coord { x: centre.x + r * a.cos(), y: centre.y + r * a.sin() }
        })
        .collect();
    MultiPolygon::new(vec![Polygon::new(LineString::new(points), vec![])])
}

/// Rotate about (0, 0), counter-clockwise, in degrees.
pub fn rotate(shape: &MultiPolygon<f64>, degrees: f64) -> MultiPolygon<f64> {
    if degrees == 0.0 {
        return shape.clone();
    }
    let (s, c) = degrees.to_radians().sin_cos();
    shape.map_coords(|p| Coord { x: p.x * c - p.y * s, y: p.x * s + p.y * c })
}

pub fn translate(shape: &MultiPolygon<f64>, dx: f64, dy: f64) -> MultiPolygon<f64> {
    shape.map_coords(|p| Coord { x: p.x + dx, y: p.y + dy })
}

/// Mirror left-right about the vertical line x = axis.
pub fn mirror_x(shape: &MultiPolygon<f64>, axis: f64) -> MultiPolygon<f64> {
    shape.map_coords(|p| Coord { x: 2.0 * axis - p.x, y: p.y })
}

/// Union of any number of shapes.
pub fn union_all(shapes: impl IntoIterator<Item = MultiPolygon<f64>>) -> MultiPolygon<f64> {
    let polygons: Vec<Polygon<f64>> = shapes.into_iter().flat_map(|m| m.0).collect();
    if polygons.is_empty() {
        return MultiPolygon::new(vec![]);
    }
    unary_union(polygons.iter())
}

/// Bounding box of several shapes (None if all are empty).
pub fn bounds<'a>(shapes: impl IntoIterator<Item = &'a MultiPolygon<f64>>) -> Option<Rect<f64>> {
    shapes.into_iter().filter_map(|s| s.bounding_rect()).reduce(|a, b| {
        Rect::new(
            Coord { x: a.min().x.min(b.min().x), y: a.min().y.min(b.min().y) },
            Coord { x: a.max().x.max(b.max().x), y: a.max().y.max(b.max().y) },
        )
    })
}

/// All rings (outer edges and holes) of a shape, as closed point lists.
pub fn rings(shape: &MultiPolygon<f64>) -> Vec<Vec<Coord<f64>>> {
    shape
        .0
        .iter()
        .flat_map(|p| std::iter::once(p.exterior()).chain(p.interiors()))
        .map(|ring| ring.0.clone())
        .filter(|ring| ring.len() >= 3)
        .collect()
}
