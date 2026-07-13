//! Tiny SVG trajectory plotter.
//!
//! Writes two trajectory views per mission:
//!   - top-down: world x (forward) vs y (right), math orientation (screen y inverted)
//!   - side: world x (forward) vs altitude (= -z in NED)
//!
//! Hand-rolled SVG keeps the sweep binary dependency-free. The axes, origin
//! reference lines, and start/end markers make it easy to eyeball a
//! trajectory at a glance in a markdown renderer.

use std::fmt::Write as _;
use std::io::Result;
use std::path::Path;

use crate::metrics::Sample;

const WIDTH: f32 = 440.0;
const HEIGHT: f32 = 360.0;
const MARGIN_L: f32 = 56.0;
const MARGIN_R: f32 = 20.0;
const MARGIN_T: f32 = 30.0;
const MARGIN_B: f32 = 40.0;

pub fn write_top_down_svg(samples: &[Sample], path: &Path) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let xs: Vec<f32> = samples.iter().map(|s| s.pos[0]).collect();
    let ys: Vec<f32> = samples.iter().map(|s| s.pos[1]).collect();
    std::fs::write(
        path,
        render(&xs, &ys, "x (m, forward)", "y (m, right)", "Top-down (x-y)"),
    )
}

pub fn write_side_svg(samples: &[Sample], path: &Path) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let xs: Vec<f32> = samples.iter().map(|s| s.pos[0]).collect();
    let alt: Vec<f32> = samples.iter().map(|s| -s.pos[2]).collect();
    std::fs::write(
        path,
        render(&xs, &alt, "x (m, forward)", "altitude (m)", "Side view (x-z)"),
    )
}

fn render(xs: &[f32], ys: &[f32], xlabel: &str, ylabel: &str, title: &str) -> String {
    if xs.is_empty() {
        return format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{WIDTH}\" height=\"{HEIGHT}\">\
             <text x=\"10\" y=\"20\" font-family=\"sans-serif\">no data</text></svg>"
        );
    }

    let (xmn, xmx) = pad(bounds(xs), 0.1, 0.2);
    let (ymn, ymx) = pad(bounds(ys), 0.1, 0.2);

    let plot_w = WIDTH - MARGIN_L - MARGIN_R;
    let plot_h = HEIGHT - MARGIN_T - MARGIN_B;
    let dx = (xmx - xmn).max(1e-6);
    let dy = (ymx - ymn).max(1e-6);
    let px = |x: f32| MARGIN_L + (x - xmn) / dx * plot_w;
    let py = |y: f32| HEIGHT - MARGIN_B - (y - ymn) / dy * plot_h;

    let mut s = String::new();
    let _ = write!(
        s,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{WIDTH}\" height=\"{HEIGHT}\" \
         viewBox=\"0 0 {WIDTH} {HEIGHT}\" font-family=\"sans-serif\" font-size=\"11\">"
    );
    let _ = write!(s, "<rect width=\"{WIDTH}\" height=\"{HEIGHT}\" fill=\"#ffffff\"/>");
    let _ = write!(
        s,
        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#fafafa\" stroke=\"#888\"/>",
        MARGIN_L, MARGIN_T, plot_w, plot_h
    );

    if xmn < 0.0 && xmx > 0.0 {
        let x0 = px(0.0);
        let _ = write!(
            s,
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#ccc\" stroke-dasharray=\"3,3\"/>",
            x0, MARGIN_T, x0, HEIGHT - MARGIN_B
        );
    }
    if ymn < 0.0 && ymx > 0.0 {
        let y0 = py(0.0);
        let _ = write!(
            s,
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#ccc\" stroke-dasharray=\"3,3\"/>",
            MARGIN_L, y0, WIDTH - MARGIN_R, y0
        );
    }

    let _ = write!(
        s,
        "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\" font-size=\"13\">{}</text>",
        WIDTH / 2.0, MARGIN_T - 10.0, title
    );
    let _ = write!(
        s,
        "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\">{}</text>",
        WIDTH / 2.0, HEIGHT - 8.0, xlabel
    );
    let _ = write!(
        s,
        "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\" \
         transform=\"rotate(-90 {:.1} {:.1})\">{}</text>",
        14.0, HEIGHT / 2.0, 14.0, HEIGHT / 2.0, ylabel
    );

    let _ = write!(s, "<text x=\"{:.1}\" y=\"{:.1}\">{:.2}</text>", MARGIN_L, HEIGHT - MARGIN_B + 14.0, xmn);
    let _ = write!(s, "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"end\">{:.2}</text>", WIDTH - MARGIN_R, HEIGHT - MARGIN_B + 14.0, xmx);
    let _ = write!(s, "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"end\">{:.2}</text>", MARGIN_L - 4.0, HEIGHT - MARGIN_B + 4.0, ymn);
    let _ = write!(s, "<text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"end\">{:.2}</text>", MARGIN_L - 4.0, MARGIN_T + 8.0, ymx);

    let mut pts = String::new();
    for (x, y) in xs.iter().zip(ys.iter()) {
        let _ = write!(pts, "{:.2},{:.2} ", px(*x), py(*y));
    }
    let _ = write!(
        s,
        "<polyline fill=\"none\" stroke=\"#2176c7\" stroke-width=\"1.4\" points=\"{pts}\"/>"
    );

    let _ = write!(
        s,
        "<circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"4\" fill=\"#2a8\"/>",
        px(xs[0]), py(ys[0])
    );
    let _ = write!(
        s,
        "<circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"4\" fill=\"#d33\"/>",
        px(*xs.last().unwrap()), py(*ys.last().unwrap())
    );

    let _ = write!(s, "</svg>");
    s
}

fn bounds(xs: &[f32]) -> (f32, f32) {
    let mn = xs.iter().cloned().fold(f32::INFINITY, f32::min);
    let mx = xs.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    (mn, mx)
}

fn pad((mn, mx): (f32, f32), pct: f32, min_span: f32) -> (f32, f32) {
    let span = mx - mn;
    if !span.is_finite() {
        return (-min_span / 2.0, min_span / 2.0);
    }
    if span < min_span {
        let c = (mn + mx) / 2.0;
        return (c - min_span / 2.0, c + min_span / 2.0);
    }
    let p = span * pct;
    (mn - p, mx + p)
}
