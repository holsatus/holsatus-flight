use std::process::Command;

fn main() {
    println!(
        "cargo:rustc-link-search={}",
        std::env::var("CARGO_MANIFEST_DIR").unwrap()
    );

    // Capture git provenance for end-to-end versioning of every flight log.
    // Exported as the GIT_SHA env var so firmware can `env!("GIT_SHA")`.
    // Falls back to "unknown" if not in a git work tree (e.g. tarball build)
    // and appends "-dirty" if there are uncommitted changes at build time.
    //
    // The Makefile's `git-clean` target is the primary guard against flashing
    // uncommitted code; this is a second-line witness written into the binary
    // itself so a log can always be traced to a commit without consulting the
    // host that produced it.
    let sha = Command::new("git")
        .args(["rev-parse", "--short=10", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout).ok()
            } else {
                None
            }
        })
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".to_string());

    // Only check firmware-critical paths for dirtiness, matching the Makefile
    // `git-clean` target's `GIT_CRITICAL_PATHS` list. Without this restriction,
    // the whole-repo `git status --porcelain` tags every build as "-dirty"
    // because of legitimately-uncommitted sibling directories (flight-data/,
    // .bin artefacts, .DS_Store, etc.) that have zero effect on firmware.
    // Keep this list in sync with the Makefile.
    const CRITICAL_PATHS: &[&str] = &[
        "device/micoairh743v2-dev/src",
        "device/micoairh743v2-dev/build.rs",
        "device/micoairh743v2-dev/Cargo.toml",
        "device/micoairh743v2-dev/Cargo.lock",
        "device/micoairh743v2-dev/Makefile",
        "device/micoairh743v2-dev/memory.x",
        "device/micoairh743v2-dev/.cargo",
        "common/src",
        "common/Cargo.toml",
        "common/Cargo.lock",
    ];

    // git status resolves pathspecs relative to CWD; the cargo invocation
    // CWD is the crate dir, but our paths are repo-root-relative. Run from
    // repo root so pathspec resolution matches the Makefile.
    let repo_root = Command::new("git")
        .args(["rev-parse", "--show-toplevel"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout).ok()
            } else {
                None
            }
        })
        .map(|s| s.trim().to_string());

    let dirty = if let Some(root) = &repo_root {
        let mut args: Vec<&str> = vec!["status", "--porcelain", "--"];
        args.extend(CRITICAL_PATHS.iter().copied());
        Command::new("git")
            .current_dir(root)
            .args(&args)
            .output()
            .ok()
            .map(|o| !o.stdout.is_empty())
            .unwrap_or(false)
    } else {
        false
    };

    let version = if dirty {
        format!("{sha}-dirty")
    } else {
        sha
    };
    println!("cargo:rustc-env=GIT_SHA={version}");

    // Rebuild on git state changes. Pointing at HEAD catches new commits;
    // pointing at index catches staging changes (which matter because a
    // staged-but-uncommitted state affects the dirty flag).
    println!("cargo:rerun-if-changed=../../.git/HEAD");
    println!("cargo:rerun-if-changed=../../.git/index");

    emit_odid_secrets();
}

/// Lift operator/aircraft IDs out of `tools/odid_emit/secrets.toml` (gitignored)
/// and into the firmware as build-time env vars. Same file the Python smoke
/// test reads, so identity lives in one place. Missing or partial file is OK:
/// `odid.rs` falls back to placeholders via `option_env!`.
fn emit_odid_secrets() {
    const PATH: &str = "tools/odid_emit/secrets.toml";
    println!("cargo:rerun-if-changed={PATH}");

    let Ok(text) = std::fs::read_to_string(PATH) else { return };

    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        let Some((key, val)) = line.split_once('=') else { continue };
        let val = val.trim().trim_matches('"').trim_matches('\'');
        match key.trim() {
            "operator_id" => println!("cargo:rustc-env=ODID_OPERATOR_ID={val}"),
            "uas_id"      => println!("cargo:rustc-env=ODID_UAS_ID={val}"),
            "description" => println!("cargo:rustc-env=ODID_SELF_ID={val}"),
            _ => {}
        }
    }
}
