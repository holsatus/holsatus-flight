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

    let dirty = Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .ok()
        .map(|o| !o.stdout.is_empty())
        .unwrap_or(false);

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
}
