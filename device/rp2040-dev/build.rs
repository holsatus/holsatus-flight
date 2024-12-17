fn main() {
    println!("cargo:rustc-linker=flip-link");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-inline-threshold=5");
    println!("cargo:rustc-no-vectorize-loops=true");
}
