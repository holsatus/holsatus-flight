fn main() {
    println!("cargo:rustc-check-cfg=cfg(usart_v3)");
    println!("cargo:rustc-check-cfg=cfg(usart_v4)");

    println!("cargo:rustc-linker=flip-link");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
