//! Helpers for statically allocating buffers that DMA reads or writes.

/// Declare a `static` that backs a DMA transfer.
///
/// On chips whose DMA-facing SRAM lives in a separate, non-cached domain
/// (e.g. the H7 `RAM_D3` / SRAM4 region) the buffer is emitted into the
/// `.ram_d3` section. The device's `memory.x` is responsible for mapping that
/// section into the D3 region, e.g.
///
/// ```ld
/// .ram_d3 (NOLOAD) : ALIGN(4) { KEEP(*(.ram_d3 .ram_d3.*)) } > RAM_D3
/// ```
///
/// On other chips this expands to a plain `static`, so platforms without a
/// D3 region need no linker changes and the buffer simply lives in normal RAM.
#[cfg(feature = "__use_ram_d3")]
#[macro_export]
macro_rules! dma_buffer {
    ($(#[$attr:meta])* $vis:vis static $name:ident : $ty:ty = $init:expr;) => {
        $(#[$attr])*
        #[unsafe(link_section = ".ram_d3")]
        $vis static $name: $ty = $init;
    };
}

/// Declare a `static` that backs a DMA transfer.
///
/// See the `use_ram_d3` variant for details. On chips without a D3 region this
/// is a plain `static` and no linker support is required.
#[cfg(not(feature = "__use_ram_d3"))]
#[macro_export]
macro_rules! dma_buffer {
    ($(#[$attr:meta])* $vis:vis static $name:ident : $ty:ty = $init:expr;) => {
        $(#[$attr])*
        $vis static $name: $ty = $init;
    };
}
