use embassy_rp::flash::{Flash,Instance,Mode,ERASE_SIZE};

unsafe fn any_as_u8<T: Sized>(orig: &T) -> &[u8] {
    core::slice::from_raw_parts(
        (orig as *const T) as *const u8,
        ::core::mem::size_of::<T>(),
    )
}

/// Write `data` of type into RP2040 flash storage at address `offset`.
pub(crate) unsafe fn write<I,M,T,const F: usize>(flash: &mut Flash<I,M,F>, data: T, offset: u32)
where
    I: Instance,
    M: Mode
{

    // Erase the place in memory
    defmt::unwrap!(flash.blocking_erase(offset, offset + ERASE_SIZE as u32));

    // Convert struct into slice of u8
    let data_u8 = core::slice::from_raw_parts(
        (&data as *const T) as *const u8,
        ::core::mem::size_of::<T>(),
    );

    // Create a fixed size buffer for entire erased area
    let mut buf = [0u8; ERASE_SIZE];

    // Fill buffer with u8'ified config
    for (b,u) in buf.iter_mut().zip(data_u8) { *b = *u }

    // Write buffer to flash
    defmt::unwrap!(flash.blocking_write(offset, &mut buf));
}

/// Read data from RP2040 flash storage at address `offset`, and convert it into type `T`.
pub(crate) unsafe fn read<I,M,T,const F: usize>(flash: &mut Flash<I,M,F>, offset: u32) -> T
where
    I: Instance,
    M: Mode,
    T: Sized,
    [u8; core::mem::size_of::<T>()]:
{
    // Setup empty buffer for reading into
    let mut data_u8 = [0u8;core::mem::size_of::<T>()];

    // Read memory into buffer
    defmt::unwrap!(flash.blocking_read(offset, &mut data_u8));

    // Force data to become type T
    core::mem::transmute_copy(&data_u8) // UNSAFE
}