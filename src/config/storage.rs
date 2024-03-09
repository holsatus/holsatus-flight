use embassy_rp::flash::{Flash, Instance, Mode, ERASE_SIZE};

/// Write `data` of type into RP2040 flash storage at address `offset`.
pub unsafe fn write<I, M, T, const F: usize>(flash: &mut Flash<I, M, F>, data: &T, offset: u32)
where
    I: Instance,
    M: Mode,
{
    // Erase the place in memory
    defmt::unwrap!(flash.blocking_erase(offset, offset + ERASE_SIZE as u32));

    // Convert struct into slice of u8
    let data_u8 =
        unsafe { core::slice::from_raw_parts((data as *const T) as *const u8, ::core::mem::size_of::<T>()) };

    // Create a fixed size buffer for entire erased area
    // TODO - Calculate checksum and store it in the first 4 bytes. This will make the read operation safer.
    let mut buf = [0u8; ERASE_SIZE];

    // Fill buffer with u8'ified config
    for (b, u) in buf.iter_mut().zip(data_u8) {
        *b = *u
    }

    // Write buffer to flash
    defmt::unwrap!(flash.blocking_write(offset, &mut buf));
}

/// Read data from RP2040 flash storage at address `offset`, and convert it into type `T`.
/// # Safety
/// This function is unsafe because it uses transmutation to convert the bytes read from
/// flash memory, into a specific type `T`. It is up to the user to ensure that the type
/// is valid. If the type is not valid, it can lead to undefined behavior.
pub unsafe fn read<I, M, T, const F: usize>(flash: &mut Flash<I, M, F>, offset: u32) -> T
where
    I: Instance,
    M: Mode,
    T: Sized,
    [u8; core::mem::size_of::<T>()]:,
{
    // Setup empty buffer for reading into
    let mut data_u8 = [0u8; core::mem::size_of::<T>()];

    // Read memory into buffer
    defmt::unwrap!(flash.blocking_read(offset, &mut data_u8));

    // Force data to become type T
    unsafe { core::mem::transmute_copy(&data_u8) }
}
