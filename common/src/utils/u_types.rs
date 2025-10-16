use ufmt::{uDisplay, uWrite, uwrite};

pub struct UBuffer<const N: usize> {
    inner: heapless::Vec<u8, N>,
}

impl<const N: usize> UBuffer<N> {
    pub fn new() -> Self {
        Self {
            inner: heapless::Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.inner.clear();
    }

    pub fn bytes(&self) -> &[u8] {
        self.inner.as_slice()
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn push_byte(&mut self, byte: u8) -> Result<(), u8> {
        self.inner.push(byte)
    }
}

impl<const N: usize> uWrite for UBuffer<N> {
    type Error = embedded_io::ErrorKind;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.inner
            .extend_from_slice(s.as_bytes())
            .map_err(|_| embedded_io::ErrorKind::OutOfMemory)
    }
}

fn f32_components(original: f32, precision: u8) -> (bool, i32, u32) {
    #[allow(unused_imports)]
    use num_traits::Float as _;

    let positive = original.is_sign_positive();
    let precision = precision.min(7);
    let factor = 10f32.powi(precision as i32);
    let scaled = original.abs() * factor;
    let rounded = scaled.round();
    let base = (rounded / factor).trunc() as i32;
    let decimals = (rounded % factor).round() as u32;

    (positive, base, decimals)
}

pub struct UFloat(pub f32, pub u8);

impl uDisplay for UFloat {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        #[allow(unused_imports)]
        use num_traits::Float as _;

        let (positive, base, mut decimals) = f32_components(self.0, self.1);

        if positive {
            uwrite!(f, "{}.", base)?;
        } else {
            uwrite!(f, "-{}.", base)?;
        }

        if self.1 == 0 {
            return Ok(());
        }

        let scale = 10u32.pow(self.1 as u32);
        if decimals >= scale {
            decimals = scale - 1;
        }

        let mut divisor = scale / 10;
        while divisor > 0 {
            let digit = decimals / divisor;
            uwrite!(f, "{}", digit)?;
            decimals %= divisor;
            divisor /= 10;
        }

        Ok(())
    }
}
