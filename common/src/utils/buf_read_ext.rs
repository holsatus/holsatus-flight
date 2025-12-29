use embedded_io::ReadExactError;
use embedded_io_async::BufRead;


#[allow(async_fn_in_trait)]
pub trait BufReadExt: BufRead {
    async fn skip_until(&mut self, delim: u8) -> Result<usize, ReadExactError<Self::Error>> {
        let mut read: usize = 0;
        loop {
            let (done, used) = {
                let available = self.fill_buf().await?;

                if available.is_empty() {
                    return Err(ReadExactError::UnexpectedEof);
                }

                match available.iter().position(|p| *p == delim) {
                    Some(i) => (true, i + 1),
                    None => (false, available.len()),
                }
            };
            self.consume(used);
            read += used;
            if done || used == 0 {
                return Ok(read);
            }
        }
    }
}

impl<R: BufRead> BufReadExt for R {}