use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub enum Fw2Pc {
    Heartbeat,
    PingRes,
}

#[derive(Serialize, Deserialize)]
pub enum Pc2Fw {
    Ping,
}

impl Fw2Pc {
    pub fn to_slice<'b>(&self, buf: &'b mut [u8]) -> Result<&'b mut [u8], postcard::Error> {
        postcard::to_slice(self, buf)
    }
}

impl Pc2Fw {
    pub fn to_slice<'b>(&self, buf: &'b mut [u8]) -> Result<&'b mut [u8], postcard::Error> {
        postcard::to_slice(self, buf)
    }
}
