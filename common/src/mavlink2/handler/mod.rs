use mavio::{
    error::SpecError, mavspec::rust::spec::MessageSpecStatic, prelude::MaybeVersioned,
    protocol::Payload, Frame,
};

use crate::mavlink2::MavlinkServer;

pub mod command;
pub mod param;
pub mod vicon;

pub trait Handler<V: MaybeVersioned> {
    async fn handle_inner(
        self,
        server: &mut MavlinkServer,
        frame: Frame<V>,
    ) -> Result<(), super::Error>;

    async fn handle(server: &mut MavlinkServer, frame: Frame<V>) -> Result<(), super::Error>
    where
        for<'a> Self: MessageSpecStatic + TryFrom<&'a Payload, Error = SpecError>,
    {
        let this = frame.decode_message::<Self>()?;
        this.handle_inner(server, frame).await
    }
}
