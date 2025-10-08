impl Imu6Dof for Imu {
    async fn read_acc(&mut self) -> core::result::Result<[f32; 3], crate::errors::DeviceError> {
        todo!()
    }

    async fn read_gyr(&mut self) -> core::result::Result<[f32; 3], crate::errors::DeviceError> {
        todo!()
    }

    async fn read_acc_gyr(
        &mut self,
    ) -> core::result::Result<
        crate::types::measurements::Imu6DofData<f32>,
        crate::errors::DeviceError,
    > {
        todo!()
    }
}

impl<T: Imu6Dof> Imu6Dof for &mut T {
    async fn read_acc(&mut self) -> core::result::Result<[f32; 3], crate::errors::DeviceError> {
        T::read_acc(self).await
    }

    async fn read_gyr(&mut self) -> core::result::Result<[f32; 3], crate::errors::DeviceError> {
        T::read_gyr(self).await
    }

    async fn read_acc_gyr(
        &mut self,
    ) -> core::result::Result<
        crate::types::measurements::Imu6DofData<f32>,
        crate::errors::DeviceError,
    > {
        T::read_acc_gyr(self).await
    }
}

struct Imu;

// ---------------

use embassy_futures::select::{select, select3, Either, Either3};
use embedded_io_async::{Read, Write};
use mutex::raw_impls::cs::CriticalSectionRawMutex as CSM;

use crate::{
    errors::DeviceError,
    hw_abstraction::Imu6Dof,
    sync::{
        channel::Channel,
        procedure::{Handle, Procedure},
    },
    types::measurements::Imu6DofData,
};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TaskRequest {
    GetParam,
    GetStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TaskResponse {
    GetStatusIdle,
    GetStatusGood,
    Params,
}

impl Into<Response> for TaskResponse {
    fn into(self) -> Response {
        Response::Task(self)
    }
}

struct ImuDriver<IMU: Imu6Dof> {
    imu: IMU,
    acc_cal: Calibration,
    gyr_cal: Calibration,
    message_count: usize,
    request_count: usize,
    imu_error_count: usize,
    channel: &'static Channel<Message, CSM, 1>,
    procedure: &'static Procedure<Request, Response, CSM, 1>,
}

#[derive(Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Calibration {
    offset: Option<[f32; 3]>,
    scale: Option<[f32; 3]>,
}

impl Calibration {
    pub fn apply(&self, mut data: [f32; 3]) -> [f32; 3] {
        if let Some(offset) = self.offset {
            for i in 0..data.len() {
                data[i] = data[i] - offset[i]
            }
        }

        if let Some(scale) = self.scale {
            for i in 0..data.len() {
                data[i] = data[i] * scale[i]
            }
        }

        data
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Message {
    SetAccCalibration(Calibration),
    SetGyrCalibration(Calibration),
    ParamUpdate(ParamUpdate),
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ParamUpdate;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Request {
    Message(Message),
    Task(TaskRequest),
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Response {
    Accepted,
    Rejected,
    Task(TaskResponse),
}

static CHANNEL: Channel<Message, CSM, 1> = Channel::new();
static PROCEDURE: Procedure<Request, Response, CSM, 1> = Procedure::new();

#[derive(Debug, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Error {
    #[error("The request contained improperly formatted data")]
    InvalidRequest,
    #[error("Imu error: {0}")]
    ImuError(#[from] DeviceError),
}

type Result<T> = core::result::Result<T, Error>;

#[embassy_executor::task]
async fn entry(mut imu: Imu) -> ! {
    ImuDriver::new(&mut imu).run().await
}

impl<IMU: Imu6Dof> ImuDriver<IMU> {
    pub fn new(imu: IMU) -> Self {
        static CHANNEL: Channel<Message, CSM, 1> = Channel::new();
        static PROCEDURE: Procedure<Request, Response, CSM, 1> = Procedure::new();

        ImuDriver {
            imu,
            acc_cal: Calibration::default(),
            gyr_cal: Calibration::default(),
            message_count: 0,
            request_count: 0,
            imu_error_count: 0,
            channel: &CHANNEL,
            procedure: &PROCEDURE,
        }
    }

    /// Run the ImuDriver task
    pub async fn run(&mut self) -> ! {
        loop {
            if let Err(error) = self.run_inner().await {
                warn!("Error: {}", error);
            }
        }
    }

    async fn run_inner(&mut self) -> Result<()> {
        match select3(
            self.channel.receive(),
            self.procedure.receive(),
            self.imu.read_acc_gyr(),
        )
        .await
        {
            Either3::First(message) => {
                self.message_count += 1;
                self.handle_message(message).await?;
            }
            Either3::Second((request, handle_opt)) => {
                self.request_count += 1;
                self.handle_request(request, handle_opt).await?;
            }
            Either3::Third(imu_data_res) => match imu_data_res {
                Ok(imu_data) => {
                    self.imu_error_count = 0;
                    self.handle_imu_data(imu_data).await?
                }
                Err(error) => {
                    self.imu_error_count += 1;
                    return Err(error.into());
                }
            },
        }

        Ok(())
    }

    pub async fn handle_message(&mut self, message: Message) -> Result<()> {
        trace!("Received message: {}", message);
        match message {
            Message::SetAccCalibration(cal) => todo!(),
            Message::SetGyrCalibration(cal) => todo!(),
            Message::ParamUpdate(param_update) => self.update_param(param_update),
        }
    }

    pub async fn handle_request(
        &mut self,
        request: Request,
        handle_opt: Option<Handle<'_, Response>>,
    ) -> Result<()> {
        trace!("Received request: {}", request);
        match request {
            Request::Message(message) => self.handle_message(message).await?,
            Request::Task(task_request) => {
                let Some(handle) = handle_opt else {
                    warn!("Received task request without a handle");
                    return Err(Error::InvalidRequest);
                };

                match task_request {
                    TaskRequest::GetStatus => handle.respond(TaskResponse::GetStatusGood),
                    TaskRequest::GetParam => todo!(),
                }
            }
        }

        Ok(())
    }

    pub async fn handle_imu_data(&mut self, imu_data: Imu6DofData<f32>) -> Result<()> {
        trace!("Received IMU data: {}", imu_data);

        Ok(())
    }

    pub fn update_param(&mut self, param_udpate: ParamUpdate) -> Result<()> {
        trace!("Updating param: {}", param_udpate);
        Ok(())
    }
}
