use core::{future::{poll_fn, Future}, str::Split};

macro_rules! flight_modes {
    ($($variant:ident),* $(,)?) => {
        flight_modes!{
            DATA = FlightModeData,
            NAME = FlightModeName,
            $($variant),*
        }
    };
    (DATA = $state:ident, NAME = $names:ident, $($variant:ident),* $(,)?) => {
        enum $state {
            $(
                $variant($variant),
            )*
        }

        enum $names {
            $(
                $variant,
            )*
        }

        impl $state {
            async fn run(&mut self) {
                let returned = match self {
                    $(
                        $state::$variant(inner) => inner.run().await,
                    )*
                };

                if let Some(flight_mode) = returned {
                    match flight_mode {
                        $(
                            $names::$variant => {
                                *self = $state::$variant($variant::initialize());
                            }
                        )*
                    }
                }
            }
        }
    };
}

// Use the macro to define the enums and implement the run method
flight_modes!{
    ReturnToLaunch,
    Stabilize,
}

trait FlightMode {
    /// Called once when the flight mode is selected to create its initial
    /// state. This is responsible for ensuring no weird "on transition"
    /// behavior occurs.
    fn initialize() -> Self;
    fn run(&mut self) -> impl Future<Output = Option<FlightModeName>>;
}

struct ReturnToLaunch;

impl FlightMode for ReturnToLaunch {
    fn initialize() -> Self {
        Self
    }

    async fn run(&mut self) -> Option<FlightModeName> {
        info!("Returning to launch");
        None
    }
}

struct Stabilize;

impl FlightMode for Stabilize {
    fn initialize() -> Self {
        Self
    }

    async fn run(&mut self) -> Option<FlightModeName> {
        info!("Returning to launch");
        None
    }
}


fn parse_entry(string: &str) -> Option<()> {

    let mut split = string.split("_");
    match split.next()? {
        "CAL" => parse_cal(&mut split)?,
        "PID" => parse_pid(&mut split)?,
        _ => {}
    }

    Some(())
}

fn parse_cal(split: &mut Split<'_, &str>) -> Option<()> {
    let imu = match split.next()? {
        "IMU0" => 0,
        "IMU1" => 1,
        "IMU2" => 2,
        _ => return None,
    };

    match split.next()? {
        "ID" => {
        }
        _ => return None,
    }

    Some(())
}


macro_rules! param_split {
    ($split:ident => $name:ident {
        $($variant:ident),+ $(,)?
    }) => {
        mod $name{

            enum $name {
                $($variant,)*
            }

            match $split.next()? {
                $(
                    stringify!($variant) => $name::$variant,
                )*
                _ => return None,
            }
        }
    };
}

fn parse_pid(split: &mut Split<'_, &str>) -> Option<()> {
    let con = match split.next()? {
        stringify!(RAT) => 0,
        "ANG" => 1,
        "POS" => 2,
        "VEL" => 2,
        _ => return None,
    };

    let gain = match split.next()? {
        "P" => 0,
        "I" => 1,
        "D" => 2,
        _ => return None,
    };

    let con = param_split!(split => Con {RAT, ANG, POS, VEL});
    let gain = param_split!(split => Gain {P, I, D});

    match (con, gain) {
        (Con::RAT, Gain::P) => {
        
        }
        _ => {}
    }

    Some(())
}
enum Var {
    Bool(bool),
    F64(f64),
    F32(f32),
    U32(u32),
    I32(i32),
    U16(u16),
    I16(i16),
    U8(u8),
    I8(i8),
}

fn main() {
    let param = "PID_RAT_P";

    let mut base = BaseParam {
        cal: CalParams {
            imu0: 0,
            imu1: 0,
            imu2: 0,
        },
        pid: PidParams {
            rate: PidGains {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            angle: PidGains {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            position: PidGains {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            velocity: PidGains {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
        },
        rate: RateLoop {
            pid: PidGains {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            cal: CalParams {
                imu0: 0,
                imu1: 0,
                imu2: 0,
            },
        },
    };

    match base.find("PID_RAT_P") {
        Some(VarMut::f32(param)) => *param = 1.0,
        Some(VarMut::f64(param)) => *param = 1.0,
        Some(VarMut::i32(param)) => *param = 100,
        Some(VarMut::i16(param)) => *param = 100,
        _ => {}
    }
}


#[allow(non_camel_case_types)]
enum VarMut<'a> {
    bool(&'a mut bool),
    f64(&'a mut f64),
    f32(&'a mut f32),
    u64(&'a mut u64),
    i64(&'a mut i64),
    u32(&'a mut u32),
    i32(&'a mut i32),
    u16(&'a mut u16),
    i16(&'a mut i16),
    u8(&'a mut u8),
    i8(&'a mut i8),
}

trait Param {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param>;

    fn find<'a>(&'a mut self, path: &str) -> Option<VarMut<'a>>
    where Self: Sized
    {
        let mut split = path.split("_");
        let mut current = self as &mut dyn Param;

        loop {
            match current.contains(&mut split) {
                Search::Found(var) => return Some(var),
                Search::Continue(next) => current = next,
                Search::NotFound => return None,
            }
        }
    }

    fn entries(&self) -> &'static[&'static str];
}


enum Search<T, C> {
    Found(T),
    NotFound,
    Continue(C),
}

struct BaseParam {
    cal: CalParams,
    pid: PidParams,
    rate: RateLoop,
}

impl Param for BaseParam {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param> {
        match split.next() {
            Some("CAL") => Search::Continue(&mut self.cal),
            Some("PID") => Search::Continue(&mut self.pid),
            Some("RATE") => Search::Continue(&mut self.rate),
            _ => Search::NotFound,
        }
    }

    fn entries(&self) -> &'static[&'static str] {
        &["CAL", "PID", "RATE"]
    }
}

struct RateLoop {
    pid: PidGains,
    cal: CalParams,
}

impl Param for RateLoop {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param> {
        match split.next() {
            Some("PID") => Search::Continue(&mut self.pid),
            Some("CAL") => Search::Continue(&mut self.cal),
            _ => Search::NotFound,
        }
    }

    fn entries(&self) -> &'static[&'static str] {
        &["PID", "CAL"]
    }
}

struct CalParams {
    imu0: u8,
    imu1: u8,
    imu2: u8,
}

impl Param for CalParams {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param> {
        match split.next() {
            Some("IMU0") => Search::Found(VarMut::u8(&mut self.imu0)),
            Some("IMU1") => Search::Found(VarMut::u8(&mut self.imu1)),
            Some("IMU2") => Search::Found(VarMut::u8(&mut self.imu2)),
            _ => Search::NotFound,
        }
    }

    fn entries(&self) -> &'static[&'static str] {
        &["IMU0", "IMU1", "IMU2"]
    }
}

struct PidParams {
    rate: PidGains,
    angle: PidGains,
    position: PidGains,
    velocity: PidGains,
}

impl Param for PidParams {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param> {
        match split.next() {
            Some("RAT") => Search::Continue(&mut self.rate),
            Some("ANG") => Search::Continue(&mut self.angle),
            Some("POS") => Search::Continue(&mut self.position),
            Some("VEL") => Search::Continue(&mut self.velocity),
            _ => Search::NotFound,
        }
    }

    fn entries(&self) -> &'static[&'static str] {
        &["RAT", "ANG", "POS", "VEL"]
    }
}

struct PidGains {
    p: f32,
    i: f32,
    d: f32,
}

impl Param for PidGains {
    fn contains<'a>(&'a mut self, split: &mut Split<'_, &str>) -> Search<VarMut<'a>, &'a mut dyn Param> {
        match split.next() {
            Some("P") => Search::Found(VarMut::f32(&mut self.p)),
            Some("I") => Search::Found(VarMut::f32(&mut self.i)),
            Some("D") => Search::Found(VarMut::f32(&mut self.d)),
            _ => Search::NotFound,
        }
    }

    fn entries(&self) -> &'static[&'static str] {
        &["P", "I", "D"]
    }
}
