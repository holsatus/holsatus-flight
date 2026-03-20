
#[macro_export]
macro_rules! get_or_warn {
    ($rcv:ident) => {
        async {
            loop {
                use embassy_time::{with_timeout, Duration};
                match with_timeout(Duration::from_secs(1), $rcv.get()).await {
                    Ok(value) => break value,
                    Err(_) => trace!("{}: Awaiting value for <{}>", ID, stringify!($rcv)),
                }
            }
        }
    };
}

#[macro_export]
macro_rules! const_default {
    ($type:ty => { $($token:tt)+ } ) => {
        impl $crate::utils::const_default::ConstDefault for $type {
            const DEFAULT: Self = Self::const_default();
        }

        impl $type {
            #[allow(unused)]
            pub const fn const_default() -> Self {
                Self { $($token)+ }
            }
        }

        impl Default for $type {
            fn default() -> Self {
                Self::const_default()
            }
        }
    };
    ($type:ty => $($token:tt)+ ) => {
        impl $crate::utils::const_default::ConstDefault for $type {
            const DEFAULT: Self = Self::const_default();
        }

        impl $type {
            pub const fn const_default() -> Self {
                $($token)+
            }
        }

        impl Default for $type {
            fn default() -> Self {
                Self::const_default()
            }
        }
    };
}

pub trait ConstDefault {
    const DEFAULT: Self;
}
