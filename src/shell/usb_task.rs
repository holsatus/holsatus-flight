use embassy_usb::UsbDevice;

use crate::messaging as msg;

const TASK_ID: &str = "[SHELL USB]";

/// This simple task is responsible for signaling the USB connection state to other tasks.
#[embassy_executor::task]
pub(super) async fn embassy_usb_task(mut usb: UsbDevice<'static, crate::bsp::UsbPeripheral>) -> ! {
    
    let usb_connected = msg::USB_CONNECTED.sender();
    let mut arm_vehicle = msg::CMD_ARM_VEHICLE.receiver().unwrap();

    defmt::info!("{}: Task started", TASK_ID);
    loop {
        
        // Wait to vehicle to be disarmed
        arm_vehicle.get_and(|b|b == &false).await;

        // Wait for the USB to be connected
        defmt::info!("{}: Waiting for USB connection", TASK_ID);
        usb.wait_resume().await;

        // Ensure that the vehicle is still disarmed
        if arm_vehicle.try_get() != Some(false) {
            continue;
        }

        // Signal the USB connection status and run USB until it is suspended
        usb_connected.send(true);
        defmt::info!("{}: USB connection resumed", TASK_ID);
        usb.run_until_suspend().await;
        usb_connected.send(false);
        defmt::info!("{}: USB connection suspended", TASK_ID);
    }
}
