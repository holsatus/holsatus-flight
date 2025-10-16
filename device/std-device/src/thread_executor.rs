use embassy_executor::Executor;
use embassy_executor::SendSpawner;
use std::sync::mpsc;
use std::sync::LazyLock;
use tokio::runtime::Runtime;

pub fn new_spawner() -> Result<SendSpawner, Box<dyn std::error::Error>> {
    let (tx, rx) = mpsc::channel();

    _ = std::thread::spawn(move || {
        let mut executor = Executor::new();

        // SAFETY: since executor.run() never returns, and we throw away the
        // thread handle, we can safely extend the lifetime of the executor.
        let static_executor: &'static mut Executor = unsafe { core::mem::transmute(&mut executor) };
        static_executor.run(|spawner| {
            let spawner = spawner.make_send();
            tx.send(spawner).expect("Failed to send spawner");
        });
    });

    Ok(rx.recv()?)
}

pub static RUNTIME: LazyLock<Runtime> = LazyLock::new(|| {
    let runtime = Runtime::new().expect("Unable to create tokio Runtime");
    Box::leak(Box::new(runtime.enter()));
    runtime
});
