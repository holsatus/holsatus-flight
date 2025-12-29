use core::ops::SubAssign;

use heapless::Deque;
use maitake_sync::{blocking::DefaultMutex, WaitQueue};
use mutex::{BlockingMutex, ConstInit, ScopedRawMutex};

pub struct Broadcast<T, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    state: BlockingMutex<M, State<T, N>>,
    recv_queue: WaitQueue<M>,
    send_queue: WaitQueue<M>,
}

impl<T: Clone, M: ScopedRawMutex + ConstInit, const N: usize> Default for Broadcast<T, N, M> {
    fn default() -> Self {
        Self::new()
    }
}

struct State<T, const N: usize> {
    deque: Deque<Entry<T>, N>,
    message_id: usize,
    recv_count: usize,
}

struct Entry<T> {
    message: T,
    unread_count: usize,
}

impl<T: Clone, M: ScopedRawMutex, const N: usize> Broadcast<T, N, M> {
    /// Creates a new [`Broadcast`] channel with a fixed-size buffer.
    ///
    /// The channel allows multiple [`Receiver`]s to subscribe to messages sent by a [`Sender`].
    /// Messages are stored in a circular buffer of size `N`. If the buffer is full, older messages
    /// will be overwritten when using methods like [`send_immediate`].
    ///
    /// # Requirements
    ///
    /// - The mutex type `M` must implement [`ConstInit`] to allow for static initialization.
    /// - The message type `T` must implement [`Clone`] to allow multiple receivers to access the same message.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone, PartialEq)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    /// let mut receiver = BROADCAST.receiver();
    ///
    /// // Send a message (or async with `sender.send().await`)
    /// assert!(sender.try_send(Message(1)).is_ok());
    ///
    /// // Receive the message (or async with `receiver.receive().await`)
    /// assert!(receiver.try_receive() == Some(Ok(Message(1))));
    /// ```
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            state: BlockingMutex::new(State {
                deque: Deque::new(),
                message_id: 0,
                recv_count: 0,
            }),
            recv_queue: WaitQueue::new_with_raw_mutex(M::INIT),
            send_queue: WaitQueue::new_with_raw_mutex(M::INIT),
        }
    }

    /// Creates a [`Receiver`] handle for this [`Broadcast`] channel.
    ///
    /// The [`Receiver`] keeps track of the latest message index it has received and updates the
    /// channel's state accordingly. To ensure no messages are missed, a task interested in receiving
    /// all messages should maintain a persistent [`Receiver`] handle.
    ///
    /// # Behavior
    ///
    /// - Only messages sent *after* the creation of the [`Receiver`] will be received.
    /// - If the [`Receiver`] is dropped, unread messages for that receiver will be marked as read.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let receiver = BROADCAST.receiver();
    /// ```
    pub fn receiver(&self) -> Receiver<'_, T, N, M> {
        self.state.with_lock(|state| {
            state.recv_count += 1;
            Receiver {
                inner: self,
                messade_id: state.message_id,
            }
        })
    }

    /// Creates a [`Sender`] handle for this [`Broadcast`] channel.
    ///
    /// The [`Sender`] allows sending messages to the channel. Unlike the [`Receiver`], the [`Sender`]
    /// does not maintain any internal state. Messages sent by the [`Sender`] will only be stored
    /// in the buffer if there is at least one active [`Receiver`].
    ///
    /// # Behavior
    ///
    /// - If no [`Receiver`] is registered, messages will be discarded.
    /// - If the buffer is full, sending a message will fail unless using methods like [`send_immediate`].
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    /// ```
    pub fn sender(&self) -> Sender<'_, T, N, M> {
        Sender { inner: self }
    }

    /// Attempts to send a message to the broadcast channel if there is room in the channel's buffer.
    ///
    /// See [`Sender::try_send`] documentation for more information.
    pub fn try_send(&self, message: T) -> Result<(), T> {
        let res = self.state.with_lock(|state| {
            if state.recv_count == 0 {
                return Ok(());
            }

            state
                .deque
                .push_back(Entry {
                    message,
                    unread_count: state.recv_count,
                })
                .map_err(|message| message.message)?;

            state.message_id += 1;

            Ok(())
        });

        if res.is_ok() {
            self.recv_queue.wake_all();
        }

        res
    }

    /// Sends a message to the broadcast channel, waiting for receivers to fetch old messages if necessary.
    ///
    /// See [`Sender::send`] documentation for more information.
    pub async fn send(&self, mut message: T) {
        loop {
            let wait = self.send_queue.wait();
            match self.try_send(message) {
                Ok(()) => break,
                Err(retry) => {
                    let res = wait.await;
                    debug_assert!(res.is_ok());
                    message = retry;
                }
            }
        }
    }

    /// Sends a message to the broadcast channel immediately, overwriting the oldest message if the buffer is full.
    ///
    /// See [`Sender::send_immediate`] documentation for more information.
    pub fn send_immediate(&self, message: T) {
        self.state.with_lock(|state| {
            if state.recv_count == 0 {
                return;
            }

            if state.deque.is_full() {
                state.deque.pop_front();
            }

            // Safety: We just checked that the deque is not full
            debug_assert!(!state.deque.is_full(), "Deque full prior to UNSAFE push");
            unsafe {
                state.deque.push_back_unchecked(Entry {
                    message,
                    unread_count: state.recv_count,
                });
            }

            state.message_id = state.message_id.wrapping_add(1);
        });

        self.recv_queue.wake_all();
    }

    /// Attempts to receive a message from the broadcast channel without blocking.
    ///
    /// See [`Receiver::try_receive`] documentation for more information.
    pub fn try_receive(&self, msg_id: &mut usize) -> Option<Result<T, usize>> {
        self.state.with_lock(|state: &mut State<T, N>| {
            let start_id = state.message_id.wrapping_sub(state.deque.len());

            let msg_id_diff = msg_id.wrapping_sub(start_id);

            if msg_id_diff > state.deque.len() {
                let lagged_by = start_id.wrapping_sub(*msg_id);
                *msg_id = start_id;
                return Some(Err(lagged_by));
            }

            // We've checked that the index is valid
            let entry_mut = state.deque.iter_mut().nth(msg_id_diff)?;

            // We're reading this item, so decrement the counter
            entry_mut.unread_count.sub_assign(1);
            *msg_id = msg_id.wrapping_add(1);

            let message = if msg_id_diff == 0 && entry_mut.unread_count == 0 {
                debug_assert!(
                    state.deque.front().is_some(),
                    "Front entry not present prior to UNSAFE pop"
                );
                let entry = unsafe { state.deque.pop_front_unchecked() };
                self.send_queue.wake();
                // Return pop'd message without clone
                entry.message
            } else {
                entry_mut.message.clone()
            };

            Some(Ok(message))
        })
    }

    /// Receives a message from the broadcast channel, waiting asynchronously if no message is available.
    ///
    /// See [`Receiver::receive`] documentation for more information.
    pub async fn receive(&self, msg_id: &mut usize) -> Result<T, usize> {
        loop {
            let wake = self.recv_queue.wait();
            match self.try_receive(msg_id) {
                Some(value) => break value,
                None => {
                    let res = wake.await;
                    debug_assert!(res.is_ok())
                }
            }
        }
    }
}

pub struct Sender<'a, T, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    inner: &'a Broadcast<T, N, M>,
}

impl<T: Clone, M: ScopedRawMutex, const N: usize> Sender<'_, T, N, M> {
    /// Attempts to send a message to the broadcast channel if there is room in the channel's buffer.
    ///
    /// If the channel's buffer is full, the message will not be sent, and the original message
    /// will be returned as an error. If there are no active receivers, the message will be discarded
    /// without being stored in the buffer.
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the message was successfully sent.
    /// - `Err(T)` if the channel's buffer is full, returning the original message.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone)]
    /// struct Message(u8);
    ///
    /// let broadcast = Broadcast::<Message, 2>::new();
    ///
    /// let sender = broadcast.sender();
    /// let _receiver = broadcast.receiver();
    ///
    /// // Send {2} messages successfully
    /// assert!(sender.try_send(Message(1)).is_ok());
    /// assert!(sender.try_send(Message(2)).is_ok());
    ///
    /// // Attempt to send another message when the buffer is full
    /// assert!(sender.try_send(Message(3)).is_err());
    /// ```
    pub fn try_send(&self, value: T) -> Result<(), T> {
        self.inner.try_send(value)
    }

    /// Sends a message to the broadcast channel, waiting for receivers to fetch old messages if necessary.
    ///
    /// If the channel's buffer is full, and some receivers have not yet received a message, this function
    /// will wait until there is room in the channel. This function will therefore always succeed.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone, PartialEq)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    /// let mut receiver = BROADCAST.receiver();
    ///
    /// # futures_executor::block_on(async {
    /// // Send a message
    /// sender.send(Message(1)).await;
    ///
    /// // Receive the message asynchronously
    /// assert!(receiver.receive().await == Ok(Message(1)));
    /// # });
    /// ```
    pub async fn send(&self, value: T) {
        self.inner.send(value).await
    }

    /// Sends a message to the broadcast channel immediately, overwriting the oldest message if the buffer is full.
    ///
    /// This method does not block and will always send the message, even if the buffer is full.
    /// If the buffer is full, the oldest message will be removed to make room for the new message.
    /// If there are no active receivers, the message will be discarded.
    ///
    /// # Behavior
    ///
    /// - If the buffer is full, the oldest message will be overwritten.
    /// - If no receivers are registered, the message will be discarded.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    ///
    /// // Send a message immediately
    /// sender.send_immediate(Message(1));
    /// ```
    pub fn send_immediate(&self, value: T) {
        self.inner.send_immediate(value);
    }
}

pub struct Receiver<'a, T: Clone, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    messade_id: usize,
    inner: &'a Broadcast<T, N, M>,
}

impl<T: Clone, M: ScopedRawMutex, const N: usize> Receiver<'_, T, N, M> {
    /// Attempts to receive a message from the broadcast channel without blocking.
    ///
    /// This method checks if a message is available in the buffer and returns it immediately if found.
    /// If the message has been overwritten due to buffer overflow, an error is returned indicating
    /// how many messages were missed. If no message is available, `None` is returned.
    ///
    /// # Returns
    ///
    /// - `Some(Ok(T))` if a message was successfully received.
    /// - `Some(Err(usize))` if messages were missed due to buffer overflow, with the number of missed messages.
    /// - `None` if no message is available.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone, PartialEq)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    /// let mut receiver = BROADCAST.receiver();
    ///
    /// // Send a message
    /// sender.send_immediate(Message(1));
    ///
    /// // Try to receive the message
    /// assert!(receiver.try_receive() == Some(Ok(Message(1))));
    /// ```
    pub fn try_receive(&mut self) -> Option<Result<T, usize>> {
        self.inner.try_receive(&mut self.messade_id)
    }

    /// Receives a message from the broadcast channel, waiting asynchronously if no message is available.
    ///
    /// This method waits until a message becomes available in the buffer and then returns it. If the
    /// message has been overwritten due to buffer overflow, an error is returned indicating how many
    /// messages were missed.
    ///
    /// # Returns
    ///
    /// - `Ok(T)` if a message was successfully received.
    /// - `Err(usize)` if messages were missed due to buffer overflow, with the number of missed messages.
    ///
    /// # Example
    ///
    /// ```rust
    /// use common::sync::broadcast::Broadcast;
    ///
    /// #[derive(Clone, PartialEq)]
    /// struct Message(u8);
    ///
    /// static BROADCAST: Broadcast<Message, 2> = Broadcast::new();
    ///
    /// let sender = BROADCAST.sender();
    /// let mut receiver = BROADCAST.receiver();
    ///
    /// // Send a message
    /// sender.send_immediate(Message(1));
    ///
    /// // Receive the message asynchronously
    /// # futures_executor::block_on(async {
    /// assert!(receiver.receive().await == Ok(Message(1)));
    /// # });
    /// ```
    pub async fn receive(&mut self) -> Result<T, usize> {
        self.inner.receive(&mut self.messade_id).await
    }
}

impl<T: Clone, M: ScopedRawMutex, const N: usize> Drop for Receiver<'_, T, N, M> {
    fn drop(&mut self) {
        let mut wake_senders = 0;
        self.inner.state.with_lock(|state| {
            state.recv_count -= 1;

            // All messages that haven't been read yet by this subscriber must have their counter decremented
            let start_id = state.message_id - state.deque.len();
            if self.messade_id >= start_id {
                let msg_id_diff = self.messade_id.wrapping_sub(start_id);
                state
                    .deque
                    .iter_mut()
                    .skip(msg_id_diff)
                    .for_each(|message| message.unread_count -= 1);

                while let Some(message) = state.deque.front() {
                    if message.unread_count > 0 {
                        break;
                    }
                    debug_assert!(
                        state.deque.front().is_some(),
                        "Front entry not present prior to UNSAFE pop"
                    );
                    unsafe { state.deque.pop_front_unchecked() };
                    wake_senders += 1;
                }
            }
        });
        for _ in 0..wake_senders {
            self.inner.send_queue.wake();
        }
    }
}

#[cfg(all(test, not(feature = "defmt")))]
mod tests {
    use super::*;
    use futures::task::SpawnExt;

    #[test]
    fn test_basic_read_write() {
        #[derive(Clone)]
        struct Message;

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let mut rcv = BROADCAST.receiver();

        let snd = BROADCAST.sender();

        // Repeat behavior a few times
        for _ in 0..4 {
            assert!(rcv.try_receive().is_none());

            assert!(snd.try_send(Message).is_ok());
            assert!(snd.try_send(Message).is_ok());
            assert!(snd.try_send(Message).is_err());

            assert!(rcv.try_receive().is_some());
            assert!(rcv.try_receive().is_some());
        }
    }

    #[test]
    fn test_send_immedate_lagged() {
        #[derive(Clone, PartialEq, Debug)]
        struct Message;

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let mut rcv = BROADCAST.receiver();

        let snd = BROADCAST.sender();

        // Repeat behavior a few times
        for i in 1..5 {
            assert!(snd.try_send(Message).is_ok());
            assert!(snd.try_send(Message).is_ok());
            assert!(snd.try_send(Message).is_err());
            for _ in 0..i {
                snd.send_immediate(Message);
            }

            assert_eq!(rcv.try_receive(), Some(Err(i)));
            assert_eq!(rcv.try_receive(), Some(Ok(Message)));
            assert_eq!(rcv.try_receive(), Some(Ok(Message)));
            assert_eq!(rcv.try_receive(), None);
        }
    }

    #[test]
    fn test_send_counted() {
        #[derive(Clone, PartialEq, Debug)]
        struct Message(u8);

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let mut rcv = BROADCAST.receiver();

        let snd = BROADCAST.sender();

        // Repeat behavior a few times
        for _ in 0..4 {
            assert!(snd.try_send(Message(0)).is_ok());
            assert!(snd.try_send(Message(1)).is_ok());
            snd.send_immediate(Message(2));
            assert!(snd.try_send(Message(3)).is_err());

            assert_eq!(rcv.try_receive(), Some(Err(1)));
            assert_eq!(rcv.try_receive(), Some(Ok(Message(1))));
            assert_eq!(rcv.try_receive(), Some(Ok(Message(2))));
            assert_eq!(rcv.try_receive(), None);
        }
    }

    #[test]
    fn test_drop_unregistration() {
        #[derive(Clone, PartialEq, Debug)]
        struct Message(u8);

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let snd = BROADCAST.sender();

        // Repeat behavior a few times
        for _ in 0..4 {
            let mut rcv = BROADCAST.receiver();

            assert!(snd.try_send(Message(0)).is_ok());
            assert!(snd.try_send(Message(1)).is_ok());
            assert!(snd.try_send(Message(2)).is_err());
            snd.send_immediate(Message(2));

            assert_eq!(rcv.try_receive(), Some(Err(1))); // Message(0) got pushed out
            assert_eq!(rcv.try_receive(), Some(Ok(Message(1))));

            assert!(snd.try_send(Message(3)).is_ok());
            assert!(snd.try_send(Message(4)).is_err());

            drop(rcv);
        }
    }

    #[test]
    fn test_multireceivers() {
        #[derive(Clone, PartialEq, Debug)]
        struct Message(u8);

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let snd = BROADCAST.sender();
        let mut rcv1 = BROADCAST.receiver();
        let mut rcv2 = BROADCAST.receiver();

        // Repeat behavior a few times
        for _ in 0..4 {
            assert!(snd.try_send(Message(0)).is_ok());
            assert!(snd.try_send(Message(1)).is_ok());
            assert!(snd.try_send(Message(2)).is_err());

            assert_eq!(rcv1.try_receive(), Some(Ok(Message(0))));
            assert_eq!(rcv1.try_receive(), Some(Ok(Message(1))));

            assert!(snd.try_send(Message(2)).is_err());

            assert_eq!(rcv2.try_receive(), Some(Ok(Message(0))));

            assert!(snd.try_send(Message(2)).is_ok());
            assert!(snd.try_send(Message(3)).is_err());

            assert_eq!(rcv1.try_receive(), Some(Ok(Message(2))));

            assert_eq!(rcv2.try_receive(), Some(Ok(Message(1))));
            assert_eq!(rcv2.try_receive(), Some(Ok(Message(2))));
        }
    }

    #[test]
    fn test_multireceivers_drop() {
        #[derive(Clone, PartialEq, Debug)]
        struct Message(u8);

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        let snd = BROADCAST.sender();
        let mut rcv1 = BROADCAST.receiver();

        // Repeat behavior a few times
        for _ in 0..4 {
            let rcv2 = BROADCAST.receiver();

            assert!(snd.try_send(Message(0)).is_ok());
            assert!(snd.try_send(Message(1)).is_ok());
            assert!(snd.try_send(Message(2)).is_err());

            assert_eq!(rcv1.try_receive(), Some(Ok(Message(0))));
            assert_eq!(rcv1.try_receive(), Some(Ok(Message(1))));

            // Fails because rcv2 has still not seen the message
            assert!(snd.try_send(Message(2)).is_err());

            // Dropping receiver 2 unregisters it
            drop(rcv2);

            // New messages can be sent now
            assert!(snd.try_send(Message(2)).is_ok());
            assert!(snd.try_send(Message(3)).is_ok());
            assert!(snd.try_send(Message(4)).is_err());

            // Able to receive
            assert_eq!(rcv1.try_receive(), Some(Ok(Message(2))));
            assert_eq!(rcv1.try_receive(), Some(Ok(Message(3))));
            assert_eq!(rcv1.try_receive(), None);
        }
    }

    #[test]
    fn test_async_basic_read_write() {
        let mut spawner = futures_executor::LocalPool::new();

        #[derive(Clone, Debug, PartialEq)]
        struct Message(u8);

        static BROADCAST: Broadcast<Message, 2> = Broadcast::new();

        // Register subscriber to make sure first messages can be received
        let mut rcv = BROADCAST.receiver();

        spawner
            .spawner()
            .spawn(async move {
                let snd = BROADCAST.sender();

                for i in 0..10 {
                    snd.send(Message(i)).await;
                }
            })
            .unwrap();

        spawner
            .spawner()
            .spawn(async move {
                for i in 0..10 {
                    let message = rcv.receive().await;
                    assert_eq!(message, Ok(Message(i)));
                }
            })
            .unwrap();

        spawner.run();
    }
}
