//! An implementation of a shared buffer for IPC that can be used for 1-byte messages.
//! We still need to make the channel generic to use atomics upto AtomicU64

#![no_std]
#![feature(no_more_cas)]
#![feature(naked_functions)]
extern crate alloc;
#[macro_use] extern crate log;
extern crate bit_field;
extern crate pmu_x86;
extern crate task;
extern crate scheduler;
extern crate irq_safety;
extern crate spin;
extern crate volatile;

use core::sync::atomic::{Ordering, AtomicU16};
use alloc::sync::Arc;
use bit_field::BitField;
use pmu_x86::{Counter};
use task::TaskRef;
use scheduler::switch_to;
use irq_safety::hold_interrupts;
use core::cell::RefCell;
use spin::Mutex;
use volatile::Volatile;

/// A channel implemented using a lock-free shared buffer 
struct Channel {
    // The upper 8 bits are the buffer and the LSB is the full flag which indicates
    // whether the buffer has been used and has a message stored in it. 
    buffer: Mutex<Volatile<Option<u8>>>,
}

impl Channel {
    pub fn new() -> Channel {
        Channel{
            buffer: Mutex::new(Volatile::new(None)),
        }
    }
}

/// Channel endpoint that only allows sending messages.
pub struct Client(Arc<Channel>);

impl Client{

    /// Tries to send a message until succesful.
    /// Task will spin in a loop until the full flag is cleared. 
    // #[naked]   
    // #[inline(always)]
    pub fn send(&self, msg: u8, server_task: &TaskRef, client_task: &TaskRef) {
        if self.0.buffer.lock().read().is_none() {
            debug!("sent");
            self.0.buffer.lock().write(Some(msg));
            switch_to(server_task, client_task);
        } else {
            panic!("Something already in buffer at time of send");
        }
    }

    pub fn receive(&self, server_task: &TaskRef, client_task: &TaskRef) -> u8 {
        if self.0.buffer.lock().read().is_some() {
            let res = self.0.buffer.lock().read().unwrap();
            self.0.buffer.lock().write(None);
            debug!("receive");
            // switch_to(server_task, client_task);
            res
        } else {
            panic!("Nothing in buffer at time of receive");
        }
    }
}

/// Channel endpoint that only allows receiving messages.
pub struct Server(Arc<Channel>);

impl Server {

    /// Tries to receive a message once. If the buffer is empty, then returns an Err.
    // #[inline(always)]        
    pub fn receive_reply(&self, msg: u8, client_task: &TaskRef, server_task: &TaskRef) -> u8 {
        if self.0.buffer.lock().read().is_some() {
            let res = self.0.buffer.lock().read().unwrap();
            self.0.buffer.lock().write(Some(msg));
            debug!("reply receive");

            switch_to(client_task, server_task);            
            res
        } else {
            panic!("Nothing in buffer at time of reply receive");
        }
    }
}

/// Creates a new channel and returns the endpoints
pub fn new_channel() -> (Client, Server) {
    let client = Arc::new(Channel::new());
    let server = client.clone();
    (Client(client), Server(server))
}
