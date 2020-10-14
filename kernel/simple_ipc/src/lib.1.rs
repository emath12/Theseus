//! An implementation of a shared buffer for IPC that can be used for 1-byte messages.
//! We still need to make the channel generic to use atomics upto AtomicU64

#![no_std]
#![feature(no_more_cas)]
#![feature(naked_functions)]
extern crate alloc;
// #[macro_use] extern crate log;
extern crate bit_field;
extern crate pmu_x86;

use core::sync::atomic::{Ordering, AtomicU16};
use alloc::sync::Arc;
use bit_field::BitField;
use pmu_x86::{Counter};

/// A channel implemented using a lock-free shared buffer 
struct Channel {
    // The upper 8 bits are the buffer and the LSB is the full flag which indicates
    // whether the buffer has been used and has a message stored in it. 
    buffer: AtomicU16,
}

impl Channel {
    pub fn new() -> Channel {
        Channel{
            buffer: AtomicU16::new(0),
        }
    }
}

/// Channel endpoint that only allows sending messages.
pub struct Sender(Arc<Channel>);

impl Sender{

    /// Tries to send a message once. If the buffer is full, then returns an Err.
    #[inline(always)]        
    pub fn try_send(&self, msg: u8) -> Option<u8> {
        self.0.buffer.fetch_update( |val| {
            if !val.get_bit(0) {
                let msg: u16 = ((msg as u16) << 8) | 0x1;
                Some(msg)
            } else {
                None
            }
        }, Ordering::SeqCst, Ordering::SeqCst)
        .map(|prev_val| (prev_val >> 8) as u8 & 0xFF)
        .ok()
    }

    // pub fn try_send(&self, msg: u8) -> Option<u8> {
    //     let prev = self.0.buffer.load(Ordering::SeqCst);
    //     if !prev.get_bit(0) {
    //         self.0.buffer.store((msg as u16) << 8 | 0x1, Ordering::SeqCst);
    //         Some((prev >> 8) as u8 & 0xFF)
    //     } else {
    //         None
    //     }
    // }

    /// Tries to send a message until succesful.
    /// Task will spin in a loop until the full flag is cleared. 
    // #[naked]   
    // #[inline(always)]
    pub fn send(&self, msg: u8) {
        let mut res = Err(0); //self.try_send(msg);
        while res.is_err() {
            res = self.0.buffer.fetch_update( |val| {
                if val & 0x1 == 0x0 {
                    let msg: u16 = ((msg as u16) << 8) | 0x1;
                    Some(msg)
                } else {
                    None
                }
            }, Ordering::Release, Ordering::Relaxed)
        }
    }

    // #[inline(always)]
    // pub fn send(&self, msg: u8) {
    //     let mut res = self.try_send(msg);
    //     while res.is_none() {
    //         res = self.try_send(msg);
    //     }
    // }
}

/// Channel endpoint that only allows receiving messages.
pub struct Receiver(Arc<Channel>);

impl Receiver {

    /// Tries to receive a message once. If the buffer is empty, then returns an Err.
    #[inline(always)]        
    pub fn try_receive(&self) -> Option<u8> {
        self.0.buffer.fetch_update( |val| {
            if val.get_bit(0) {
                Some(0)
            } else {
                None
            }
        }, Ordering::SeqCst, Ordering::SeqCst)
        .map(|msg| (msg >> 8) as u8 & 0xFF)
        .ok()
    }

    // pub fn try_receive(&self) -> Option<u8> {
    //     let msg = self.0.buffer.load(Ordering::SeqCst);
    //     if msg.get_bit(0) {
    //         self.0.buffer.store(0, Ordering::SeqCst);
    //         Some((msg >> 8) as u8 & 0xFF)
    //     } else {
    //         None
    //     }
    // }

    /// Tries to receive a message until succesful.
    /// Task will spin in a loop until the full flag is set.
    // #[naked]   
    // #[inline(always)] 
    pub fn receive(&self) -> u8 {
        let mut res = Err(0); //self.try_receive();
        while res.is_err() {
            res = self.0.buffer.fetch_update( |val| {
                if val & 0x1 == 0x1 {
                    Some(0)
                } else {
                    None
                }
            }, Ordering::Release, Ordering::Relaxed);
        }
        // unwrap is safe here since the condition is checked in the loop
        res.map(|msg| (msg >> 8) as u8).unwrap()
    }

    // #[inline(always)] 
    // pub fn receive(&self) -> u8 {
    //     let mut res = self.try_receive();
    //     while res.is_none() {
    //         res = self.try_receive();
    //     }
    //     // unwrap is safe here since the condition is checked in the loop
    //     res.unwrap()
    // }
}

/// Creates a new channel and returns the endpoints
pub fn new_channel() -> (Sender, Receiver) {
    let sender = Arc::new(Channel::new());
    let receiver = sender.clone();
    (Sender(sender), Receiver(receiver))
}
