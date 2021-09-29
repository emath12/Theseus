#![no_std]

extern crate nic_buffers;
extern crate dma_buffers;

use nic_buffers::{TransmitBuffer, ReceivedFrame};
use dma_buffers::{SWOwnedBuffer, HWOwnedBuffer};


/// A trait that defines the necessary minimum functions that all network interface card (NIC) drivers
/// should implement. 
pub trait NetworkInterfaceCard {
    /// Sends a packet contained in the given `transmit_buffer` out through this NetworkInterfaceCard. 
    /// Blocks until the packet has been successfully sent by the networking card hardware.
    fn send_packet(&mut self, transmit_buffer: TransmitBuffer) -> Result<(), &'static str>;

    /// Returns the earliest `ReceivedFrame`, which is essentially a list of `ReceiveBuffer`s 
    /// that each contain an individual piece of the frame.
    fn get_received_frame(&mut self) -> Option<ReceivedFrame>;

    /// Poll the NIC for received frames. 
    /// Can be used as an alternative to interrupts, or as a supplement to interrupts.
    fn poll_receive(&mut self) -> Result<(), &'static str>;

    /// Returns the MAC address that this NIC is configured with.
    /// If spoofed, it will return the spoofed MAC address, 
    /// otherwise it will return the regular MAC address defined by the NIC hardware.
    fn mac_address(&self) -> [u8; 6];
}

pub trait NetworkInterfaceCard2 {
    /// Sends a packet contained in the given `transmit_buffer` out through this NetworkInterfaceCard. 
    fn send_packet(&mut self, transmit_buffer: SWOwnedBuffer) -> HWOwnedBuffer;

    fn reclaim_packet(&mut self, transmit_buffer: HWOwnedBuffer) -> SWOwnedBuffer;
}
