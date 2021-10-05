#![no_std]

extern crate nic_buffers;
extern crate dma_buffers;

use nic_buffers::{TransmitBuffer, ReceivedFrame};
use dma_buffers::{Buffer, State};


/// A trait that defines the necessary minimum functions that all network interface card (NIC) drivers
/// should implement. 
pub trait NetworkInterfaceCard {

    /// Sends a packet contained in the given `transmit_buffer` out through this NetworkInterfaceCard. 
    /// Blocks until the packet has been successfully sent by the networking card hardware.
    fn send_packet(&mut self, transmit_buffer: TransmitBuffer) -> Result<(), &'static str>;

    /// Sends a packet contained in the given `transmit_buffer` out through this NetworkInterfaceCard. 
    /// Also, reclaims transmit packet buffers that are no longer in use by the hardware.
    fn send_packet_and_reclaim(&mut self, transmit_buffer: Buffer<{State::SWOwned}>) -> Result<(), &'static str>;

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
