//! Application which checks the functionality of the ixgbe driver by creating multiple virtual NICs
//! and sending and receiving packets with them.
//! 
//! For the receiving functionality, you will have to set up a program on another machine that
//! sends packets to the IP addresses assigned to the virtual NICs.
//! Since we are not going through the network stack, I manually add the (mac address, ip address) pair
//! to the ARP table of the sending machine:  "sudo arp -i interface_name -s ip_address mac_address" 
//! e.g."sudo arp -i eno1 -s 192.168.0.20 0c:c4:7a:d2:ee:1a"

#![no_std]
#[macro_use] extern crate alloc;
// #[macro_use] extern crate log;
#[macro_use] extern crate terminal_print;
extern crate network_interface_card;
extern crate e1000;
extern crate spawn;
extern crate dma_buffers;

use alloc::vec::Vec;
use alloc::string::String;
use network_interface_card::NetworkInterfaceCard;
use dma_buffers::{Buffer, State};

pub fn main(_args: Vec<String>) -> isize {
    println!("Ixgbe test application");

    match rmain() {
        Ok(()) => {
            println!("Ixgbe test was successful");
            return 0;
        }
        Err(e) => {
            println!("Ixgbe test failed with error : {:?}", e);
            return -1;
        }
    }
}

fn rmain() -> Result<(), &'static str> {
    
    
    let mut nic = e1000::get_e1000_nic().ok_or("e1000 nic was not initialized")?.lock();
    let mac_address = nic.mac_address();

    for _ in 0..200{
        let buffer = create_raw_packet(&[0x0, 0x1F, 0xC6, 0x9C, 0xB2, 0x07], &mac_address, &[1;46])?;
        nic.send_packet_and_reclaim(buffer)?;
    }
        Ok(())
}


pub fn create_raw_packet(
    dest_mac_address: &[u8], 
    source_mac_address: &[u8], 
    message: &[u8]
) -> Result<Buffer<{State::SWOwned}>, &'static str> {
    
    const ETHER_TYPE_LEN: usize = 2;
    const MAC_ADDR_LEN: usize = 6;
    const ETHERNET_HEADER_LEN: usize = MAC_ADDR_LEN * 2 + ETHER_TYPE_LEN;
    const MIN_PACKET_LEN: usize = 46;

    let mut len = message.len();
    if len > 1500 {
        return Err("Too long for a raw packet");
    }
    if len < MIN_PACKET_LEN {
        len = 46;
    }

    let ether_type: [u8; ETHER_TYPE_LEN] = [(len >> 8) as u8, len as u8];

    let mut transmit_buffer = Buffer::new(ETHERNET_HEADER_LEN + len)?;
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(0, MAC_ADDR_LEN)?;
        buffer.copy_from_slice(&dest_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(6, MAC_ADDR_LEN)?;
        buffer.copy_from_slice(&source_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(12, ETHER_TYPE_LEN)?;
        buffer.copy_from_slice(&ether_type);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(14, message.len())?;
        buffer.copy_from_slice(&message);
    }

    Ok(transmit_buffer)
}