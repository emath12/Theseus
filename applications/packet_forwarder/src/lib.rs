//! Basic Packet Forwarder that receives as many packets as the PACKETS_LIMIT variable allows, changes the address by 1, and then forwards all the recieved packets back 

// TODO: Ring Buffer?
// Specifying Rx and Tx Devices

#![no_std]
#[macro_use] extern crate alloc;
// #[macro_use] extern crate log;
#[macro_use] extern crate terminal_print;
extern crate irq_safety;
extern crate network_interface_card;
extern crate ixgbe;
extern crate spawn;
extern crate getopts;
extern crate nic_buffers;
extern crate memory;
extern crate memory_structs;
extern crate pci;
extern crate hpet;
#[macro_use] extern crate libtest;
extern crate RingBuffer;

use alloc::vec::Vec;
use alloc::string::String;
use ixgbe::{
    get_ixgbe_nics_list,
    IXGBE_NUM_RX_QUEUES_ENABLED, IXGBE_NUM_TX_QUEUES_ENABLED, IxgbeNic,
};
use network_interface_card::NetworkInterfaceCard;
use getopts::{Matches, Options};
use nic_buffers::{ReceivedFrame, TransmitBuffer, ReceiveBuffer};
use memory::{MappedPages};
use memory_structs::{PhysicalAddress};
use pci::{PciLocation};
use hpet::get_hpet;
use irq_safety::MutexIrqSafeGuard;
use libtest::{hpet_timing_overhead, hpet_2_ns, calculate_stats, check_myrq};
use RingBuffer::Buffer;

const BATCH_SIZE: usize = 32;

pub fn main(args: Vec<String>) -> isize {
    let mut opts = Options::new();
    opts.optflag("h", "help", "prints the help menu");
    opts.optflag("c", "commence", "starts the operation");

    let matches = match opts.parse(&args) {
        Ok(m) => m,
        Err(_) => {
            println!("invalid flag!");
            return -1;
        }
    };

    if matches.opt_present("h") {
        println!("Run with flag -c to commence the packet forwarding");
        return 0;
    }

    match rmain(&matches, &opts) {
        Ok(()) => {
            println!("Packet forwarding successful");
            return 0;
        }
        Err(e) => {
            println!("Packet forwarding failed with error: {:?}", e);
            return -1;
        }

    }

}

fn rmain(matches: &Matches, opts: &Options) -> Result<(), &'static str> {
    println!("Waiting to receive packets...");

    if matches.opt_present("commence") {

        let mut start_hpet: u64 = 0;
        let mut end_hpet: u64;
        let mut delta_hpet: u64 = 0;

        if IXGBE_NUM_RX_QUEUES_ENABLED != IXGBE_NUM_TX_QUEUES_ENABLED {
            return Err("Unequal number of Rx and Tx queues enabled")
        }

        let nic = get_ixgbe_nics_list().ok_or("IXGBE NICs list not initialized")?;
        println!("{} ", nic.len());
        let mut nic = nic[0].lock();
        let mut received_frames: Vec<Buffer> = Vec::with_capacity(BATCH_SIZE);

        IxgbeNic::clear_stats(&nic.regs2);
        
        let hpet = get_hpet().ok_or("Could not retrieve hpet counter")?;

        loop {

            // Returns the Rx and Tx statistics in the form: (Good Rx packets, Good Rx bytes, Good Tx packets, Good Tx bytes).

            if delta_hpet >= 1_000_000_000 as u64 {
                let stats = &nic.get_stats();
                println!("RX: {} Gb/s", (stats.1 * 8) / (1_000_000_000));
                println!("TX: {} Gb/s", (stats.3 * 8) / (1_000_000_000));
                println!("\n");
                delta_hpet = 0;
            }

            start_hpet = hpet.get_counter();

            forward(& mut nic, & mut received_frames, 0)?;

            end_hpet = hpet.get_counter();

            delta_hpet = hpet_2_ns(end_hpet - start_hpet) + delta_hpet;

            // println!("{} ns elapsed", delta_hpet);

        }
    }
    Ok(())
}

fn forward(
    nic: &mut MutexIrqSafeGuard<IxgbeNic>,
    mut buffers: & mut Vec<Buffer>,
    qid: usize
) -> Result<(), &'static str> {
    let pool = nic.rx_queues[qid].rx_buffer_pool;
    
    let new_bufs = nic.rx_batch(qid, buffers, BATCH_SIZE).unwrap();
    
    for frame in buffers.iter_mut() {
        let m = & mut frame.mp;
        let m: &mut [u8] = m.as_slice_mut(0, 16)?;
        m[0] += 1;
    }

    let (_pkts_sent, buffer_shells) = nic.tx_batch(qid, & mut buffers).unwrap();  
    
    // for buffer in buffer_shells {
    //     if let Err(_e) = pool.push(buffer) {
    //         println!("error adding to pool")

    //     }    
    // }

    Ok(())
}

fn two_forward(
    rx_nic: &mut MutexIrqSafeGuard<IxgbeNic>,
    tx_nic: &mut MutexIrqSafeGuard<IxgbeNic>,
    mut buffers: & mut Vec<Buffer>,
    qid: usize
) -> Result<(), &'static str> {
    let pool = rx_nic.rx_queues[qid].rx_buffer_pool;
    
    let new_bufs = rx_nic.rx_batch(qid, buffers, BATCH_SIZE).unwrap();
    
    println!("{} new buffers were init'd", new_bufs);
    
    for frame in buffers.iter_mut() {
        let m = & mut frame.mp;
        let m: &mut [u8] = m.as_slice_mut(0, 16)?;
        m[0] += 1;
    }

    let (_pkts_sent, buffer_shells) = tx_nic.tx_batch(qid, & mut buffers).unwrap(); 

    Ok(())
}


