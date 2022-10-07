// //! Application which checks the functionality of the ixgbe driver by sending 1 packet on each enabled queue,
// //! and then (optionally) endlessly polling the receive queues.
// //! There are two ways to test the nic:
// //! 
// //! 1. Directly accessing the queues through the main ixgbe nic object. This is the default test.
// //! In this case all transmit queues can be tested by sending a packet on each transmit queue, but receive functionality will be tested randomly.
// //! If Receive Side Scaling is enabled then received packets will be randomly forwarded to different receive queues.
// //! If it's disabled, then all packets will be received on queue 0.
// //! 
// //! 2. Creating multiple virtual NICs and sending and receiving packets with them.
// //! In this case all transmit queues can be tested by sending a packet on each transmit queue, and all receive queues can be tested.
// //! Each receive queue is associated with a unique IP address, and packets with that destination IP address will be forwarded to that queue.
// //! 
// //! For the receiving functionality, you will have to set up a program on another machine that sends packets to the IP addresses assigned
// //! to the virtual NICs. Since these are static IP addresses and won't be resolved through ARP, I manually add the (mac address, ip address)
// //! pair to the ARP table of the sending machine:  "sudo arp -i interface_name -s ip_address mac_address" 
// //! e.g."sudo arp -i eno1 -s 192.168.0.20 0c:c4:7a:d2:ee:1a"

#![no_std]
// #[macro_use] extern crate alloc;
// // #[macro_use] extern crate log;
// #[macro_use] extern crate terminal_print;
// extern crate network_interface_card;
// extern crate ixgbe;
// extern crate spawn;
// extern crate getopts;

// use alloc::vec::Vec;
// use alloc::string::String;
// use ixgbe::{
//     virtual_function, get_ixgbe_nics_list,
//     test_packets::create_raw_packet,
//     IXGBE_NUM_RX_QUEUES_ENABLED, IXGBE_NUM_TX_QUEUES_ENABLED
// };
// use network_interface_card::NetworkInterfaceCard;
// use getopts::{Matches, Options};


// const DEST_MAC_ADDR: [u8; 6] = [0xa0, 0x36, 0x9f, 0x1d, 0x94, 0x4c];

// pub fn main(args: Vec<String>) -> isize {

//     let mut opts = Options::new();
//     opts.optflag("h", "help", "print this help menu");
//     opts.optflag("r", "receive", "Test receive functionality. 
//     If this flag is enabled then the program will never terminate since the queues are endlessly polled for incoming packets.");

//     let matches = match opts.parse(&args) {
//         Ok(m) => m,
//         Err(_f) => {
//             println!("{}", _f);
//             print_usage(&opts);
//             return -1; 
//         }
//     };

//     if matches.opt_present("h") {
//         print_usage(&opts);
//         return 0;
//     }

//     println!("Ixgbe batch test application");

//     match rmain(&matches, &opts) {
//         Ok(()) => {
//             println!("Ixgbe test was successful");
//             return 0;
//         }
//         Err(e) => {
//             println!("Ixgbe test failed with error : {:?}", e);
//             return -1;
//         }
//     }
// }

// fn rmain(matches: &Matches, opts: &Options) -> Result<(), &'static str> {
//     let batch_size = 32;

//     let (dev_id, mac_address) = {
//         let ixgbe_devs = get_ixgbe_nics_list().ok_or("Ixgbe NICs list not initialized")?;
//         if ixgbe_devs.is_empty() { return Err("No ixgbe device available"); }
//         let nic = ixgbe_devs[0].lock();
//         (nic.device_id(), nic.mac_address())
//     };

//     let ixgbe_devs = get_ixgbe_nics_list().ok_or("Ixgbe NICs list not initialized")?;
//     if ixgbe_devs.is_empty() { return Err("No ixgbe device available"); }
//     let mut nic = ixgbe_devs[0].lock();
//     for qid in 0..1/*IXGBE_NUM_TX_QUEUES_ENABLED*/ {
//         let mut buffers = Vec::with_capacity(32);
//         for _ in 0..batch_size {
//             buffers.push(create_raw_packet(&DEST_MAC_ADDR, &mac_address, &[1;46])?);
//         }
//         let (pkts_sent, free_buffers) = nic.tx_batch(qid, &mut buffers)?;
//         println!("packets sent  = {}, used buffers = {}", pkts_sent, free_buffers.len());
//     }

//     if matches.opt_present("r") {
//         let mut buffers = Vec::with_capacity(batch_size);
//         // loop {
//             for qid in 0..1/*IXGBE_NUM_RX_QUEUES_ENABLED*/ {
//                 let pkts_received = nic.rx_batch(qid as usize, &mut buffers, batch_size)?;
//                 println!("pkts received = {}, {}", pkts_received, buffers.len());
//             }
//         // }
//     }

//     Ok(())
// }


// fn print_usage(opts: &Options) {
//     println!("{}", opts.usage(USAGE));
// }

// const USAGE: &'static str = "Usage: test_ixgbe [ARGS]
// Checks the receive and transmit functionality of the ixgbe NIC.";
