#![no_std]

extern crate alloc;
#[macro_use] extern crate log;
extern crate memory;
extern crate mpmc;
extern crate intel_ethernet;
extern crate nic_buffers;
extern crate owning_ref;
extern crate nic_queues;
extern crate irq_safety;
extern crate ethernet_smoltcp_device;
extern crate smoltcp;
extern crate network_manager; 


use ethernet_smoltcp_device::EthernetDevice;
use network_manager::{NetworkInterface};
use smoltcp::{
    socket::SocketSet,
    time::Instant,
    phy::DeviceCapabilities,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
    iface::{EthernetInterface, EthernetInterfaceBuilder, NeighborCache, Routes},
};
use core::ops::{Deref, DerefMut};
use memory::{PhysicalAddress, MappedPages, EntryFlags, create_contiguous_mapping};
use owning_ref::BoxRefMut;
use alloc::{
    vec::Vec,
    collections::VecDeque, boxed::Box
};
use intel_ethernet::descriptors::{RxDescriptor, TxDescriptor};
use nic_buffers::{ReceiveBuffer, ReceivedFrame, TransmitBuffer};
use nic_queues::{RxQueueRegisters, NIC_MAPPING_FLAGS, TxQueueRegisters, RxQueue};
use irq_safety::MutexIrqSafe;


pub struct AcquiredFrame(pub Vec<Buffer>);

pub struct RingRxQueue<S: RxQueueRegisters, T: RxDescriptor> {
    /// The number of the queue, stored here for our convenience.
    pub id: u8,
    /// Registers for this receive queue
    pub regs: S,
    /// Receive descriptors
    pub rx_descs: BoxRefMut<MappedPages, [T]>,
    /// The number of receive descriptors in the descriptor ring
    pub num_rx_descs: u16,
    /// Current receive descriptor index
    pub rx_cur: u16,
    /// The list of rx buffers, in which the index in the vector corresponds to the index in `rx_descs`.
    /// For example, `rx_bufs_in_use[2]` is the receive buffer that will be used when `rx_descs[2]` is the current rx descriptor (rx_cur = 2).
    pub rx_bufs_in_use: Vec<Buffer>,
    pub rx_buffer_size_bytes: u16,
    /// The queue of received Ethernet frames, ready for consumption by a higher layer.
    /// Just like a regular FIFO queue, newly-received frames are pushed onto the back
    /// and frames are popped off of the front.
    /// Each frame is represented by a Vec<ReceiveBuffer>, because a single frame can span multiple receive buffers.
    /// TODO: improve this? probably not the best cleanest way to expose received frames to higher layers   
    pub received_frames: VecDeque<AcquiredFrame>,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything, but we use this value when setting the cpu id for interrupts and DCA.
    pub cpu_id: Option<u8>,
    /// Pool where `ReceiveBuffer`s are stored.
    pub rx_buffer_pool: &'static mpmc::Queue<Buffer>,
    /// The filter id for the physical NIC filter that is set for this queue
    pub filter_num: Option<u8>
}

pub trait RingNetworkFunctions {
    fn send_packet(&mut self, transmit_buffer: Buffer) -> Result<(), &'static str>;

    /// Returns the earliest `ReceivedFrame`, which is essentially a list of `ReceiveBuffer`s 
    /// that each contain an individual piece of the frame.
    fn get_received_frame(&mut self) -> Option<AcquiredFrame>;

    /// Poll the NIC for received frames. 
    /// Can be used as an alternative to interrupts, or as a supplement to interrupts.
    fn poll_receive(&mut self) -> Result<(), &'static str>;

    fn mac_address(&self) -> [u8; 6];
}

pub struct Buffer {
    pub mp: MappedPages, 
    pub phys_addr: PhysicalAddress,
    pub length: u16,
    pool: &'static mpmc::Queue<Buffer>,
}

impl Buffer {
    pub fn new(
        mp: MappedPages, 
        phys_addr: PhysicalAddress, 
        length: u16, 
        pool: &'static mpmc::Queue<Buffer>
    )  -> Buffer {
        Buffer {
            mp: mp,
            phys_addr: phys_addr,
            length: length,
            pool: pool
        }
    }

}

impl Drop for Buffer {
    fn drop(& mut self) {
        let new_buff: Buffer = Buffer {
            mp: core::mem::replace(&mut self.mp, MappedPages::empty()),
            phys_addr: self.phys_addr,
            length: 0,
            pool: self.pool,
        };

        if let Err(_e) = self.pool.push(new_buff) {
            error!("NIC: couldn't return dropped Buffer to pool, buf length: {}, phys_addr: {:#X}", _e.length, _e.phys_addr);
        }
    }
}

impl Deref for Buffer {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}

impl DerefMut for Buffer {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}

impl<S: RxQueueRegisters, T: RxDescriptor> RingRxQueue<S,T> {
    /// Polls the queue and removes all received packets from it.
    /// The received packets are stored in the receive queue's `received_frames` FIFO queue.
    pub fn poll_queue_and_store_received_packets_as_ring(&mut self) -> Result<(), &'static str> {
        let mut cur = self.rx_cur as usize;
       
        let mut receive_buffers_in_frame: Vec<Buffer> = Vec::new();
        let mut _total_packet_length: u16 = 0;

        while self.rx_descs[cur].descriptor_done() {
            // get information about the current receive buffer
            let length = self.rx_descs[cur].length();
            _total_packet_length += length as u16;
            // error!("poll_queue_and_store_received_packets {}: received descriptor of length {}", self.id, length);
            
            // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
            // (because we're saving it for higher layers to use),
            // we need to obtain a new `ReceiveBuffer` and set it up such that the NIC will use it for future receivals.
            let new_receive_buf = match self.rx_buffer_pool.pop() {
                Some(rx_buf) => rx_buf,
                None => {
                    warn!("NIC RX BUF POOL WAS EMPTY.... reallocating! This means that no task is consuming the accumulated received ethernet frames.");
                    // if the pool was empty, then we allocate a new receive buffer
                    let len = self.rx_buffer_size_bytes;
                    let (mp, phys_addr) = create_contiguous_mapping(len as usize, NIC_MAPPING_FLAGS)?;
                    Buffer::new(mp, phys_addr, len, self.rx_buffer_pool)
                }
            };

            // actually tell the NIC about the new receive buffer, and that it's ready for use now
            self.rx_descs[cur].set_packet_address(new_receive_buf.phys_addr);

            // Swap in the new receive buffer at the index corresponding to this current rx_desc's receive buffer,
            // getting back the receive buffer that is part of the received ethernet frame
            self.rx_bufs_in_use.push(new_receive_buf);
            let mut current_rx_buf = self.rx_bufs_in_use.swap_remove(cur); 
            current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
            receive_buffers_in_frame.push(current_rx_buf);

            // move on to the next receive buffer to see if it's ready for us to take
            self.rx_cur = (cur as u16 + 1) % self.num_rx_descs;
            self.regs.set_rdt(cur as u32); 

            if self.rx_descs[cur].end_of_packet() {
                let buffers = core::mem::replace(&mut receive_buffers_in_frame, Vec::new());
                self.received_frames.push_back(AcquiredFrame(buffers));
            } else {
                warn!("NIC::poll_queue_and_store_received_packets(): Received multi-rxbuffer frame, this scenario not fully tested!");
            }
            self.rx_descs[cur].reset_status();
            cur = self.rx_cur as usize;
        }

        Ok(())
    }

    pub fn return_frame(&mut self) -> Option<AcquiredFrame> {
        self.received_frames.pop_front()
    }
}

pub fn init_ring_rx_queue<T: RxDescriptor, S:RxQueueRegisters>(num_desc: usize, rx_buffer_pool: &'static mpmc::Queue<Buffer>, buffer_size: usize, rxq_regs: &mut S)
    -> Result<(BoxRefMut<MappedPages, [T]>, Vec<Buffer>), &'static str> 
{    
    let size_in_bytes_of_all_rx_descs_per_queue = num_desc * core::mem::size_of::<T>();
    
    // Rx descriptors must be 128 byte-aligned, which is satisfied below because it's aligned to a page boundary.
    let (rx_descs_mapped_pages, rx_descs_starting_phys_addr) = create_contiguous_mapping(size_in_bytes_of_all_rx_descs_per_queue, NIC_MAPPING_FLAGS)?;

    // cast our physically-contiguous MappedPages into a slice of receive descriptors
    let mut rx_descs = BoxRefMut::new(Box::new(rx_descs_mapped_pages)).try_map_mut(|mp| mp.as_slice_mut::<T>(0, num_desc))?;

    // now that we've created the rx descriptors, we can fill them in with initial values
    let mut rx_bufs_in_use: Vec<Buffer> = Vec::with_capacity(num_desc);
    for rd in rx_descs.iter_mut()
    {
        // obtain or create a receive buffer for each rx_desc
        let rx_buf = rx_buffer_pool.pop()
            .ok_or("Couldn't obtain a ReceiveBuffer from the pool")
            .or_else(|_e| {
                create_contiguous_mapping(buffer_size, NIC_MAPPING_FLAGS)
                    .map(|(buf_mapped, buf_paddr)| 
                        Buffer::new(buf_mapped, buf_paddr, buffer_size as u16, rx_buffer_pool)
                    )
            })?;
        let paddr_buf = rx_buf.phys_addr;
        rx_bufs_in_use.push(rx_buf); 


        rd.init(paddr_buf); 
    }

    // debug!("intel_ethernet::init_rx_queue(): phys_addr of rx_desc: {:#X}", rx_descs_starting_phys_addr);
    let rx_desc_phys_addr_lower  = rx_descs_starting_phys_addr.value() as u32;
    let rx_desc_phys_addr_higher = (rx_descs_starting_phys_addr.value() >> 32) as u32;
    
    // write the physical address of the rx descs ring
    rxq_regs.set_rdbal(rx_desc_phys_addr_lower);
    rxq_regs.set_rdbah(rx_desc_phys_addr_higher);

    // write the length (in total bytes) of the rx descs array
    rxq_regs.set_rdlen(size_in_bytes_of_all_rx_descs_per_queue as u32); // should be 128 byte aligned, minimum 8 descriptors
    
    // Write the head index (the first receive descriptor)
    rxq_regs.set_rdh(0);
    rxq_regs.set_rdt(0);   

    Ok((rx_descs, rx_bufs_in_use))        
}

pub fn init_ring_rx_buf_pool(num_rx_buffers: usize, buffer_size: u16, rx_buffer_pool: &'static mpmc::Queue<Buffer>) -> Result<(), &'static str> {
    let length = buffer_size;
    for _i in 0..num_rx_buffers {
        let (mp, phys_addr) = create_contiguous_mapping(length as usize, NIC_MAPPING_FLAGS)?; 
        let rx_buf = Buffer::new(mp, phys_addr, length, rx_buffer_pool);
        if rx_buffer_pool.push(rx_buf).is_err() {
            // if the queue is full, it returns an Err containing the object trying to be pushed
            error!("intel_ethernet::init_rx_buf_pool(): rx buffer pool is full, cannot add rx buffer {}!", _i);
            return Err("nic rx buffer pool is full");
        };
    }

    Ok(())
}

pub struct RingTxQueue<S: TxQueueRegisters, T: TxDescriptor> {
    /// The number of the queue, stored here for our convenience.
    pub id: u8,
    /// Registers for this transmit queue
    pub regs: S,
    /// Transmit descriptors 
    pub tx_descs: BoxRefMut<MappedPages, [T]>,
    /// The number of transmit descriptors in the descriptor ring
    pub num_tx_descs: u16,
    /// Current transmit descriptor index
    pub tx_cur: u16,
    /// first descriptor that has been used but not checked for transmit completion
    pub tx_clean: u16,
    /// The list of tx buffers, in which the index in the vector corresponds to the index in `tx_descs`.
    /// For example, `tx_bufs_in_use[2]` is the transmit buffer that that is being sent using `tx_descs[2]`.
    /// This field may not be used if we wait for completion of each transmission, and so don't have to store the buffers. 
    pub tx_bufs_in_use: VecDeque<Buffer>,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything but we use this value when setting the cpu id for interrupts and DCA.
    pub cpu_id : Option<u8>
}

impl<S: TxQueueRegisters, T: TxDescriptor> RingTxQueue<S,T> {
    /// Sends a packet on the transmit queue
    /// 
    /// # Arguments:
    /// * `transmit_buffer`: buffer containing the packet to be sent
    pub fn send_on_queue(&mut self, transmit_buffer: Buffer) {
        self.tx_descs[self.tx_cur as usize].send(transmit_buffer.phys_addr, transmit_buffer.length);  
        // update the tx_cur value to hold the next free descriptor
        let old_cur = self.tx_cur;
        self.tx_cur = (self.tx_cur + 1) % self.num_tx_descs;
        // update the tdt register by 1 so that it knows the previous descriptor has been used
        // and has a packet to be sent
        self.regs.set_tdt(self.tx_cur as u32);
        // Wait for the packet to be sent
        self.tx_descs[old_cur as usize].wait_for_packet_tx();
    }
}

pub trait PhysicalRingNic<S: RxQueueRegisters, T: RxDescriptor, U: TxQueueRegisters, V: TxDescriptor> {
    /// Returns the `RxQueue`s owned by a virtual NIC back to the physical NIC.
    fn return_rx_queues(&mut self, queues: Vec<RingRxQueue<S,T>>);
    /// Returns the `TxQueue`s owned by a virtual NIC back to the physical NIC.
    fn return_tx_queues(&mut self, queues: Vec<RingTxQueue<U,V>>);
}



/// A structure that contains a set of `RxQueue`s and `TxQueue`s that can be used to send and receive packets.
pub struct IxgbeVirtualNic<S: RxQueueRegisters + 'static, T: RxDescriptor + 'static, U: TxQueueRegisters + 'static, V: TxDescriptor + 'static> {
    /// The virtual NIC id is set to the id of its first receive queue
    id: u8, 
    /// Set of `RxQueue`s assigned to a virtual NIC
    rx_queues: Vec<RingRxQueue<S,T>>,
    /// The queue that a packet is received on if no other queue is specified
    default_rx_queue: usize,
    /// Set of `TxQueue`s assigned to a virtual NIC
    tx_queues: Vec<RingTxQueue<U,V>>,
    /// The queue that a packet is sent on if no other queue is specified
    default_tx_queue: usize,
    /// MAC address of the NIC
    mac_address: [u8; 6],
    /// Reference to the physical NIC that Rx/Tx queues will be returned to.
    physical_nic_ref: &'static MutexIrqSafe<dyn PhysicalRingNic<S,T,U,V>>
}

impl<S: RxQueueRegisters, T: RxDescriptor, U: TxQueueRegisters, V: TxDescriptor> IxgbeVirtualNic<S,T,U,V> {
    /// Create a new `VirtualNIC` with the given parameters.
    /// For now we require that there is at least one Rx and one Tx queue.
    pub fn new(
        rx_queues: Vec<RingRxQueue<S,T>>,
        default_rx_queue: usize, 
        tx_queues: Vec<RingTxQueue<U,V>>,
        default_tx_queue: usize, 
        mac_address: [u8; 6], 
        physical_nic_ref: &'static MutexIrqSafe<dyn PhysicalRingNic<S,T,U,V>>
    ) -> Result<IxgbeVirtualNic<S,T,U,V>, &'static str> {

        if rx_queues.is_empty() || tx_queues.is_empty() { 
            return Err("Must have at least one Rx and Tx queue to create virtual NIC");
        }

        Ok(IxgbeVirtualNic {
            id: rx_queues[0].id,
            rx_queues,
            default_rx_queue,
            tx_queues,
            default_tx_queue,
            mac_address,
            physical_nic_ref
        })
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    /// Send a packet on the specified queue.
    #[allow(dead_code)]
    pub fn send_packet_on_queue(&mut self, qid: usize, transmit_buffer: Buffer) -> Result<(), &'static str> {
        if qid >= self.tx_queues.len() { return Err("Invalid qid"); }
        self.tx_queues[qid].send_on_queue(transmit_buffer);
        Ok(())
    }

    /// Retrieve a received frame from the specified queue.
    #[allow(dead_code)]
    fn get_received_frame_from_queue(&mut self, qid: usize) -> Result<AcquiredFrame, &'static str> {
        if qid >= self.rx_queues.len() { return Err("Invalid qid"); }
        // return one frame from the queue's received frames
        self.rx_queues[qid].received_frames.pop_front().ok_or("No frames received")
    }

    /// Poll the specified queue to check if any packets have been received.
    #[allow(dead_code)]
    fn poll_receive_queue(&mut self, qid: usize) -> Result<(), &'static str> {
        if qid >= self.rx_queues.len() { return Err("Invalid qid"); }
        self.rx_queues[qid].poll_queue_and_store_received_packets_as_ring()?;
        Ok(())
    }
}

impl<S: RxQueueRegisters, T: RxDescriptor, U: TxQueueRegisters, V: TxDescriptor> RingNetworkFunctions for IxgbeVirtualNic<S,T,U,V> {
    fn send_packet(&mut self, transmit_buffer: Buffer) -> Result<(), &'static str> {
        // by default, when using the physical NIC interface, we send on queue 0.
        let qid = 0;
        self.tx_queues[qid].send_on_queue(transmit_buffer);
        Ok(())
    }

    fn get_received_frame(&mut self) -> Option<AcquiredFrame> {
        // by default, when using the physical NIC interface, we receive on queue 0.
        let qid = 0;
        // return one frame from the queue's received frames
        self.rx_queues[qid].received_frames.pop_front()
    }

    fn poll_receive(&mut self) -> Result<(), &'static str> {
        // by default, when using the physical NIC interface, we receive on queue 0.
        let qid = 0;
        self.rx_queues[qid].poll_queue_and_store_received_packets_as_ring()
    }

    fn mac_address(&self) -> [u8; 6] {
        self.mac_address
    }
}

impl<S: RxQueueRegisters, T: RxDescriptor, U: TxQueueRegisters, V: TxDescriptor> Drop for IxgbeVirtualNic<S,T,U,V> {
    // Right now we assume that a `virtualNIC` is only dropped when all packets have been removed from queues.
    // TODO: check that queues are empty before returning to the NIC.
    fn drop(&mut self) {
        // get access to the physical NIC
        let mut nic = self.physical_nic_ref.lock();
        // remove queues from virtual NIC
        let mut rx_queues = Vec::new();
        let mut tx_queues = Vec::new();
        core::mem::swap(&mut rx_queues, &mut self.rx_queues);
        core::mem::swap(&mut tx_queues, &mut self.tx_queues);
        // return queues to physical NIC
        nic.return_rx_queues(rx_queues);
        nic.return_tx_queues(tx_queues);
    }
}
   


/// To connect the ethernet driver to smoltcp, 
/// we implement transmit and receive callbacks
/// that allow smoltcp to interact with the NIC.
/// 
// impl<'d, N: RingNetworkFunctions + 'static> smoltcp::phy::Device<'d> for EthernetDevice<N> {

//     /// The buffer type returned by the receive callback.
//     type RxToken = RxToken;
//     /// The buffer type returned by the transmit callback.x
//     type TxToken = TxToken<N>;

//     fn capabilities(&self) -> DeviceCapabilities {
//         let mut caps = DeviceCapabilities::default();
//         caps.max_transmission_unit = DEFAULT_MTU;
//         caps
//     }

//     fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
//         // According to the smoltcp code, AFAICT, this function should poll the ethernet driver
//         // to see if a new packet (Ethernet frame) has arrived, and if so, 
//         // take ownership of it and return it inside of an RxToken.
//         // Otherwise, if no new packets have arrived, return None.
//         let received_frame = {
//             let mut nic = self.nic_ref.lock();
//             nic.poll_receive().map_err(|_e| {
//                 error!("EthernetDevice::receive(): error returned from poll_receive(): {}", _e);
//                 _e
//             }).ok()?;
//             nic.get_received_frame()?
//         };

//         // debug!("EthernetDevice::receive(): got Ethernet frame, consists of {} ReceiveBuffers.", received_frame.0.len());
//         // TODO FIXME: add support for handling a frame that consists of multiple ReceiveBuffers
//         if received_frame.0.len() > 1 {
//             error!("EthernetDevice::receive(): WARNING: Ethernet frame consists of {} ReceiveBuffers, we currently only handle a single-buffer frame, so this may not work correctly!",  received_frame.0.len());
//         }

//         let first_buf_len = received_frame.0[0].length;
//         let rxbuf_byte_slice = BoxRefMut::new(Box::new(received_frame))
//             .try_map_mut(|rxframe| rxframe.0[0].as_slice_mut::<u8>(0, first_buf_len as usize))
//             .map_err(|e| {
//                 error!("EthernetDevice::receive(): couldn't convert receive buffer of length {} into byte slice, error {:?}", first_buf_len, e);
//                 e
//             })
//             .ok()?;

//         // Just create and return a pair of (receive token, transmit token), 
//         // the actual rx buffer handling is done in the RxToken::consume() function
//         Some((
//             RxToken(rxbuf_byte_slice),
//             TxToken {
//                 nic_ref: self.nic_ref,
//             },
//         ))
//     }

//     fn transmit(&mut self) -> Option<Self::TxToken> {
//         // Just create and return a transmit token, 
//         // the actual tx buffer creation is done in the TxToken::consume() function.
//         // Also, we can't accurately create an actual transmit buffer here
//         // because we don't yet know its required length.
//         Some(TxToken {
//             nic_ref: self.nic_ref,
//         })
//     }
// }


// /// The transmit token type used by smoltcp, which contains only a reference to the relevant NIC 
// /// because the actual transmit buffer is allocated lazily only when it needs to be consumed.
// pub struct TxToken<N: RingNetworkFunctions + 'static> {
//     nic_ref: &'static MutexIrqSafe<N>,
// }
// impl<N: RingNetworkFunctions + 'static> smoltcp::phy::TxToken for TxToken<N> {
//     fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> smoltcp::Result<R>
//         where F: FnOnce(&mut [u8]) -> smoltcp::Result<R>
//     {
//         // According to the smoltcp documentation, this function must obtain a transmit buffer 
//         // with the requested `length` (or one at least that big) and then return it. 
//         // Because we can dynamically allocate transmit buffers, we just do that here.
//         if len > (u16::max_value() as usize) {
//             error!("EthernetDevice::transmit(): requested tx buffer size {} exceeds the max size of u16!", len);
//             return Err(smoltcp::Error::Exhausted)
//         }

//         // debug!("EthernetDevice::transmit(): creating new TransmitBuffer of {} bytes, timestamp: {}", len, _timestamp);
//         // create a new TransmitBuffer, cast it as a slice of bytes, call the passed `f` closure, and then send it!
//         let mut txbuf = Buffer::new().map_err(|e| {
//             error!("EthernetDevice::transmit(): couldn't allocate TransmitBuffer of length {}, error {:?}", len, e);
//             smoltcp::Error::Exhausted
//         })?;

//         let closure_retval = {
//             let txbuf_byte_slice = txbuf.as_slice_mut::<u8>(0, len).map_err(|e| {
//                 error!("EthernetDevice::transmit(): couldn't convert TransmitBuffer of length {} into byte slice, error {:?}", len, e);
//                 smoltcp::Error::Exhausted
//             })?;
//             f(txbuf_byte_slice)?
//         };
//         self.nic_ref.lock()
//             .send_packet(txbuf)
//             .map_err(|e| {
//                 error!("EthernetDevice::transmit(): error sending Ethernet packet: {:?}", e);
//                 smoltcp::Error::Exhausted
//             })?;
        
//         Ok(closure_retval)
//     }
// }


/// The receive token type used by smoltcp, 
/// which contains only a `ReceivedFrame` to be consumed later.
pub struct RxToken(BoxRefMut<ReceivedFrame, [u8]>);

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(mut self, _timestamp: Instant, f: F) -> smoltcp::Result<R>
        where F: FnOnce(&mut [u8]) -> smoltcp::Result<R>
    {
        f(self.0.as_mut())
    }
}


