use memory::{MappedPages, create_contiguous_mapping, PhysicalAddress};
use crate::descriptors::AdvancedTxDescriptor;
use crate::queue_registers::IxgbeTxQueueRegisters;
use crate::NumDesc;
use nic_buffers::TransmitBuffer;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}};
use nic_initialization::NIC_MAPPING_FLAGS;
use alloc::boxed::Box;

/// A struct that holds all information for a transmit queue. 
/// There should be one such object per queue.
pub struct TxQueue {
    /// The number of the queue, stored here for our convenience.
    id: u8,
    /// Registers for this transmit queue
    pub(crate) regs: IxgbeTxQueueRegisters,
    /// Transmit descriptors 
    pub(crate) tx_descs: IxgbeTxDescriptors,
    /// The number of transmit descriptors in the descriptor ring
    num_tx_descs: u16,
    /// Current transmit descriptor index
    tx_cur: u16,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything but we use this value when setting the cpu id for interrupts and DCA.
    cpu_id : Option<u8>
}

impl TxQueue {
    pub(crate) fn new(mut regs: IxgbeTxQueueRegisters, tx_descs: IxgbeTxDescriptors, cpu_id: Option<u8>) -> TxQueue {
        let num_tx_descs = tx_descs.len();
        // write the physical address of the tx descs array
        regs.tdbal.write(tx_descs.paddr.value() as u32); 
        regs.tdbah.write((tx_descs.paddr.value() >> 32) as u32); 

        // write the length (in total bytes) of the tx descs array
        regs.tdlen.write((num_tx_descs * core::mem::size_of::<AdvancedTxDescriptor>()) as u32);               
        
        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        regs.tdh.write(0);
        regs.tdt.write(0);

        TxQueue { id: regs.id() as u8, regs, tx_descs, num_tx_descs: num_tx_descs as u16, tx_cur: 0, cpu_id }
    }

    /// Sends a packet on the transmit queue
    /// 
    /// # Arguments:
    /// * `transmit_buffer`: buffer containing the packet to be sent
    pub fn send_on_queue(&mut self, transmit_buffer: TransmitBuffer) {
        self.tx_descs[self.tx_cur as usize].send(transmit_buffer.phys_addr, transmit_buffer.length);  
        // update the tx_cur value to hold the next free descriptor
        let old_cur = self.tx_cur;
        self.tx_cur = (self.tx_cur + 1) % self.num_tx_descs;
        // update the tdt register by 1 so that it knows the previous descriptor has been used
        // and has a packet to be sent
        self.regs.tdt.write(self.tx_cur as u32);
        // Wait for the packet to be sent
        self.tx_descs[old_cur as usize].wait_for_packet_tx();
    }
}

pub struct IxgbeTxDescriptors {
    desc_ring: BoxRefMut<MappedPages, [AdvancedTxDescriptor]>,
    paddr: PhysicalAddress
}

impl IxgbeTxDescriptors {
    pub fn new(num_desc: NumDesc) -> Result<IxgbeTxDescriptors, &'static str> {
        let size_in_bytes_of_all_tx_descs = num_desc as usize * core::mem::size_of::<AdvancedTxDescriptor>();
        
        // Tx descriptors must be 128 byte-aligned, which is satisfied below because it's aligned to a page boundary.
        let (tx_descs_mapped_pages, tx_descs_starting_phys_addr) = create_contiguous_mapping(size_in_bytes_of_all_tx_descs, NIC_MAPPING_FLAGS)?;
        let offset_in_mp = 0;

        if (tx_descs_starting_phys_addr.value() + offset_in_mp) % 128 != 0 {
            return Err("Descriptors are not 128-byte aligned");
        }

        let mut desc_ring = BoxRefMut::new(Box::new(tx_descs_mapped_pages))
            .try_map_mut(|mp| mp.as_slice_mut::<AdvancedTxDescriptor>(offset_in_mp, num_desc as usize))?;

        
        for desc in desc_ring.iter_mut() {
            desc.init()
        }

        Ok(IxgbeTxDescriptors{ desc_ring, paddr: tx_descs_starting_phys_addr + offset_in_mp })
    }
}

impl Deref for IxgbeTxDescriptors {
    type Target = BoxRefMut<MappedPages, [AdvancedTxDescriptor]>;
    fn deref(&self) -> &BoxRefMut<MappedPages, [AdvancedTxDescriptor]> {
        &self.desc_ring
    }
}

impl DerefMut for IxgbeTxDescriptors {
    fn deref_mut(&mut self) -> &mut BoxRefMut<MappedPages, [AdvancedTxDescriptor]> {
        &mut self.desc_ring
    }
}

