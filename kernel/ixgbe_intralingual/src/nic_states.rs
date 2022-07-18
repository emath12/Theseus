use pci::{PciDevice, MSIX_CAPABILITY, PciConfigSpaceAccessMechanism, PciLocation};
use memory::{PhysicalAddress, MappedPages};
use owning_ref::BoxRefMut;
use alloc::{
    vec::Vec,
    sync::Arc,
    boxed::Box,
};
use nic_initialization::{allocate_memory};
use core::mem::ManuallyDrop;

use crate::NumDesc;
use crate::regs::*;
use crate::queue_registers::*;
use crate::queues::{TxQueue, IxgbeTxDescriptors};
use crate::mapped_pages_fragments::MappedPagesFragments;

struct IxgbeNic<S: InitializationState> {
    /// Device ID of the NIC assigned by the device manager.
    dev_id: PciLocation,
    /// Type of Base Address Register 0,
    /// if it's memory mapped or I/O.
    bar_type: u8,
    /// MMIO Base Address     
    mem_base: PhysicalAddress,
    /// remaining info that depends on the state
    state: S
}

impl IxgbeNic<Start> {
    pub fn new(ixgbe_pci_dev: PciDevice) -> Result<IxgbeNic<Start>, &'static str> {
        let bar0 = ixgbe_pci_dev.bars[0];
        // Determine the type from the base address register
        let bar_type = (bar0 as u8) & 0x01;    

        // If the base address is not memory mapped then exit
        if bar_type == PciConfigSpaceAccessMechanism::IoPort as u8 {
            error!("ixgbe::init(): BAR0 is of I/O type");
            return Err("ixgbe::init(): BAR0 is of I/O type")
        }

        // 16-byte aligned memory mapped base address
        let mem_base =  ixgbe_pci_dev.determine_mem_base(0)?;

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();

        Ok(IxgbeNic { dev_id: ixgbe_pci_dev.location, bar_type, mem_base, state: Start{} })
    }

    pub fn map_regs(self) -> Result<IxgbeNic<MappedRegisters>, &'static str> {
        // map the IntelIxgbeRegisters structs to the address found from the pci space
        let (regs1, regs2, regs3, regs_mac, 
            rx_regs_disabled, tx_regs_disabled) = self.mapped_reg()?;

        Ok(IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: MappedRegisters{
                regs1,
                regs2,
                regs3,
                regs_mac,
                rx_regs_disabled,
                tx_regs_disabled
            } 
        })
    }

    /// Returns the memory-mapped control registers of the nic and the rx/tx queue registers.
    fn mapped_reg(&self) -> Result<(
        BoxRefMut<MappedPages, IntelIxgbeRegisters1>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters2>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters3>, 
        BoxRefMut<MappedPages, IntelIxgbeMacRegisters>, 
        Vec<IxgbeRxQueueRegisters>, 
        Vec<IxgbeTxQueueRegisters>
    ), &'static str> {
        // We've divided the memory-mapped registers into multiple regions.
        // The size of each region is found from the data sheet, but it always lies on a page boundary.
        const GENERAL_REGISTERS_1_SIZE_BYTES:   usize = 4096;
        const RX_REGISTERS_SIZE_BYTES:          usize = 4096;
        const GENERAL_REGISTERS_2_SIZE_BYTES:   usize = 4 * 4096;
        const TX_REGISTERS_SIZE_BYTES:          usize = 2 * 4096;
        const MAC_REGISTERS_SIZE_BYTES:         usize = 5 * 4096;
        const GENERAL_REGISTERS_3_SIZE_BYTES:   usize = 18 * 4096;

        // Allocate memory for the registers, making sure each successive memory region begins where the previous region ended.
        let mut offset = self.mem_base;
        let nic_regs1_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_1_SIZE_BYTES)?;

        offset += GENERAL_REGISTERS_1_SIZE_BYTES;
        let nic_rx_regs1_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES)?;

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs2_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_2_SIZE_BYTES)?;  

        offset += GENERAL_REGISTERS_2_SIZE_BYTES;
        let nic_tx_regs_mapped_page = allocate_memory(offset, TX_REGISTERS_SIZE_BYTES)?;

        offset += TX_REGISTERS_SIZE_BYTES;
        let nic_mac_regs_mapped_page = allocate_memory(offset, MAC_REGISTERS_SIZE_BYTES)?;

        offset += MAC_REGISTERS_SIZE_BYTES;
        let nic_rx_regs2_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES)?;   

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs3_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_3_SIZE_BYTES)?;

        // Map the memory as the register struct and tie the lifetime of the struct with its backing mapped pages
        let regs1 = BoxRefMut::new(Box::new(nic_regs1_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters1>(0))?;
        let regs2 = BoxRefMut::new(Box::new(nic_regs2_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters2>(0))?;
        let regs3 = BoxRefMut::new(Box::new(nic_regs3_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters3>(0))?;
        let mac_regs = BoxRefMut::new(Box::new(nic_mac_regs_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeMacRegisters>(0))?;
        
        // Divide the pages of the Rx queue registers into multiple 64B regions
        let mut regs_rx = Self::mapped_regs_from_rx_memory(nic_rx_regs1_mapped_page);
        regs_rx.append(&mut Self::mapped_regs_from_rx_memory(nic_rx_regs2_mapped_page));
        
        // Divide the pages of the Tx queue registers into multiple 64B regions
        let regs_tx = Self::mapped_regs_from_tx_memory(MappedPagesFragments::new(nic_tx_regs_mapped_page))?;
            
        Ok((regs1, regs2, regs3, mac_regs, regs_rx, regs_tx))
    }

    /// Split the pages where rx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_rx_memory(mp: MappedPages) -> Vec<IxgbeRxQueueRegisters> {
        const QUEUES_IN_MP: usize = 64;
        const RX_QUEUE_REGISTERS_SIZE_BYTES: usize = core::mem::size_of::<RegistersRx>();
        
        assert!(mp.size_in_bytes() >= QUEUES_IN_MP * RX_QUEUE_REGISTERS_SIZE_BYTES);

        let starting_address = mp.start_address();

        // We share the backing mapped pages among all the queue registers
        let shared_mp = Arc::new(mp);
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            // This is safe because we have checked that the number of queues we want to partition from these mapped pages fit into the allocated memory,
            // and that each queue starts at the end of the previous.
            // We also ensure that the backing mapped pages are included in the same struct as the registers, almost as a pseudo OwningRef
            let registers = unsafe{ Box::from_raw((starting_address.value() + (i * RX_QUEUE_REGISTERS_SIZE_BYTES)) as *mut RegistersRx) };
            pointers_to_queues.push(
                IxgbeRxQueueRegisters {
                    regs: ManuallyDrop::new(registers),
                    backing_pages: shared_mp.clone()
                }
            );
        }
        pointers_to_queues
    }

    /// Split the pages where tx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_tx_memory(mut mp: MappedPagesFragments) -> Result<Vec<IxgbeTxQueueRegisters>, &'static str> {
        const QUEUES_IN_MP: usize = 128;

        // We share the backing mapped pages among all the queue registers
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            pointers_to_queues.push(
                IxgbeTxQueueRegisters::new(i, &mut mp)?
            );
        }
        Ok(pointers_to_queues)
    }
}

impl IxgbeNic<MappedRegisters> {
    pub fn disable_interrupts(mut self) -> IxgbeNic<InterruptDisabled>{
        self.state.regs1.eimc_disable_interrupts();
        
        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: InterruptDisabled { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled 
            } 
        }
    }
}

impl IxgbeNic<InterruptDisabled> {
    fn global_reset(mut self) -> IxgbeNic<GlobalReset> {
   // master disable algorithm (sec 5.2.5.3.2)
        // global reset = sw reset + link reset 
        self.state.regs1.ctrl_reset();

        //wait 10 ms
        let wait_time = 10_000;
        let _ = pit_clock::pit_wait(wait_time);

        //disable flow control.. write 0 TO FCTTV, FCRTL, FCRTH, FCRTV and FCCFG
        for fcttv in self.state.regs2.fcttv.iter_mut() {
            fcttv.write(0);
        }

        self.state.regs2.fcrtl_clear();
        self.state.regs2.fcrth_clear();
        self.state.regs2.fcrtv_clear();
        self.state.regs2.fccfg_clear();

        //disable interrupts
        self.state.regs1.eimc_disable_interrupts();

        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: GlobalReset { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled 
            } 
        }
    }
}

impl IxgbeNic<GlobalReset> {
    fn wait_for_auto_read_completion(mut self) -> IxgbeNic<EEPROMAutoRead> {
        //wait for eeprom auto read completion
        while !self.state.regs3.eec_auto_read(){}

        //read MAC address
        debug!("Ixgbe: MAC address low: {:#X}", self.state.regs_mac.ral.read());
        debug!("Ixgbe: MAC address high: {:#X}", self.state.regs_mac.rah.read() & 0xFFFF);

        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: EEPROMAutoRead { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled 
            } 
        }
    }
}

impl IxgbeNic<EEPROMAutoRead> {
    fn wait_for_dma_init(self) -> IxgbeNic<DMAInitDone> {
        //wait for dma initialization done (RDRXCTL.DMAIDONE)
        // TODO: can move to a function
        let mut val = self.state.regs2.rdrxctl_read();
        let dmaidone_bit = 1 << 3;
        while val & dmaidone_bit != dmaidone_bit {
            val = self.state.regs2.rdrxctl_read();
        }

        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: DMAInitDone { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled 
            } 
        }
    }
}

impl IxgbeNic<DMAInitDone> {
    fn set_up_phy_and_link(self) -> IxgbeNic<SetUpLink> {
        //TODO: seems like we do nothing, remove state?
        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: SetUpLink { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled 
            } 
        }
    }
}

impl IxgbeNic<SetUpLink> {
    fn clear_stats(self) -> IxgbeNic<ClearStats> {
        self.read_stat_regs();
        let mac_hardware = self.read_mac_address_from_nic();

        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: ClearStats { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled,
                mac_hardware,
                mac_spoofed: None
            } 
        }
    }

    /// Reads the actual MAC address burned into the NIC hardware.
    fn read_mac_address_from_nic(&self) -> [u8; 6] {
        let mac_32_low = self.state.regs_mac.ral.read();
        let mac_32_high = self.state.regs_mac.rah.read();

        let mut mac_addr = [0; 6]; 
        mac_addr[0] =  mac_32_low as u8;
        mac_addr[1] = (mac_32_low >> 8) as u8;
        mac_addr[2] = (mac_32_low >> 16) as u8;
        mac_addr[3] = (mac_32_low >> 24) as u8;
        mac_addr[4] =  mac_32_high as u8;
        mac_addr[5] = (mac_32_high >> 8) as u8;

        debug!("Ixgbe: read hardware MAC address: {:02x?}", mac_addr);
        mac_addr
    }

    /// Clear the statistic registers by reading from them.
    fn read_stat_regs(&self) {
        self.state.regs2.gprc.read();
        self.state.regs2.gptc.read();
        self.state.regs2.gorcl.read();
        self.state.regs2.gorch.read();
        self.state.regs2.gotcl.read();
        self.state.regs2.gotch.read();
    }
}

impl IxgbeNic<ClearStats> {
    fn init_rx(self) -> IxgbeNic<InitRx> {
        IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: InitRx { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: self.state.tx_regs_disabled,
                mac_hardware: self.state.mac_hardware,
                mac_spoofed: None,
                num_rx_queues: 0
            } 
        }
    }
}

impl IxgbeNic<InitRx> {
    fn init_tx(mut self, num_tx_descs: NumDesc) -> Result<IxgbeNic<InitTx>, &'static str> {
        let tx_regs_disabled = self.state.tx_regs_disabled.split_off(crate::IXGBE_NUM_TX_QUEUES_ENABLED as usize);
        let tx_queues = self.tx_init(num_tx_descs)?;

        Ok(IxgbeNic { 
            dev_id: self.dev_id, 
            bar_type: self.bar_type, 
            mem_base: self.mem_base, 
            state: InitTx { 
                regs1: self.state.regs1, 
                regs2: self.state.regs2, 
                regs3: self.state.regs3, 
                regs_mac: self.state.regs_mac, 
                rx_regs_disabled: self.state.rx_regs_disabled, 
                tx_regs_disabled: tx_regs_disabled,
                mac_hardware: self.state.mac_hardware,
                mac_spoofed: None,
                num_rx_queues: 0,
                num_tx_queues: tx_queues.len() as u8,
                tx_queues,
            } 
        })
    }
    /// Initialize the array of transmit descriptors for all queues and returns them.
    /// Also enables transmit functionality for the NIC.
    fn tx_init(&mut self, num_tx_descs: NumDesc) -> Result<Vec<TxQueue>, &'static str> {
        // disable transmission
        self.state.regs2.dmatxctl_disable_tx();

        // CRC offload and small packet padding enable
        self.state.regs2.hlreg0_crc_en();
        self.state.regs2.hlreg0_tx_pad_en();

        // Set RTTFCS.ARBDIS to 1
        self.state.regs2.rttdcs_set_arbdis();

        // program DTXMXSZRQ and TXPBSIZE according to DCB and virtualization modes (both off)
        self.state.regs_mac.txpbsize_write(0, TXPBSIZE_160KB)?;
        for i in 1..8 {
            self.state.regs_mac.txpbsize_write(i, 0)?;
        }
        self.state.regs_mac.dtxmxszrq_write(DTXMXSZRQ_MAX_BYTES)?; 

        // Clear RTTFCS.ARBDIS
        self.state.regs2.rttdcs_clear_arbdis();

        let mut tx_all_queues = Vec::new();

        // enable transmit operation, only have to do this for the first queue
        self.state.regs2.dmatxctl_enable_tx();

        let mut tx_regs_disabled = Vec::new();
        core::mem::swap(&mut tx_regs_disabled, &mut self.state.tx_regs_disabled);

        for tx_reg in tx_regs_disabled {
            let mut txq = Self::init_tx_queue(num_tx_descs, tx_reg)?;
        
            // Set descriptor thresholds
            // If we enable this then we need to change the packet send function to stop polling
            // for a descriptor done on every packet sent
            // txq.txdctl.write(TXDCTL_PTHRESH | TXDCTL_HTHRESH | TXDCTL_WTHRESH); 

            //enable tx queue
            txq.regs.txdctl_txq_enable(); 

            //make sure queue is enabled
            while txq.regs.txdctl_read() & TX_Q_ENABLE == 0 {} 

            tx_all_queues.push(txq);
        }
        Ok(tx_all_queues)
    }  

    /// Steps to create and initialize a transmit descriptor queue
    /// 
    /// # Arguments
    /// * `num_desc`: number of descriptors in the queue
    /// * `txq_regs`: registers needed to set up a transmit queue
    fn init_tx_queue(num_desc: NumDesc, txq_regs: IxgbeTxQueueRegisters) 
        -> Result<TxQueue, &'static str> 
    {
        let tx_descs = IxgbeTxDescriptors::new(num_desc)?;
        Ok(TxQueue::new(txq_regs, tx_descs, None))
    }
}

struct Start {}

struct MappedRegisters {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct InterruptDisabled {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct GlobalReset {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct EEPROMAutoRead {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct DMAInitDone {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct SetUpLink {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
}

struct ClearStats {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    /// The optional spoofed MAC address to use in place of `mac_hardware` when transmitting.  
    mac_spoofed: Option<[u8; 6]>,
}

struct InitRx {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    /// The optional spoofed MAC address to use in place of `mac_hardware` when transmitting.  
    mac_spoofed: Option<[u8; 6]>,
    /// The number of rx queues enabled
    num_rx_queues: u8,
}

struct InitTx {
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Registers for the disabled queues
    rx_regs_disabled: Vec<IxgbeRxQueueRegisters>,
    /// Registers for the disabled queues
    tx_regs_disabled: Vec<IxgbeTxQueueRegisters>,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    /// The optional spoofed MAC address to use in place of `mac_hardware` when transmitting.  
    mac_spoofed: Option<[u8; 6]>,
    /// The number of rx queues enabled
    num_rx_queues: u8,
    /// The number of tx queues enabled
    num_tx_queues: u8,
    /// Vector of the enabled tx queues
    tx_queues: Vec<TxQueue>,
}

trait InitializationState {}
impl InitializationState for Start {}
impl InitializationState for MappedRegisters {}
impl InitializationState for InterruptDisabled {}
impl InitializationState for GlobalReset {}
impl InitializationState for EEPROMAutoRead {}
impl InitializationState for DMAInitDone {}
impl InitializationState for SetUpLink {}
impl InitializationState for ClearStats {}
impl InitializationState for InitRx {}
impl InitializationState for InitTx {}
