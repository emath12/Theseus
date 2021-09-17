//! A mlx5 driver for a ConnectX-5 100GbE Network Interface Card.
//! 
//! Currently we only support reading the device PCI space, mapping the initialization segment,
//! and setting up a command queue to pass commands to the NIC.
//! 
//! All information is taken from the Mellanox Adapters Programmer’s Reference Manual (PRM) Rev 0.54,
//! unless otherwise specified. 

#![no_std]

#[macro_use] extern crate log;
extern crate alloc;
extern crate spin;
extern crate irq_safety;
extern crate memory;
extern crate pci; 
extern crate owning_ref;
extern crate nic_initialization;
extern crate mlx_ethernet;
extern crate kernel_config;

use spin::Once; 
use alloc::{
    vec::Vec,
    boxed::Box
};
use irq_safety::MutexIrqSafe;
use memory::{PhysicalAddress, MappedPages, create_contiguous_mapping};
use pci::PciDevice;
use owning_ref::BoxRefMut;
use nic_initialization::{NIC_MAPPING_FLAGS, allocate_memory};
use mlx_ethernet::{InitializationSegment, command_queue::{CommandBuilder, CommandOpcode, CommandQueue, CommandQueueEntry, ManagePagesOpMod, QueryHcaCapCurrentOpMod, QueryPagesOpMod}, completion_queue::CompletionQueue, event_queue::EventQueue, send_queue::SendQueue};
use kernel_config::memory::PAGE_SIZE;
use core::sync::atomic::fence;
use core::sync::atomic::Ordering;

/// Vendor ID for Mellanox
pub const MLX_VEND:           u16 = 0x15B3;
/// Device ID for the ConnectX-5 NIC
pub const CONNECTX5_DEV:      u16 = 0x1019; //7;

/// The singleton connectx-5 NIC.
/// TODO: Allow for multiple NICs
static CONNECTX5_NIC: Once<MutexIrqSafe<ConnectX5Nic>> = Once::new();

/// Returns a reference to the NIC wrapped in a MutexIrqSafe,
/// if it exists and has been initialized.
pub fn get_mlx5_nic() -> Option<&'static MutexIrqSafe<ConnectX5Nic>> {
    CONNECTX5_NIC.get()
}

/// Struct representing a ConnectX-5 network interface card.
#[allow(dead_code)]
pub struct ConnectX5Nic {
    /// Initialization segment base address
    mem_base: PhysicalAddress,
    /// Initialization Segment
    init_segment: BoxRefMut<MappedPages, InitializationSegment>,
    /// Command Queue
    command_queue: CommandQueue,
    /// Boot pages passed to the NIC. Once transferred, they should not be accessed by the driver.
    boot_pages: Vec<MappedPages>,
    /// Init pages passed to the NIC. Once transferred, they should not be accessed by the driver.
    init_pages: Vec<MappedPages>,
    event_queue: EventQueue,
    // completion_queue: CompletionQueue,
    // send_queue: SendQueue
}


/// Functions that setup the NIC struct.
impl ConnectX5Nic {

    /// Initializes the new ConnectX-5 network interface card that is connected as the given PciDevice.
    /// (steps taken from the PRM, Section 7.2: HCA Driver Start-up)
    pub fn init(mlx5_pci_dev: &PciDevice) -> Result<&'static MutexIrqSafe<ConnectX5Nic>, &'static str> {

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        mlx5_pci_dev.pci_set_command_bus_master_bit();

        // retrieve the memory-mapped base address of the initialization segment
        let mem_base = mlx5_pci_dev.determine_mem_base(0)?;
        trace!("mlx5 mem base = {}", mem_base);

        let mem_size = mlx5_pci_dev.determine_mem_size(0);
        trace!("mlx5 mem size = {}", mem_size);

        // map pages to the physical address given by mem_base as that is the intialization segment
        let mut init_segment = ConnectX5Nic::map_init_segment(mem_base)?;

        trace!("{:?}", init_segment);
        
        // find number of entries in command queue and stride
        let num_cmdq_entries = init_segment.num_cmdq_entries() as usize;
        trace!("mlx5 cmdq entries = {}", num_cmdq_entries);

        // find command queue entry stride, the number of bytes between the start of two adjacent entries.
        let cmdq_stride = init_segment.cmdq_entry_stride() as usize;
        trace!("mlx5 cmdq stride = {}", cmdq_stride);
        
        // We assume that the stride is equal to the size of the entry.
        if cmdq_stride != core::mem::size_of::<CommandQueueEntry>() {
            error!("Command Queue layout is no longer accurate due to invalid assumption.");
            return Err("Command Queue layout is no longer accurate due to invalid assumption.");
        }

        // create command queue
        let size_in_bytes_of_cmdq = num_cmdq_entries * cmdq_stride;
        trace!("total size in bytes of cmdq = {}", size_in_bytes_of_cmdq);
    
        // allocate mapped pages for the command queue
        let (cmdq_mapped_pages, cmdq_starting_phys_addr) = create_contiguous_mapping(size_in_bytes_of_cmdq, NIC_MAPPING_FLAGS)?;
        trace!("cmdq mem base = {}", cmdq_starting_phys_addr);
    
        // cast our physically-contiguous MappedPages into a slice of command queue entries
        let mut cmdq = CommandQueue::create(
            BoxRefMut::new(Box::new(cmdq_mapped_pages)).try_map_mut(|mp| mp.as_slice_mut::<CommandQueueEntry>(0, num_cmdq_entries))?,
            num_cmdq_entries
        )?;

        // write physical location of command queue to initialization segment
        init_segment.set_physical_address_of_cmdq(cmdq_starting_phys_addr)?;

        // Read initalizing field from initialization segment until it is cleared
        while init_segment.device_is_initializing() { trace!("device is initializing"); }
        trace!("initializing field is cleared.");

        // Execute ENABLE_HCA command
        let init_cmd = cmdq.create_command(CommandBuilder::new(CommandOpcode::EnableHca))?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        trace!("EnableHCA: {:?}", status);

        // execute QUERY_ISSI
        let init_cmd = cmdq.create_command( CommandBuilder::new(CommandOpcode::QueryIssi))?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let (current_issi, available_issi) = cmdq.get_query_issi_command_output(completed_cmd)?;
        trace!("QueryISSI: {:?}, issi version :{}, available: {:#X}", status, current_issi, available_issi);

        // execute SET_ISSI
        const ISSI_VERSION_1: u8 = 0x2;
        if available_issi & ISSI_VERSION_1 == ISSI_VERSION_1 {
            let init_cmd = cmdq.create_command( CommandBuilder::new(CommandOpcode::SetIssi))?;
            let posted_cmd = init_segment.post_command(init_cmd);
            let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
            trace!("SetISSI: {:?}", status);
        } else {
            return Err("ISSI indicated by PRM is not supported");
        }

        // Query pages for boot
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QueryPages).opmod(QueryPagesOpMod::BootPages as u16)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let num_boot_pages = cmdq.get_query_pages_command_output(completed_cmd)?;
        trace!("Query pages status: {:?}, Boot pages: {:?}", status, num_boot_pages);

        // Allocate pages for boot
        let mut boot_mp = Vec::with_capacity(num_boot_pages as usize);
        let mut boot_pa = Vec::with_capacity(num_boot_pages as usize);
        for _ in 0..num_boot_pages {
            let (page, pa) = create_contiguous_mapping(PAGE_SIZE, NIC_MAPPING_FLAGS)?;
            boot_mp.push(page);
            boot_pa.push(pa);
        }

        // execute MANAGE_PAGES command to transfer boot pages to device
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::ManagePages)
                .opmod(ManagePagesOpMod::AllocationSuccess as u16)
                .allocated_pages(boot_pa)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        trace!("Manage pages boot status: {:?}", status);

        // Query HCA capabilities
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QueryHcaCap)
                .opmod(QueryHcaCapCurrentOpMod::GeneralDeviceCapabilities as u16), 
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let port_type = cmdq.get_port_type(completed_cmd)?;
        trace!("Query HCA cap status:{:?}, port_type: {:?}", status, port_type);

        // Query pages for init
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QueryPages)
                .opmod(QueryPagesOpMod::InitPages as u16)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let num_init_pages = cmdq.get_query_pages_command_output(completed_cmd)?;
        trace!("Query pages status: {:?}, init pages: {:?}", status, num_init_pages);

        let mut init_mp = Vec::with_capacity(num_init_pages as usize);
        if num_init_pages != 0 {
            // Allocate pages for init
            let mut init_pa = Vec::with_capacity(num_init_pages as usize);
            for _ in 0..num_init_pages {
                let (page, pa) = create_contiguous_mapping(PAGE_SIZE, NIC_MAPPING_FLAGS)?;
                init_mp.push(page);
                init_pa.push(pa);
            }

            // execute MANAGE_PAGES command to transfer init pages to device
            let init_cmd = cmdq.create_command(
                CommandBuilder::new(CommandOpcode::ManagePages)
                    .opmod(ManagePagesOpMod::AllocationSuccess as u16)
                    .allocated_pages(init_pa)
            )?;
            let posted_cmd = init_segment.post_command(init_cmd);
            let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
            trace!("Manage pages init status: {:?}", status);
        }

        // execute INIT_HCA
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::InitHca) 
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        trace!("Init HCA status: {:?}", status);

        // Set driver version 
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::SetDriverVersion)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        trace!("Set Driver Version: {:?}", status);

        // execute ALLOC_UAR
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::AllocUar)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let uar = cmdq.get_uar(completed_cmd)?;
        trace!("UAR status: {:?}, UAR: {}", status, uar);        

        // execute CREATE_EQ for page request event
        // Allocate pages for EQ
        let num_eq_pages = 2;
        let mut eq_mp = Vec::with_capacity(num_eq_pages as usize);
        let mut eq_pa = Vec::with_capacity(num_eq_pages as usize);
        for _ in 0..num_eq_pages {
            let (page, pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
            eq_mp.push(page);
            eq_pa.push(pa);
        }
        // set the ownership bit of the EQE to HW owned
        let mut event_queue = EventQueue::create(eq_mp)?;
        event_queue.init();

        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::CreateEq)
                .allocated_pages(eq_pa)
                .uar(uar)
                .log_queue_size(7)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let eq_number = cmdq.get_eq_number(completed_cmd)?;
        trace!("Create EQ status: {:?}, number: {}", status, eq_number);

        // execute QUERY_VPORT_STATE
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QueryVportState)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let (tx_speed, admin_state, state) = cmdq.get_vport_state(completed_cmd)?;
        trace!("Query Vport State status: {:?}, tx_speed: {:#X}, admin_state:{:#X}, state: {:#X}", status, tx_speed, admin_state, state);  
        
        // execute QUERY_NIC_VPORT_CONTEXT
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QueryNicVportContext)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let mac = cmdq.get_vport_mac_address(completed_cmd)?;
        trace!("Query Nic Vport context status: {:?}, mac address: {:#X?}", status, mac);

        // execute ALLOC_PD
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::AllocPd)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let pd = cmdq.get_protection_domain(completed_cmd)?;
        trace!("Alloc PD status: {:?}, protection domain num: {}", status, pd);

        // execute ALLOC_TRANSPORT_DOMAIN
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::AllocTransportDomain)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let td = cmdq.get_transport_domain(completed_cmd)?;
        trace!("Alloc TD status: {:?}, transport domain num: {}", status, td);

        // execute QUERY_SPECIAL_CONTEXTS
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QuerySpecialContexts)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let rlkey = cmdq.get_reserved_lkey(completed_cmd)?;
        trace!("Query Special Contexts status: {:?}, rlkey: {}", status, rlkey);
        
        // execute CREATE_CQ 
        // Allocate pages for CQ
        let num_cq_pages = 1;
        let mut cq_mp = Vec::with_capacity(num_cq_pages as usize);
        let mut cq_pa = Vec::with_capacity(num_cq_pages as usize);
        for _ in 0..num_cq_pages {
            let (page, pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
            cq_mp.push(page);
            cq_pa.push(pa);
        }
        // Allocate page for doorbell
        let (db_page, db_pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
        
        // Initialize the CQ
        let mut completion_queue = CompletionQueue::create(cq_mp, db_page)?;
        completion_queue.init();

        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::CreateCq) 
                .allocated_pages(cq_pa)
                .uar(uar)
                .log_queue_size(0)
                .eqn(eq_number)
                .db_page(db_pa)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let cq_number = cmdq.get_cq_number(completed_cmd)?;
        trace!("Create CQ status: {:?}, number: {}", status, cq_number);

        // execute CREATE_TIS
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::CreateTis)
                .td(td),
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let tisn = cmdq.get_tis_context_number(completed_cmd)?;
        trace!("Create TIS status: {:?}, tisn: {}", status, tisn);

        // execute CREATE_SQ for page request event
        // Allocate pages for SQ
        let num_sq_pages = 2;
        let mut sq_mp = Vec::with_capacity(num_sq_pages as usize);
        let mut sq_pa = Vec::with_capacity(num_sq_pages as usize);
        for _ in 0..num_sq_pages {
            let (page, pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
            sq_mp.push(page);
            sq_pa.push(pa);
        }
        // Allocate page for doorbell
        let (db_page, db_pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
        // Allocate page for UAR
        let uar_mem_base = mem_base.value() + ((uar as usize - 1) * 4096);
        let uar_page = allocate_memory(PhysicalAddress::new(uar_mem_base).ok_or("Could not create starting address for uar")?, 4096)?;
        
        // Create the SQ
        let mut send_queue = SendQueue::create(sq_mp, db_page, uar_page)?;

        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::CreateSq) 
                .allocated_pages(sq_pa) 
                .uar(uar) 
                .log_queue_size(7)
                .db_page(db_pa)
                .cqn(cq_number) 
                .tisn(tisn) 
                .pd(pd)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let sq_number = cmdq.get_send_queue_number(completed_cmd)?;
        trace!("Create SQ status: {:?}, number: {}", status, sq_number);

        // MODIFY_SQ
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::ModifySq) 
                .cqn(cq_number) 
                .tisn(tisn) 
                .sqn(sq_number)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        trace!("Modify SQ status: {:?}", status);

        // QUERY_SQ
        let init_cmd = cmdq.create_command(
            CommandBuilder::new(CommandOpcode::QuerySq)
                .sqn(sq_number)
        )?;
        let posted_cmd = init_segment.post_command(init_cmd);
        let (completed_cmd, status) = cmdq.wait_for_command_completion(posted_cmd)?;
        let state = cmdq.get_sq_state(completed_cmd)?;
        trace!("Query SQ status: {:?}, state: {}", status, state);

        let (mut packet, pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS)?;
        let buffer: &mut [u8] = packet.as_slice_mut(0, 298)?;
        
        let dhcp_packet: [u8; 298] = [ 
            0x01, 0x2c, 0xa8, 0x36, 0x00, 0x00, 0xfa, 0x11, 0x17, 0x8b, 0x00, 0x00, 0x00, 0x00,
        0xff, 0xff, 0xff, 0xff, 0x00, 0x44, 0x00, 0x43, 0x01, 0x18, 0x59, 0x1f, 0x01, 0x01, 0x06,
        0x00, 0x00, 0x00, 0x3d, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc6, 0x9c, 0x89,
        0x4c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x82, 0x53, 0x63, 0x35, 0x01, 0x01,
        0x3d, 0x07, 0x01, 0x00, 0x1f, 0xc6, 0x9c, 0x89, 0x4c, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00,
        0x37, 0x04, 0x01, 0x03, 0x06, 0x2a, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];
        
        buffer.copy_from_slice(&dhcp_packet);

        // send_queue.send(sq_number, tisn, rlkey, pa)?;
        send_queue.nop(sq_number, tisn, rlkey)?;
        while completion_queue.hw_owned() {}
        completion_queue.check_packet_transmission();

        let mlx5_nic = ConnectX5Nic {
            mem_base: mem_base,
            init_segment: init_segment,
            command_queue: cmdq, 
            boot_pages: boot_mp,
            init_pages: init_mp,
            event_queue: event_queue,
            // completion_queue: completion_queue,
            // send_queue: send_queue
        };
        
        let nic_ref = CONNECTX5_NIC.call_once(|| MutexIrqSafe::new(mlx5_nic));
        Ok(nic_ref)
    }
    
    /// Returns the memory-mapped initialization segment of the NIC
    fn map_init_segment(mem_base: PhysicalAddress) -> Result<BoxRefMut<MappedPages, InitializationSegment>, &'static str> {
        let mp = allocate_memory(mem_base, core::mem::size_of::<InitializationSegment>())?;
        BoxRefMut::new(Box::new(mp)).try_map_mut(|mp| mp.as_type_mut::<InitializationSegment>(0))
    }
}
