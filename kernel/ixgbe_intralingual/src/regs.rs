//! This file contains the structs that are used to access device registers and contains configuration values to write to registers.
//! 
//! The registers are divided into multiple structs because we need to separate out the 
//! receive and transmit queue registers and store them separately for virtualization. 
//! 
//! The 7 structs which cover the registers of the entire memory-mapped region are:
//! * `IntelIxgbeRegisters1`
//! * `IntelIxgbeRxRegisters1`
//! * `IntelIxgbeRegisters2`
//! * `IntelIxgbeTxRegisters`
//! * `IntelIxgbeMacRegisters`
//! * `IntelIxgbeRxRegisters2`
//! * `IntelIxgbeRegisters3`
//! 
//! Some of the type classifiers may be more restrictive than specified in the datasheet.
//! For example, setting RW fields to RO or keeping registers private.
//! This simply indicates that the extra functions are currently not used in the driver, 
//! and so we haven't implemented the necessary checks for safe access.

use volatile::{Volatile, ReadOnly, WriteOnly};
use zerocopy::FromBytes;
use bit_field::BitField;

/// The layout in memory of the first set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters1 {
    /// Device Control Register
    ctrl:                               Volatile<u32>,          // 0x0
    _padding0:                          [u8; 4],                // 0x4 - 0x7
    
    /// Device Status Register
    status:                             ReadOnly<u32>,          // 0x8
    _padding1:                          [u8; 12],               // 0xC - 0x17

    /// Extended Device Control Register
    ctrl_ext:                           Volatile<u32>,          // 0x18
    _padding2:                          [u8; 12],               // 0x1C - 0x27

    /// I2C Control
    i2cctl:                             Volatile<u32>,          // 0x28
    _padding3:                          [u8; 2004],             // 0x2C - 0x7FF

    /// Extended Interrupt Cause Register
    pub eicr:                           Volatile<u32>,          // 0x800
    _padding4:                          [u8; 4],                // 0x804 - 0x807

    /// Extended Interrupt Cause Set Register
    eics:                               WriteOnly<u32>,         // 0x808
    _padding5:                          [u8; 4],                // 0x80C - 0x80F

    /// Extended Interrupt Auto Clear Register
    eiac:                               Volatile<u32>,          // 0x810; 
    _padding6:                          [u8; 12],               // 0x814 - 0x81F 
    
    /// Extended Interrupt Throttle Registers
    eitr:                               [Volatile<u32>; 24],    // 0x820 - 0x87F; 

    /// Extended Interrupt Mask Set/ Read Register
    eims:                               Volatile<u32>,          // 0x880; 
    _padding7:                          [u8; 4],                // 0x884 - 0x887

    /// Extended Interrupt Mask Clear Register
    eimc:                               WriteOnly<u32>,         // 0x888; 
    _padding8:                          [u8; 4],                // 0x88C - 0x88F     

    /// Extended Interrupt Auto Mask Enable Register
    eiam:                               Volatile<u32>,          // 0x890; 
    _padding9:                          [u8; 4],                // 0x894 - 0x897

    /// General Purpose Interrupt Enable
    gpie:                               Volatile<u32>,          // 0x898; 
    _padding10:                         [u8; 100],              // 0x89C - 0x8FF

    /// Interrupt Vector Allocation Registers
    ivar:                               [Volatile<u32>; 64],    // 0x900 - 0x9FF  
    _padding11:                         [u8; 1536],             // 0xA00 - 0xFFF

} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters1>(), 4096);


impl IntelIxgbeRegisters1 {
    pub fn ctrl_reset(&mut self) {
        let val = self.ctrl.read();
        self.ctrl.write(val | CTRL_RST | CTRL_LRST);
    }

    pub fn eimc_disable_interrupts(&mut self) {
        self.eimc.write(DISABLE_INTERRUPTS);
    }

    pub fn ctrl_ext_no_snoop_disable(&mut self) {
        self.ctrl_ext.write(self.ctrl_ext.read() | CTRL_EXT_NO_SNOOP_DIS);
    }

    pub fn ivar_read(&mut self, reg_idx: usize) -> u32 {
        self.ivar[reg_idx].read()
    }

    pub fn ivar_write(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        let write_mask = 0xBFBF_BFBF;
        const NUM_IVAR_REGS: usize = 64;

        if (val & !write_mask != 0) || (reg_idx >= NUM_IVAR_REGS) {
            Err("failed to write to ivar reg")
        } else {
            self.ivar[reg_idx].write(val);
            Ok(())
        }
    }

    pub fn gpie_enable_immediate_int_and_multiple_msix(&mut self) {
        let val = self.gpie.read();
        self.gpie.write(val | GPIE_EIMEN | GPIE_MULTIPLE_MSIX | GPIE_PBA_SUPPORT); 
    }

    pub fn eims_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x7FFF_FFFF;
        if val & !write_mask != 0 {
            Err("invalid write to EIMS")
        } else {
            self.eims.write(val);
            Ok(())
        }
    }

    pub fn eiac_enable_rtxq_autoclear(&mut self) {
        self.eiac.write(EIAC_RTXQ_AUTO_CLEAR);
    }

    pub fn eitr_set_interval(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        const NUM_EITR_REGS: usize = 24;
        let itr_interval_mask = 0x1FF; // only uses 9 bits

        if (val & !itr_interval_mask != 0) || (reg_idx >= NUM_EITR_REGS) {
            Err("failed to write to eitr reg")
        } else {
            self.eitr[reg_idx].write(val << EITR_ITR_INTERVAL_SHIFT);
            Ok(())
        }
    }
}

/// The layout in memory of the first set of receive queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRxRegisters1 {
    /// First set of Rx Registers for 64 Rx Queues
    pub rx_regs1:                       [RegistersRx; 64],      // 0x1000 - 0x1FFF

} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRxRegisters1>(), 4096);

/// The layout in memory of the second set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters2 {
    _padding1:                          [u8; 3840],             // 0x2000 - 0x2EFF
    
    /// Receive DMA Control Register
    rdrxctl:                            Volatile<u32>,          // 0x2F00;
    _padding2:                          [u8; 252],              // 0x2F04 - 0x2FFF

    /// Receive Control Register
    rxctrl:                             Volatile<u32>,          // 0x3000;
    _padding3:                          [u8; 508],              // 0x3004 - 0x31FF

    /// Flow Control Transmit Timer Value
    pub fcttv:                          [Volatile<u32>;4],      // 0x3200 - 0x320F
    _padding4:                          [u8; 16],               // 0x3210 - 0x321F

    /// Flow Control Receive Threshold Low
    fcrtl:                              [Volatile<u32>;8],      // 0x3220 - 0x323F
    _padding5:                          [u8; 32],               // 0x3240 - 0x325F 

    /// Flow Control Receive Threshold High
    fcrth:                              [Volatile<u32>;8],      // 0x3260 - 0x327F
    _padding6:                          [u8; 32],               // 0x3280 - 0x329F

    /// Flow Control Refresh Threshold Value
    fcrtv:                              Volatile<u32>,          // 0x32A0;
    _padding7:                          [u8; 2396],             // 0x32A4 - 0x3CFF

    ///Receive Packet Buffer Size
    rxpbsize:                           [Volatile<u32>;8],      // 0x3C00   
    _padding8:                          [u8; 224],              // 0x3C20 - 0x3CFF        

    /// Flow Control Configuration
    fccfg:                              Volatile<u32>,          // 0x3D00;
    _padding9:                          [u8; 880],              // 0x3D04 - 0x4073

    /// Good Packets Received Count
    pub gprc:                           Volatile<u32>,          // 0x4074
    _padding10:                         [u8; 8],                // 0x4078 - 0x407F

    /// Good Packets Transmitted Count
    pub gptc:                           Volatile<u32>,          // 0x4080
    _padding11:                         [u8; 4],                // 0x4084 - 0x4087 

    /// Good Octets Received Count Low
    pub gorcl:                          Volatile<u32>,          // 0x4088

    /// Good Octets Received Count High
    pub gorch:                          Volatile<u32>,          // 0x408C
    
    /// Good Octets Transmitted Count Low
    pub gotcl:                          Volatile<u32>,          // 0x4090

    /// Good Octets Transmitted Count High
    pub gotch:                          Volatile<u32>,          // 0x4094
    _padding12:                         [u8; 424],              // 0x4098 - 0x423F

    /// MAC Core Control 0 Register 
    hlreg0:                             Volatile<u32>,          // 0x4240;
    _padding13:                         [u8; 92],               // 0x4244 - 0x429F

    /// Auto-Negotiation Control Register
    pub autoc:                          Volatile<u32>,          // 0x42A0;

    /// Link Status Register
    pub links:                          Volatile<u32>,          // 0x42A4;

    /// Auto-Negotiation Control 2 Register    
    autoc2:                             Volatile<u32>,          // 0x42A8;
    _padding14:                         [u8; 120],              // 0x42AC - 0x4323

    /// Link Status Register 2
    pub links2:                         Volatile<u32>,          // 0x4324
    _padding15:                         [u8; 1496],             // 0x4328 - 0x48FF

    /// DCB Transmit Descriptor Plane Control and Status
    rttdcs:                             Volatile<u32>,          // 0x4900;
    _padding16:                         [u8; 380],              // 0x4904 - 0x4A7F

    /// DMA Tx Control
    dmatxctl:                           Volatile<u32>,          // 0x4A80;
    _padding17:                         [u8; 4],                // 0x4A84 - 0x4A87
    
    /// DMA Tx TCP Flags Control Low
    dtxtcpflgl:                         Volatile<u32>,          // 0x4A88;
    
    /// DMA Tx TCP Flags Control High
    dtxtcpflgh:                         Volatile<u32>,          // 0x4A8C;
    _padding18:                         [u8; 1392],             // 0x4A90 - 0x4FFF

    /// Receive Checksum Control
    rxcsum:                             Volatile<u32>,          // 0x5000
    _padding19:                         [u8; 124],              // 0x5004 - 0x507F

    /// Filter Control Register
    fctrl:                              Volatile<u32>,          // 0x5080;
    _padding20:                         [u8; 164],              // 0x5084 - 0x5127

    /// EType Queue Filter
    etqf:                               [Volatile<u32>;8],      // 0x5128 - 0x5147;
    _padding21:                         [u8; 3768],             // 0x5148 - 0x5FFF
} // 4 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters2>(), 4 * 4096);

impl IntelIxgbeRegisters2 {
    pub fn rdrxctl_read(&self) -> u32 {
        self.rdrxctl.read()
    }

    pub fn rdrxctl_crc_strip(&mut self) {
        let init_val = 0x0600_8800;
        self.rdrxctl.write(init_val | RDRXCTL_CRC_STRIP);
    }

    pub fn rxctrl_rx_enable(&mut self) {
        let val = self.rxctrl.read();
        self.rxctrl.write(val | RECEIVE_ENABLE); 
    }

    pub fn rxctrl_rx_disable(&mut self) {
        let val = self.rxctrl.read();
        self.rxctrl.write(val & !RECEIVE_ENABLE); 
    }

    pub fn fcrtl_clear(&mut self) {
        for fcrtl in self.fcrtl.iter_mut() {
            fcrtl.write(0);
        }
    } 
    
    pub fn fcrth_clear(&mut self) {
        for fcrth in self.fcrth.iter_mut() {
            fcrth.write(0);
        }
    } 

    pub fn fcrtv_clear(&mut self) {
        self.fcrtv.write(0);
    }

    pub fn fccfg_clear(&mut self) {
        self.fccfg.write(0);
    }

    pub fn rxpbsize_set_buffer_size(&mut self, reg_idx: usize, size: u32) -> Result<(), &'static str> {
        let size_mask = 0x3FF; // 10 bits
        if size & !size_mask != 0 {
            Err("invalid write to rxpbsize")
        } else {
            self.rxpbsize[reg_idx].write(size << 10);
            Ok(())
        }
    }

    pub fn hlreg0_crc_strip(&mut self) {
        self.hlreg0.write(self.hlreg0.read() | HLREG0_CRC_STRIP);
    }

    pub fn hlreg0_crc_en(&mut self) {
        self.hlreg0.write(self.hlreg0.read() | HLREG0_TXCRCEN);
    }

    pub fn hlreg0_tx_pad_en(&mut self) {
        self.hlreg0.write(self.hlreg0.read() | HLREG0_TXPADEN);
    }

    pub fn autoc2_read(&self) -> u32 {
        self.autoc2.read()
    }

    //TODO: can prob make this a more streamlined function
    pub fn autoc2_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x5007_0000;
        if val & !write_mask != 0 {
            Err("invalid write to AUTOC2")
        } else {
            self.autoc2.write(val);
            Ok(())
        }
    }

    // TODO: check rxctrl.rxen set to 0
    pub fn fctrl_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x0000_0702;
        if val & !write_mask != 0 {
            Err("invalid write to FCTRL")
        } else {
            self.fctrl.write(val);
            Ok(())
        }
    }

    pub fn rttdcs_set_arbdis(&mut self) {
        self.rttdcs.write(self.rttdcs.read() | RTTDCS_ARBDIS);
    }

    pub fn rttdcs_clear_arbdis(&mut self) {
        self.rttdcs.write(self.rttdcs.read() & !RTTDCS_ARBDIS);
    }

    pub fn dmatxctl_disable_tx(&mut self) {
        self.dmatxctl.write(self.dmatxctl.read() & !TE); 
    }

    pub fn dmatxctl_enable_tx(&mut self) {
        self.dmatxctl.write(self.dmatxctl.read() | TE); 
    }

    pub fn rxcsum_enable_rss_writeback(&mut self) {
        self.rxcsum.write(RXCSUM_PCSD);
    }
}

/// The layout in memory of the transmit queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub(crate) struct IntelIxgbeTxRegisters {
    /// Set of registers for 128 transmit descriptor queues
    pub tx_regs:                        [RegistersTx; 128],     // 0x6000 - 0x7FFF
} // 2 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeTxRegisters>(), 2 * 4096);

/// The layout in memory of the set of registers containing the MAC address of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeMacRegisters {
    _padding1:                          [u8; 256],              // 0x8000 - 0x80FF
    /// DMA Tx TCP Max Allow Size Requests
    dtxmxszrq:                          Volatile<u32>,          // 0X8100
    _padding2:                          [u8; 8444],             // 0x8104 - 0xA1FF
    
    /// Receive Address Low
    pub ral:                            ReadOnly<u32>,          // 0xA200;
    
    /// Receive Address High
    pub rah:                            ReadOnly<u32>,          // 0xA204;
    _padding3:                          [u8; 10744],            // 0xA208 - 0xCBFF

    /// Transmit Packet Buffer Size
    txpbsize:                           [Volatile<u32>;8],      // 0xCC00
    _padding4:                          [u8; 992],              // 0xCC20 - 0xCFFF
} // 5 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeMacRegisters>(), 5 * 4096);

impl IntelIxgbeMacRegisters {
    pub fn dtxmxszrq_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x0000_0FFF;
        if val & !write_mask != 0 {
            Err("invalid write to DTXMXSZRQ")
        } else {
            self.dtxmxszrq.write(val);
            Ok(())
        }
    }

    pub fn txpbsize_write(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        const NUM_TXPBSIZE_REGS: usize = 8;
        let write_mask = 0x000F_FC00; 

        if (val & !write_mask != 0) || (reg_idx >= NUM_TXPBSIZE_REGS) {
            Err("failed to write to TXPBSIZE reg")
        } else {
            self.txpbsize[reg_idx].write(val);
            Ok(())
        }
    }
}

/// The layout in memory of the second set of receive queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRxRegisters2 {
    /// Second set of Rx Registers for 64 Rx Queues
    pub rx_regs2:                       [RegistersRx; 64],      // 0xD000 - 0xDFFF, for 64 queues
} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRxRegisters2>(), 4096);

/// The layout in memory of the third set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters3 {
    /// Source Address Queue Filter
    pub saqf:                           [Volatile<u32>;128],    // 0xE000 - 0xE1FF
    
    /// Destination Address Queue Filter
    pub daqf:                           [Volatile<u32>;128],    // 0xE200 - 0xE3FF
    
    /// Source Destination Port Queue Filter
    pub sdpqf:                          [Volatile<u32>;128],    // 0xE400 - 0xE5FF
    
    /// Five Tuple Queue Filter
    ftqf:                               [Volatile<u32>;128],    // 0xE600 - 0xE7FF
    
    /// L3 L4 Tuples Immediate Interrupt Rx 
    l34timir:                           [Volatile<u32>;128],    // 0xE800 - 0xE9FF

    _padding1:                          [u8; 256],              // 0xEA00 - 0xEAFF

    /// Redirection Table
    reta:                               [Volatile<u32>;32],     // 0xEB00 - 0xEB7F

    /// RSS Random Key Register
    pub rssrk:                          [Volatile<u32>;10],     // 0xEB80 - 0xEBA7
    _padding2:                          [u8; 88],               // 0xEBA8 - 0xEBFF

    /// EType Queue Select
    etqs:                               [Volatile<u32>;8],      // 0xEC00 - 0xEC1F;
    _padding3:                          [u8; 96],               // 0xEC20 - 0xEC7F

    /// Multiple Receive Queues Command Register
    mrqc:                               Volatile<u32>,          // 0xEC80;
    _padding4:                          [u8; 5004],             // 0xEC84 - 0x1000F

    /// EEPROM/ Flash Control Register
    eec:                                Volatile<u32>,          // 0x10010

    /// EEPROM Read Register
    eerd:                               Volatile<u32>,          // 0x10014;
    _padding5:                          [u8; 296],              // 0x10018 - 0x1013F

    /// Software Semaphore Register
    swsm:                               Volatile<u32>,          // 0x10140
    _padding6:                          [u8; 28],               // 0x10144 - 0x1015F

    /// Software Firmware Synchronization
    sw_fw_sync:                         Volatile<u32>,          // 0x10160 
    _padding7:                          [u8; 3852],             // 0x10164 - 0x1106F

    /// DCA Requester ID Information Register
    dca_id:                             ReadOnly<u32>,          // 0x11070

    /// DCA Control Register
    dca_ctrl:                           Volatile<u32>,          // 0x11074
    _padding8:                          [u8; 61320],            // 0x11078 - 0x1FFFF

} // 18 4KiB page (total NIC mem = 128 KB)

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters3>(), 18 * 4096);

impl IntelIxgbeRegisters3 {
    pub fn ftqf_write(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        const NUM_FTQF_REGS: usize = 128;
        let write_mask = 0xFFFF_FF1F;
        if (val & !write_mask != 0) || (reg_idx >= NUM_FTQF_REGS) {
            Err("failed to write to FTQF reg")
        } else {
            self.ftqf[reg_idx].write(val);
            Ok(())
        }
    }

    pub fn ftqf_disable_filter(&mut self, reg_idx: usize) {
        let val = self.ftqf[reg_idx].read();
        self.ftqf[reg_idx].write(val & !FTQF_Q_ENABLE);
    }   

    pub fn l34timir_write(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        const NUM_L34TIMIR_REGS: usize = 128;
        let write_mask = 0x0FF0_1000;
        let reserved_val = 0x40 << 13;
        if (val & !write_mask != 0) || (reg_idx >= NUM_L34TIMIR_REGS) {
            Err("failed to write to L34TIMIR reg")
        } else {
            self.l34timir[reg_idx].write(val | reserved_val);
            Ok(())
        }
    }

    pub fn reta_write(&mut self, reg_idx: usize, val: u32) -> Result<(), &'static str> {
        const NUM_RETA_REGS: usize = 32;
        let write_mask = 0x0F0F_0F0F;
        if (val & !write_mask != 0) || (reg_idx >= NUM_RETA_REGS) {
            Err("failed to write to RETA reg")
        } else {
            self.reta[reg_idx].write(val);
            Ok(())
        }
    }

    pub fn mrqc_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0xFFFF_000F;
        if val & !write_mask != 0 {
            Err("failed to write to MRQC reg")
        } else {
            self.mrqc.write(val);
            Ok(())
        }
    }

    pub fn eec_auto_read(&self) -> bool {
        self.eec.read().get_bit(EEC_AUTO_RD as u8)
    }

    // TODO: make semaphore code intralingual
    pub fn swsm_read(&self) -> u32 {
        self.swsm.read()
    }

    pub fn swsm_smbi_write(&mut self, set: bool) {
        let mut val = self.swsm.read();
        self.swsm.write(*val.set_bit(0, set))
    }

    pub fn swsm_swesmbi_write(&mut self, set: bool) {
        let mut val = self.swsm.read();
        self.swsm.write(*val.set_bit(1, set))
    }

    pub fn sw_fw_sync_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x000_03FF;
        if val & !write_mask != 0 {
            Err("failed to write to SW_FW_SYNC reg")
        } else {
            self.sw_fw_sync.write(val);
            Ok(())
        }
    }

    pub fn sw_fw_sync_read(&self) -> u32 {
        self.sw_fw_sync.read()
    }

    pub fn dca_ctrl_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x000_001F;
        if val & !write_mask != 0 {
            Err("failed to write to DCA_CTRL reg")
        } else {
            self.dca_ctrl.write(val);
            Ok(())
        }
    }
}

// check that the sum of all the register structs is equal to the memory of the ixgbe device (128 KiB).
const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters1>() + core::mem::size_of::<IntelIxgbeRxRegisters1>() +
    core::mem::size_of::<IntelIxgbeRegisters2>() + core::mem::size_of::<IntelIxgbeTxRegisters>() + 
    core::mem::size_of::<IntelIxgbeMacRegisters>() + core::mem::size_of::<IntelIxgbeRxRegisters2>() +
    core::mem::size_of::<IntelIxgbeRegisters3>(), 0x20000);

/// Set of registers associated with one transmit descriptor queue.
#[derive(FromBytes)]
#[repr(C)]
pub(crate) struct RegistersTx {
    /// Transmit Descriptor Base Address Low
    pub tdbal:                          Volatile<u32>,        // 0x6000

    /// Transmit Descriptor Base Address High
    pub tdbah:                          Volatile<u32>,        // 0x6004
    
    /// Transmit Descriptor Length    
    pub tdlen:                          Volatile<u32>,        // 0x6008

    /// Tx DCA Control Register
    dca_txctrl:                         Volatile<u32>,          // 0x600C

    /// Transmit Descriptor Head
    pub tdh:                            Volatile<u32>,          // 0x6010
    _padding0:                          [u8; 4],                // 0x6014 - 0x6017

    /// Transmit Descriptor Tail
    pub tdt:                            Volatile<u32>,          // 0x6018
    _padding1:                          [u8; 12],               // 0x601C - 0x6027

    /// Transmit Descriptor Control
    txdctl:                             Volatile<u32>,          // 0x6028
    _padding2:                          [u8; 12],               // 0x602C - 0x6037

    /// Transmit Descriptor Completion Write Back Address Low
    tdwbal:                             Volatile<u32>,          // 0x6038

    /// Transmit Descriptor Completion Write Back Address High
    tdwbah:                             Volatile<u32>,          // 0x603C
} // 64B

const_assert_eq!(core::mem::size_of::<RegistersTx>(), 64);

// TODO: can probably hvave access functions for remaining regsiters
impl RegistersTx {
    pub fn txdctl_read(&self) -> u32 {
        self.txdctl.read()
    }

    pub fn txdctl_txq_enable(&mut self) {
        let val = self.txdctl.read();
        self.txdctl.write(val | TX_Q_ENABLE); 
    }
}

/// Set of registers associated with one receive descriptor queue.
#[derive(FromBytes)]
#[repr(C)]
pub struct RegistersRx {
    /// Receive Descriptor Base Address Low
    pub rdbal:                          Volatile<u32>,        // 0x1000

    /// Recive Descriptor Base Address High
    pub rdbah:                          Volatile<u32>,        // 0x1004

    /// Recive Descriptor Length
    pub rdlen:                          Volatile<u32>,        // 0x1008

    /// Rx DCA Control Register
    dca_rxctrl:                         Volatile<u32>,          // 0x100C

    /// Recive Descriptor Head
    pub rdh:                            Volatile<u32>,          // 0x1010

    /// Split Receive Control Registers
    srrctl:                             Volatile<u32>,          // 0x1014 //specify descriptor type

    /// Receive Descriptor Tail
    pub rdt:                            Volatile<u32>,          // 0x1018
    _padding1:                          [u8;12],                // 0x101C - 0x1027

    /// Receive Descriptor Control
    rxdctl:                             Volatile<u32>,          // 0x1028
    _padding2:                          [u8;20],                // 0x102C - 0x103F                                            
} // 64B

const_assert_eq!(core::mem::size_of::<RegistersRx>(), 64);

impl RegistersRx {
    pub fn dca_rxctrl_read(&self) -> u32 {
        self.dca_rxctrl.read()
    }

    pub fn dca_rxctrl_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0xFF00_A2E0;
        if val & !write_mask != 0 {
            Err("failed to write to DCA_RXCTRL reg")
        } else {
            self.dca_rxctrl.write(val);
            Ok(())
        }
    }
    
    pub fn srrctl_read(&self) -> u32 {
        self.srrctl.read()
    }

    //TODO: can implement more checks
    pub fn srrctl_write(&mut self, val: u32) -> Result<(), &'static str> {
        let write_mask = 0x1FC0_3F1F;
        if val & !write_mask != 0 {
            Err("failed to write to SRRCTL reg")
        } else {
            self.srrctl.write(val);
            Ok(())
        }
    }

    pub fn rxdctl_read(&self) -> u32 {
        self.rxdctl.read()
    }

    pub fn rxdctl_rxq_enable(&mut self) {
        let val = self.rxdctl.read();
        self.rxdctl.write(val | RX_Q_ENABLE); 
    }
}

/// Offset where the RDT register starts for the first 64 rx queues
pub const RDT_1:                        usize = 0x1018;
/// Offset where the RDT register starts for the second set of 64 rx queues
pub const RDT_2:                        usize = 0xD018;
/// Number of bytes between consecutive RDT registers
pub const RDT_DIST:                     usize = 0x40;
/// Offset where the TDT register starts for the first 64 queues
pub const TDT:                          usize = 0x6018;
/// Number of bytes between consecutive TDT registers
pub const TDT_DIST:                     usize = 0x40;

// Link set up commands
pub const AUTOC_LMS_CLEAR:              u32 = 0x0000_E000; 
pub const AUTOC_LMS_1_GB:               u32 = 0x0000_E000;
pub const AUTOC_LMS_10_GBE_P:           u32 = 1 << 13;
pub const AUTOC_LMS_10_GBE_S:           u32 = 3 << 13;
pub const AUTOC_LMS_KX_KX4_AUTONEG:     u32 = 6<<13; //KX/KX4//KR
pub const AUTOC_FLU:                    u32 = 1;
pub const AUTOC_RESTART_AN:             u32 = 1<<12;
pub const AUTOC_1G_PMA_PMD:             u32 = 0x0000_0200; //clear bit 9
pub const AUTOC_10G_PMA_PMD_CLEAR:      u32 = 0x0000_0180; 
pub const AUTOC_10G_PMA_PMD_XAUI:       u32 = 0 << 7; 
pub const AUTOC2_10G_PMA_PMD_S_CLEAR:   u32 = 0x0003_0000; //clear bits 16 and 17 
pub const AUTOC2_10G_PMA_PMD_S_SFI:     u32 = 1 << 17;

// CTRL commands
pub const CTRL_LRST:                    u32 = 1<<3; 
pub const CTRL_RST:                     u32 = 1<<26;

// semaphore commands
pub const SWSM_SMBI:                    u32 = 1 << 0;
pub const SWSM_SWESMBI:                 u32 = 1 << 1;
pub const SW_FW_SYNC_SMBITS_MASK:       u32 = 0x3FF;
pub const SW_FW_SYNC_SMBITS_SW:         u32 = 0x1F;
pub const SW_FW_SYNC_SMBITS_FW:         u32 = 0x3E0;
pub const SW_FW_SYNC_SW_MAC:            u32 = 1 << 3;
pub const SW_FW_SYNC_FW_MAC:            u32 = 1 << 8;

// EEPROM Commands
/// Bit which indicates that auto-read by hardware from EEPROM is done
pub const EEC_AUTO_RD:                  u32 = 9;

// Link Commands
pub const LINKS_SPEED_MASK:             u32 = 0x3 << 28;

// MAC Control Commands
/// Tx CRC Enable by HW (bit 0)
pub const HLREG0_TXCRCEN:               u32 = 1;
/// Tx Pad Frame Enable (bit 10)
pub const HLREG0_TXPADEN:               u32 = 1 << 10;
/// Enable CRC strip by HW
pub const HLREG0_CRC_STRIP:             u32 = 1 << 1;
/// Enable CRC strip by HW
pub const RDRXCTL_CRC_STRIP:            u32 = 1;
/// These 5 bits have to be cleared by software
pub const RDRXCTL_RSCFRSTSIZE:          u32 = 0x1F << 17;

/// DCB Arbiters Disable
pub const RTTDCS_ARBDIS:                u32 = 1 << 6;

/// For DCB and VT disabled, set TXPBSIZE.SIZE to 160KB
pub const TXPBSIZE_160KB:                u32 = 0xA0 << 10;
/// For DCB and VT disabled, set RXPBSIZE.SIZE to 512KB
pub const RXPBSIZE_512KB:                u32 = 0x200;

// RCTL commands
pub const BSIZEPACKET_8K:               u32 = 8;
pub const BSIZEHEADER_256B:             u32 = 4;
pub const BSIZEHEADER_0B:               u32 = 0;
pub const DESCTYPE_LEG:                 u32 = 0;
pub const DESCTYPE_ADV_1BUFFER:         u32 = 1;
pub const DESCTYPE_ADV_HS:              u32 = 2;
pub const RX_Q_ENABLE:                  u32 = 1 << 25;
pub const STORE_BAD_PACKETS:            u32 = 1 << 1;
pub const MULTICAST_PROMISCUOUS_ENABLE: u32 = 1 << 8;
pub const UNICAST_PROMISCUOUS_ENABLE:   u32 = 1 << 9;
pub const BROADCAST_ACCEPT_MODE:        u32 = 1 << 10;
pub const RECEIVE_ENABLE:               u32 = 1;
pub const DROP_ENABLE:                  u32 = 1 << 28;
pub const DCA_RXCTRL_CLEAR_BIT_12:      u32 = 1 << 12;
pub const CTRL_EXT_NO_SNOOP_DIS:        u32 = 1 << 16;

// RSS commands
pub const RXCSUM_PCSD:                  u32 = 1 << 13; 
pub const MRQC_MRQE_RSS:                u32 = 1; // set bits 0..3 in MRQC
pub const MRQC_TCPIPV4:                 u32 = 1 << 16; 
pub const MRQC_IPV4:                    u32 = 1 << 17; 
pub const MRQC_IPV6:                    u32 = 1 << 20;
pub const MRQC_TCPIPV6:                 u32 = 1 << 21;  
pub const MRQC_UDPIPV4:                 u32 = 1 << 22; 
pub const MRQC_UDPIPV6:                 u32 = 1 << 23;  
pub const RETA_ENTRY_0_OFFSET:          u32 = 0;
pub const RETA_ENTRY_1_OFFSET:          u32 = 8;
pub const RETA_ENTRY_2_OFFSET:          u32 = 16;
pub const RETA_ENTRY_3_OFFSET:          u32 = 24;

// DCA commands
pub const RX_DESC_DCA_ENABLE:           u32 = 1 << 5;
pub const RX_HEADER_DCA_ENABLE:         u32 = 1 << 6;
pub const RX_PAYLOAD_DCA_ENABLE:        u32 = 1 << 7;
pub const RX_DESC_R_RELAX_ORDER_EN:     u32 = 1 << 9;
pub const RX_DATA_W_RELAX_ORDER_EN:     u32 = 1 << 13;
pub const RX_SP_HEAD_RELAX_ORDER_EN:    u32 = 1 << 15;
pub const DCA_CPUID_SHIFT:              u32 = 24;
pub const DCA_CTRL_ENABLE:              u32 = 0;
pub const DCA_MODE_1:                   u32 = 0 << 1;  
pub const DCA_MODE_2:                   u32 = 1 << 1;

// 5-tuple Queue Filter commands
pub const SPDQF_SOURCE_SHIFT:           u32 = 0;
pub const SPDQF_DEST_SHIFT:             u32 = 16;
pub const FTQF_PROTOCOL:                u32 = 3;
pub const FTQF_PROTOCOL_TCP:            u32 = 0;
pub const FTQF_PROTOCOL_UDP:            u32 = 1;
pub const FTQF_PROTOCOL_SCTP:           u32 = 2;
pub const FTQF_PRIORITY:                u32 = 7;
pub const FTQF_PRIORITY_SHIFT:          u32 = 2;
pub const FTQF_SOURCE_ADDRESS_MASK:     u32 = 1 << 25;
pub const FTQF_DEST_ADDRESS_MASK:       u32 = 1 << 26;
pub const FTQF_SOURCE_PORT_MASK:        u32 = 1 << 27;
pub const FTQF_DEST_PORT_MASK:          u32 = 1 << 28;
pub const FTQF_PROTOCOL_MASK:           u32 = 1 << 29;
pub const FTQF_POOL_MASK:               u32 = 1 << 30;
pub const FTQF_Q_ENABLE:                u32 = 1 << 31;
pub const L34TIMIR_BYPASS_SIZE_CHECK:   u32 = 1 << 12;
pub const L34TIMIR_RESERVED:            u32 = 0x40 << 13;
pub const L34TIMIR_LLI_ENABLE:          u32 = 1 << 20;
pub const L34TIMIR_RX_Q_SHIFT:          u32 = 21;

 
// Buffer Sizes
pub const RCTL_BSIZE_256:               u32 = 3 << 16;
pub const RCTL_BSIZE_512:               u32 = 2 << 16;
pub const RCTL_BSIZE_1024:              u32 = 1 << 16;
pub const RCTL_BSIZE_2048:              u32 = 0 << 16;
pub const RCTL_BSIZE_4096:              u32 = (3 << 16) | (1 << 25);
pub const RCTL_BSIZE_8192:              u32 = (2 << 16) | (1 << 25);
pub const RCTL_BSIZE_16384:             u32 = (1 << 16) | (1 << 25);
  
 
/// Enable a transmit queue
pub const TX_Q_ENABLE:                  u32 = 1 << 25;
/// Transmit Enable
pub const TE:                           u32  = 1;           
pub const DTXMXSZRQ_MAX_BYTES:          u32 = 0xFFF;

/// Tx descriptor pre-fetch threshold (value taken from DPDK)
pub const TXDCTL_PTHRESH:               u32 = 36; 
/// Tx descriptor host threshold (value taken from DPDK)
pub const TXDCTL_HTHRESH:               u32 = 8 << 8; 
/// Tx descriptor write-back threshold (value taken from DPDK)
pub const TXDCTL_WTHRESH:               u32 = 4 << 16; 

// Interrupt Register Commands 
pub const DISABLE_INTERRUPTS:           u32 = 0x7FFFFFFF; 
/// MSI-X Mode
pub const GPIE_MULTIPLE_MSIX:           u32 = 1 << 4;
/// EICS Immediate Interrupt Enable
pub const GPIE_EIMEN:                   u32 = 1 << 6;
/// Should be set in MSIX mode and cleared in legacy/msi mode
pub const GPIE_PBA_SUPPORT:             u32 = 1 << 31;
/// Each bit enables auto clear of the corresponding RTxQ bit in the EICR register following interrupt assertion
pub const EIAC_RTXQ_AUTO_CLEAR:         u32 = 0xFFFF;
/// Bit position where the throttling interval is written
pub const EITR_ITR_INTERVAL_SHIFT:      u32 = 3;
/// Enables the corresponding interrupt in the EICR register by setting the bit
pub const EIMS_INTERRUPT_ENABLE:        u32 = 1;

/// The number of msi-x vectors this device can have. 
/// It can be set from PCI space, but we took the value from the data sheet.
pub const IXGBE_MAX_MSIX_VECTORS:     usize = 64;

/// Table that contains msi-x vector entries. 
/// It is mapped to a physical memory region specified by the BAR from the PCI space.
#[derive(FromBytes)]
#[repr(C)]
pub struct MsixVectorTable {
    pub msi_vector:     [MsixVectorEntry; IXGBE_MAX_MSIX_VECTORS],
}

/// A single Message Signaled Interrupt entry.
/// It contains the interrupt number for this vector and the core this interrupt is redirected to.
#[derive(FromBytes)]
#[repr(C)]
pub struct MsixVectorEntry {
    /// The lower portion of the address for the memory write transaction.
    /// This part contains the apic id which the interrupt will be redirected to.
    pub msg_lower_addr:         Volatile<u32>,
    /// The upper portion of the address for the memory write transaction.
    pub msg_upper_addr:         Volatile<u32>,
    /// The data portion of the msi vector which contains the interrupt number.
    pub msg_data:               Volatile<u32>,
    /// The control portion which contains the interrupt mask bit.
    pub vector_control:         Volatile<u32>,
}

/// A constant which indicates the region that is reserved for interrupt messages
pub const MSIX_INTERRUPT_REGION:    u32 = 0xFEE << 20;
/// The location in the lower address register where the destination core id is written
pub const MSIX_DEST_ID_SHIFT:       u32 = 12;
/// The bits in the lower address register that need to be cleared and set
pub const MSIX_ADDRESS_BITS:        u32 = 0xFFFF_FFF0;
/// Clear the vector control field to unmask the interrupt
pub const MSIX_UNMASK_INT:          u32 = 0;