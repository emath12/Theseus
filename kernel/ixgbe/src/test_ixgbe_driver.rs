use super::{IXGBE_NIC, NetworkInterfaceCard, TransmitBuffer};

//TODO: should probably have one testing suite for both drivers
pub fn test_nic_ixgbe_driver(_: Option<u64>) {
    match dhcp_request_packet() {
        Ok(_) => debug!("test_ixgbe_nic_driver(): sent DHCP request packet successfully!"),
        Err(e) => error!("test_ixgbe_nic_driver(): failed to send DHCP request packet: error {:?}", e),
    };
}

//should test packet transmission and reception as QEMU DHCP server will respond

//will only b able to see this Tx message in netdump.pcap if user is not mentioned in QEMU flags of Makefile
//QEMU_FLAGS += -net nic,vlan=0,model=e1000,macaddr=00:0b:82:01:fc:42 -net dump,file=netdump.pcap

//will only receive a response if user is mentioned in qemu flags
//QEMU_FLAGS += -net nic,vlan=1,model=e1000,macaddr=00:0b:82:01:fc:42 -net user,vlan=1 -net dump,file=netdump.pcap

//or else use a tap interface (default)
//QEMU_FLAGS += -device e1000,netdev=network0,mac=52:55:00:d1:55:01 -netdev tap,id=network0,ifname=tap0,script=no,downscript=no
//will receive a DHCP messgae from 00:1f:c6:9c:89:4c

pub fn dhcp_request_packet() -> Result<(), &'static str> {
    let transmit_buffer = create_test_packet()?;
    let mut ixgbe_nc = IXGBE_NIC.try().ok_or("ixgbe NIC hasn't been initialized yet")?;
    ixgbe_nc.lock().send_packet(transmit_buffer)
}

pub fn create_test_packet() -> Result<TransmitBuffer, &'static str> {
    let packet: [u8; 314] = [
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xc6, 0x9c, 0x89, 0x4c, 0x08, 0x00, 0x45,
        0x00, 0x01, 0x2c, 0xa8, 0x36, 0x00, 0x00, 0xfa, 0x11, 0x17, 0x8b, 0x00, 0x00, 0x00, 0x00,
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
    let mut transmit_buffer = TransmitBuffer::new(packet.len() as u16)?;
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(0, 314)?;
        buffer.copy_from_slice(&packet);
    }
    Ok(transmit_buffer)
}

pub fn create_raw_packet(dest_mac_address: &[u8], source_mac_address: &[u8], message: &[u8]) -> Result<TransmitBuffer, &'static str> {
    let len = message.len() as u16;

    if len > 1500 {
        return Err("Too long for a raw packet");
    }

    const ETHERNET_HEADER_LEN: u16 = 6 + 6 + 2;
    let ether_type: [u8; 2] = [(len >> 8) as u8, len as u8];

    // let mut packet = dest_mac_address.to_vec();
    // for i in 0..6 {
    //     packet.push(source_mac_address[i])
    // }

    // let len: u16 = message.len();
    // packet.push((len >> 8) as u8);
    // packet.push(len as u8);

    // for i in 0..message.len() {
    //     packet.push(message[i]);
    // }

    let mut transmit_buffer = TransmitBuffer::new(ETHERNET_HEADER_LEN + message.len() as u16)?;
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(0, 6)?;
        buffer.copy_from_slice(&dest_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(6, 6)?;
        buffer.copy_from_slice(&source_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(12, 2)?;
        buffer.copy_from_slice(&ether_type);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(14, len as usize)?;
        buffer.copy_from_slice(&message);
    }

    Ok(transmit_buffer)
}