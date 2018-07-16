const RSSI_OFFSET: i16 = 74; // Table 31: Typical RSSI_offset Values

pub fn rssi_to_dbm(raw: u8) -> i16 {
    let rssi = i16::from(raw);
    // According to spec 17.3
    if rssi < 128 {
        rssi / 2 - RSSI_OFFSET
    } else {
        (rssi - 256) / 2 - RSSI_OFFSET
    }
}
