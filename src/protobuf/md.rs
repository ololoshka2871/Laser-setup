use core::usize;

use nb;

use embedded_hal::serial::Read;

enum DecodeStage {
    Magick,
    Size(usize),
}

pub fn recive_md_header<S, E>(stream: &mut S, max_len: usize) -> nb::Result<usize, E>
where
    S: Read<u8, Error = E>,
{
    static mut STAGE: DecodeStage = DecodeStage::Magick;
    static mut SIZE_BYTES: [u8; 4] = [0; 4];

    loop {
        match { unsafe { &mut STAGE } } {
            DecodeStage::Magick => {
                match stream.read() {
                    Ok(magick) => {
                        if magick != super::messages::Info::Magick as u8 {
                            defmt::error!("Invalid magick: {}", magick);
                            continue; // invalid magick continue to next byte
                        } else {
                            // valid magick, continue to next stage
                            defmt::trace!("Magick valid");
                            unsafe {
                                STAGE = DecodeStage::Size(0);
                            }
                        }
                    }
                    Err(e) => return Err(e),
                }
            }
            DecodeStage::Size(byte_number) => match stream.read() {
                Ok(b) => {
                    unsafe {
                        SIZE_BYTES[*byte_number] = b;
                    }
                    *byte_number += 1;
                    if byte_number == &4 {
                        if let Ok(v) = prost::decode_length_delimiter(unsafe { &SIZE_BYTES[..] }) {
                            if v > 0 && v <= max_len {
                                defmt::trace!("Message size: {}", v);
                                return Ok(v);
                            } else {
                                defmt::error!("Invalid message size: {}", v);
                            }
                        }
                        unsafe {
                            STAGE = DecodeStage::Magick;
                        }
                    }
                }
                Err(e) => return Err(e),
            },
        }
    }
}
