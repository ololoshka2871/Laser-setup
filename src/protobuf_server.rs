use nb;

use embedded_hal::serial::{Read, Write};

use crate::protobuf;

enum State {
    Idle,
    ProcessBody,
}

pub fn process<S, E>(stream: &mut S, now: u64) -> nb::Result<(), E>
where
    S: Read<u8, Error = E> + Write<u8, Error = E>,
{
    static mut STATE: State = State::Idle;
    static mut DATA_SIZE: usize = 0;

    loop {
        match unsafe { &STATE } {
            State::Idle => {
                let data_size = protobuf::recive_md_header(
                    stream,
                    core::mem::size_of::<protobuf::messages::Request>(),
                )?;
                defmt::trace!("Protobuf: Header recived, data size = {} bytes", data_size);
                unsafe {
                    STATE = State::ProcessBody;
                    DATA_SIZE = data_size
                };
            }
            State::ProcessBody => {
                let request = protobuf::recive_message_body(stream, unsafe { DATA_SIZE })?;
                match request {
                    Ok(request) => {
                        defmt::debug!("Protobuf Request Id: {}", request.id);
                        let mut resp = protobuf::default_response(request.id, now);
                        protobuf::process_requiest(request, &mut resp);
                        if let Some(r) = protobuf::encode_md_message(resp)
                            .as_slice()
                            .iter()
                            .map(|b| stream.write(*b))
                            .skip_while(|res| res.is_ok()) // skip while ok
                            .next()
                        {
                            // error while writing
                            return Err(unsafe { r.err().unwrap_unchecked() });
                        } else {
                            // ok
                            unsafe { STATE = State::Idle };
                        }
                    }
                    Err(e) => {
                        // error while decoding
                        defmt::error!("Protobuf: Error while decoding request: {}", defmt::Display2Format(&e));
                        unsafe { STATE = State::Idle };
                    }
                }
            }
        }
    }
}
