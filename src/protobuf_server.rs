use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::serial::{Read, Write};

use crate::protobuf::messages::{self, I2cRequest, I2cResponse};

use crate::protobuf;

pub type I2CBussesList<'a, E> = [&'a mut dyn WriteRead<Error = E>];

enum State {
    Idle,
    ProcessBody,
}

pub fn process<S, E, I2CE>(
    stream: &mut S,
    now: u64,
    current_control_state: &mut protobuf::Control,
    i2c_buses: &mut I2CBussesList<'_, I2CE>,
    force_reset: bool,
) -> nb::Result<(u64, bool), E>
where
    S: Read<u8, Error = E> + Write<u8, Error = E>,
{
    static mut STATE: State = State::Idle;
    static mut DATA_SIZE: usize = 0;

    loop {
        if force_reset {
            unsafe {
                STATE = State::Idle;
            }
        }

        match unsafe { &*core::ptr::addr_of_mut!(STATE) } {
            State::Idle => {
                let data_size =
                    protobuf::recive_md_header(stream, core::mem::size_of::<messages::Request>())?;
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
                        defmt::info!("Protobuf Request Id: {}...", request.id);
                        let mut resp = protobuf::default_response(request.id, now);
                        protobuf::process_requiest(&request, &mut resp, current_control_state);

                        let need_reset = !process_i2c(&request.i2c, &mut resp.i2c, i2c_buses);
                        if need_reset {
                            resp.global_status = messages::Status::I2c as i32;
                        }

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
                            return Ok((now, need_reset));
                        }
                    }
                    Err(e) => {
                        // error while decoding
                        defmt::error!(
                            "Protobuf: Error while decoding request: {}",
                            defmt::Display2Format(&e)
                        );
                        unsafe { STATE = State::Idle };
                    }
                }
            }
        }
    }
}

/// Process I2C request
/// req - request
/// resp - response
/// i2c_buses - list of I2C buses available
/// return true if no errors
fn process_i2c<I2CE>(
    req: &Option<I2cRequest>,
    resp: &mut Option<I2cResponse>,
    i2c_buses: &mut I2CBussesList<'_, I2CE>,
) -> bool {
    use prost::alloc::vec::Vec;

    use messages::{i2c_operation::Operation, i2c_request::Request, i2c_response::Response};

    let buses_len = i2c_buses.len();
    let verify_input =
        move |bus_id: u32, data_len: usize, r: &mut messages::I2cSequenceResult| -> bool {
            if bus_id >= buses_len as u32 {
                r.operations.push(messages::I2cResult {
                    operation: Some(messages::i2c_result::Operation::Write(
                        messages::I2cResultCode::I2cInvalidBus as i32,
                    )),
                });
                defmt::error!("Invalid bus id: {}, abort transaction", bus_id);
                return false;
            }
            if data_len > 254 {
                r.operations.push(messages::I2cResult {
                    operation: Some(messages::i2c_result::Operation::Write(
                        messages::I2cResultCode::I2cTooLongData as i32,
                    )),
                });
                defmt::error!("Too long data to process ({})", data_len);
                return false;
            }

            true
        };

    fn bool2res(v: bool) -> &'static str {
        if v {
            "Success"
        } else {
            "Error"
        }
    }

    if let Some(req) = req {
        match &req.request {
            Some(Request::Enumerate(_empty)) => {
                let r = Response::Enumerate(messages::I2cEnumerateResponse {
                    buses: i2c_buses
                        .iter()
                        .enumerate()
                        .map(|(i, _b)| messages::I2cBus {
                            bus: i as u32,
                            max_speed: 100000,
                        })
                        .collect::<Vec<messages::I2cBus>>(),
                });
                defmt::debug!("I2C: Enumerating buses");
                resp.replace(I2cResponse { response: Some(r) });

                true
            }
            Some(Request::Sequence(s)) => {
                let mut r = messages::I2cSequenceResult {
                    bus: s.bus,
                    address: s.address,
                    operations: Vec::new(),
                };

                let mut error_found = false;
                for op in &s.operations {
                    if error_found {
                        // skip all operations after error
                        break;
                    }
                    match &op.operation {
                        Some(Operation::Write(req)) => {
                            let wr_len = req.data.len();
                            if !verify_input(s.bus, wr_len, &mut r) {
                                continue;
                            }
                            let res = i2c_buses[s.bus as usize].write_read(
                                s.address as u8,
                                &req.data,
                                &mut [],
                            );
                            r.operations.push(messages::I2cResult {
                                operation: Some(messages::i2c_result::Operation::Write(
                                    match &res {
                                        Ok(_) => messages::I2cResultCode::I2cOk as i32,
                                        Err(_e) => {
                                            error_found = true;
                                            messages::I2cResultCode::I2cNak as i32
                                        }
                                    },
                                )),
                            });
                            defmt::debug!(
                                "I2C: Write [{}:{:02X}] <= {} bytes: {}",
                                s.bus,
                                s.address,
                                wr_len,
                                bool2res(res.is_ok())
                            );
                        }
                        Some(Operation::Read(req)) => {
                            if !verify_input(s.bus, req.length as usize, &mut r) {
                                continue;
                            }
                            let mut data = Vec::new();
                            data.resize(req.length as usize, 0);
                            let res = i2c_buses[s.bus as usize].write_read(
                                s.address as u8,
                                &[],
                                &mut data,
                            );
                            r.operations.push(messages::I2cResult {
                                operation: Some(messages::i2c_result::Operation::Read(
                                    messages::I2cReadResponse {
                                        data,
                                        status: match &res {
                                            Ok(_) => messages::I2cResultCode::I2cOk as i32,
                                            Err(_e) => {
                                                error_found = true;
                                                messages::I2cResultCode::I2cNak as i32
                                            }
                                        },
                                    },
                                )),
                            });
                            defmt::debug!(
                                "I2C: Read [{}:{:02X}] => {} bytes: {}",
                                s.bus,
                                s.address,
                                req.length,
                                bool2res(res.is_ok())
                            );
                        }
                        None => {}
                    }
                }

                resp.replace(I2cResponse {
                    response: Some(Response::Sequence(r)),
                });

                !error_found
            }
            None => true,
        }
    } else {
        true
    }
}
