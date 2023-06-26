use core::sync::atomic::AtomicBool;

use defmt::Format;

use super::messages::{ActuatorState, ControlResponse, ValveState};

pub struct Control {
    pub actuator_state: ActuatorState,
    pub valve_state: ValveState,
    pub selected_channel: u32,
    updated: AtomicBool,
}

impl Control {
    pub fn reset(&mut self) {
        self.updated
            .store(false, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn is_updated(&self) -> bool {
        self.updated.load(core::sync::atomic::Ordering::Relaxed)
    }

    pub fn default() -> Self {
        Self {
            actuator_state: ActuatorState::Close,
            valve_state: ValveState::Atmosphere,
            selected_channel: 0,
            updated: AtomicBool::new(false),
        }
    }

    pub fn update(&mut self) {
        self.updated
            .store(true, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn into_response(&self) -> ControlResponse {
        ControlResponse {
            actuator_state: self.actuator_state as i32,
            valve_state: self.valve_state as i32,
            selected_channel: self.selected_channel,
        }
    }
}

impl Format for Control {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Control {{ actuator_state: {}, valve_state: {}, selected_channel: {} }}",
            self.actuator_state as u32,
            self.valve_state as u32,
            self.selected_channel
        )
    }
}

pub fn process_control(
    control: &super::messages::ControlRequest,
    current_control_state: &mut Control,
) {
    if let Some(actuator_state) = control.actuator_state {
        match actuator_state {
            0 => {
                current_control_state.actuator_state = ActuatorState::Close;
            }
            1 => {
                current_control_state.actuator_state = ActuatorState::Open;
            }
            _ => {
                defmt::error!("Protobuf: unknown actuator state: {}", actuator_state);
                return;
            }
        }
        current_control_state.update();
    }

    if let Some(valve_state) = control.valve_state {
        match valve_state {
            0 => {
                current_control_state.valve_state = ValveState::Atmosphere;
            }
            1 => {
                current_control_state.valve_state = ValveState::Vacuum;
            }
            _ => {
                defmt::error!("Protobuf: unknown valve state: {}", valve_state);
                return;
            }
        }
        current_control_state.update();
    }

    if let Some(selected_channel) = control.select_channel {
        if selected_channel > crate::config::CHANNELS_COUNT {
            defmt::error!("Protobuf: unknown channel: {}", selected_channel);
            return;
        }
        current_control_state.selected_channel = selected_channel;
        current_control_state.update();
    }
}
