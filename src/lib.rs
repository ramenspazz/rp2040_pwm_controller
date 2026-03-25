#![no_std]

use rp2040_hal::timer::Instant as RpInstant;   // rename to avoid conflict if needed
use core::convert::Infallible;
use embedded_hal::pwm::SetDutyCycle;
use heapless::Vec;

/// Fixed update interval for fading (in microseconds).
const UPDATE_INTERVAL_US: u32 = 1000;

/// Direction of the current fade.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DimmingDirection {
    Positive,
    Negative,
    None,
}

/// Internal state for a single PWM channel.
#[derive(Clone, Copy, Debug)]
pub struct PwmLedState {
    pub current_duty: u16,
    pub target_duty: u16,
    direction: DimmingDirection,
    step_size: u32,
    time_old: u64, // we store raw ticks (monotonic)
}

impl PwmLedState {
    pub fn new(initial_duty: u16, initial_ticks: u64) -> Self {
        Self {
            current_duty: initial_duty,
            target_duty: initial_duty,
            direction: DimmingDirection::None,
            step_size: 1,
            time_old: initial_ticks,
        }
    }

    pub fn set_target(&mut self, target: u16, duration_us: u32) {
        self.target_duty = target;

        let diff = target.abs_diff(self.current_duty) as u32;
        if diff == 0 {
            self.direction = DimmingDirection::None;
            return;
        }

        if duration_us == 0 {
            self.current_duty = target;
            self.direction = DimmingDirection::None;
            return;
        }

        self.direction = if target > self.current_duty {
            DimmingDirection::Positive
        } else {
            DimmingDirection::Negative
        };

        let num_updates = (duration_us / UPDATE_INTERVAL_US).max(1);
        self.step_size = (diff / num_updates).max(1);
    }

    pub fn update(&mut self, current_ticks: u64) -> bool {
        let dt = current_ticks.saturating_sub(self.time_old);

        if dt >= UPDATE_INTERVAL_US as u64 && self.current_duty != self.target_duty {
            match self.direction {
                DimmingDirection::Positive => {
                    self.current_duty = self.current_duty.saturating_add(self.step_size as u16)
                    .min(self.target_duty);
                }
                DimmingDirection::Negative => {
                    self.current_duty = self.current_duty.saturating_sub(self.step_size as u16)
                    .max(self.target_duty);
                }
                DimmingDirection::None => {
                    self.current_duty = self.target_duty;
                }
            }

            if self.current_duty == self.target_duty {
                self.direction = DimmingDirection::None;
            }

            self.time_old = current_ticks;
            true
        } else {
            false
        }
    }
}

pub struct LedController<'a, const N: usize = 8> {
    // We store &mut dyn SetDutyCycle<Error = Infallible>
    // The 'a lifetime ensures the channels live at least as long as the controller.
    channels: heapless::Vec<(&'a mut dyn embedded_hal::pwm::SetDutyCycle<Error = core::convert::Infallible>, PwmLedState), N>,
}

impl<'a, const N: usize> LedController<'a, N> {
    pub fn new() -> Self {
        Self {
            channels: heapless::Vec::new(),
        }
    }

    /// Add a PWM channel. The channel must outlive the controller.
    pub fn add_channel<C>(
        &mut self,
        channel: &'a mut C,
        initial_duty: u16,
        start_instant: RpInstant,           // <-- changed
    ) -> Result<(), ()>
    where
    C: embedded_hal::pwm::SetDutyCycle<Error = core::convert::Infallible> + 'a,
    {
        let state = PwmLedState::new(initial_duty, start_instant.ticks());   // .ticks() gives the u64
        let dyn_channel: &'a mut dyn embedded_hal::pwm::SetDutyCycle<Error = core::convert::Infallible> = channel;

        self.channels.push((dyn_channel, state)).map_err(|_| ())
    }

    /// Call this regularly (e.g. every 1 ms) with current timer ticks.
    /// Returns `true` if any channel actually stepped.
    pub fn update(&mut self, current_instant: RpInstant) -> bool {
        let current_ticks = current_instant.ticks();

        let mut did_update = false;

        for (channel, state) in &mut self.channels {
            if state.update(current_ticks) {
                did_update = true;
            }
            let _ = channel.set_duty_cycle(state.current_duty);
        }
        did_update
    }

    // The rest of the methods stay almost the same:
    pub fn set_channel(&mut self, index: usize, target: u16, duration_us: u32) {
        if let Some((_, state)) = self.channels.get_mut(index) {
            state.set_target(target, duration_us);
        }
    }

    pub fn get_channel_duty(&self, index: usize) -> Option<u16> {
        self.channels.get(index).map(|(_, state)| state.current_duty)
    }

    pub fn fade_finished(&self, index: usize) -> Option<bool> {
        self.channels.get(index).map(|(_, state)| state.current_duty == state.target_duty)
    }

    pub fn turn_off_all(&mut self, current_instant: RpInstant) {
        let current_ticks = current_instant.ticks();
        for (channel, state) in &mut self.channels {
            state.set_target(0, 0);
            let _ = channel.set_duty_cycle(state.current_duty);
            let _ = state.update(current_ticks);
        }
    }
}
