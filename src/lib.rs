#![no_std]

use core::convert::Infallible;
use embedded_hal::delay::DelayNs;
use rp2040_hal::{
    self as hal,
    clocks::Clock,
    pwm::Channel,
    timer::Timer,
};
use embedded_hal::pwm::SetDutyCycle;
use fugit::{Duration, Instant};

// Specify a 32-bit Instant with a tick rate of 1 million ticks per second
type MilisecondInstant = Instant<u32, 1, 1_000>;  // milisecond clock type

// global constants
const UPDATE_INTERVAL_US: u32 = 1000;
const MICROSECOND_PER_SECOND: f32 = 1E+06;  // microseconds in 1 second


#[derive(Clone, Copy, PartialEq)]
pub enum DimmingDirection {
    Positive,
    Negative,
    None,
}

pub struct PwmLedState {
    pub current_duty: u16,
    pub target_duty: u16,
    pub direction: DimmingDirection,
    pub steps_per_ms: u32,
    pub step_size: u32,
    time_old: hal::timer::Instant,
    time_new: hal::timer::Instant,
}

impl PwmLedState {
    pub fn new(initial_duty: u16, initial_time: hal::timer::Instant) -> Self {
        Self {
            current_duty: initial_duty,
            target_duty: initial_duty,
            direction: DimmingDirection::None,
            steps_per_ms: 1,
            step_size: 1,
            time_old: initial_time,
            time_new: initial_time,
        }
    }

    /// This function handles the logic for updating a target duty on a channel
    /// by setting fade direction, timing, and steps for fading
    pub fn set_target(&mut self, target: u16, duration_us: u32) {
        self.target_duty = target;

        let diff: u32 = target.abs_diff(self.current_duty) as u32;
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

    /// This is the magic function that does it all!
    /// This takes the value of target duty and current duty from all added channels
    /// and then manages the evolution of the state of the channel based on the elapsed
    /// time between calls of the update function, and if the change is instant or a fade
    pub fn update(&mut self, current_time: hal::timer::Instant) -> bool {
        self.time_new = current_time;

        if let Some(elapsed_us) = self.time_new.checked_duration_since(self.time_old) {
            let dt = elapsed_us.ticks() as u32;

            if dt >= UPDATE_INTERVAL_US && self.current_duty != self.target_duty {
                // We are going to perform a step → this is the "updated" case
                match self.direction {
                    DimmingDirection::Positive => {
                        self.current_duty = (self.current_duty + self.step_size as u16).min(self.target_duty);
                    }
                    DimmingDirection::Negative => {
                        self.current_duty = self.current_duty.saturating_sub(self.step_size as u16).max(self.target_duty);
                    }
                    DimmingDirection::None => {
                        self.current_duty = self.target_duty;
                    }
                }

                let finished = self.current_duty == self.target_duty;
                if finished {
                    self.direction = DimmingDirection::None;
                }

                self.time_old = self.time_new;

                // Return true whether we just finished or just took a step
                true
            } else {
                // Time hasn't passed enough OR we're already at target
                false
            }
        } else {
            // Clock went backwards or overflowed — very rare on rp2040 timer
            false
        }
    }
}

/// Trait to abstract over any PWM channel
pub trait PwmChannelControl {
    fn set_duty(&mut self, duty: u16) -> Result<(), core::convert::Infallible>;
}

// Implement for any Channel type
impl<S, C> PwmChannelControl for Channel<S, C>
where
    S: hal::pwm::AnySlice,
    C: hal::pwm::ChannelId,
    Channel<S, C>: SetDutyCycle<Error = Infallible>,
{
    fn set_duty(&mut self, duty: u16) -> Result<(), core::convert::Infallible> {
        self.set_duty_cycle(duty)
    }
}

pub struct LedController<'a> {
    channels: heapless::Vec<(&'a mut dyn PwmChannelControl, PwmLedState), 8>,
    stopwatch: hal::timer::Timer,
}

impl<'a> LedController<'a> {
    pub fn new(passed_timer: hal::timer::Timer) -> Self {
        Self {
            channels: heapless::Vec::new(),
            stopwatch: passed_timer,
        }
    }

    pub fn add_channel<S, C>(&mut self, channel: &'a mut Channel<S, C>, initial_duty: u16, start_time: hal::timer::Instant) -> Result<(), ()>
    where
        S: hal::pwm::AnySlice,
        C: hal::pwm::ChannelId,
        Channel<S, C>: SetDutyCycle<Error = Infallible>,
    {
        let state = PwmLedState::new(initial_duty, start_time);
        self.channels.push((channel as &mut dyn PwmChannelControl, state)).map_err(|_| ())
    }

    pub fn update(&mut self, current_time: hal::timer::Instant) -> bool {
        let mut return_state = false;

        for (channel, state) in self.channels.iter_mut() {
            let return_state = state.update(current_time) || return_state;
            let _ = channel.set_duty(state.current_duty);
        }
        return_state
    }

    pub fn get_channel_state(&self, index: usize) -> Option<u16> {
        let length = self.channels.len();
        // vector returns a valid value for len
        if index >= length {
            // valid value returned but the index is invalid
            None
        }
        else {
            // valid value is returned and the index is valid
            Some(self.channels[index].1.current_duty)
        }
    }

    pub fn turn_off_all_channels(&mut self, current_time: hal::timer::Instant) {
        for (channel, state) in self.channels.iter_mut() {
            let _ = state.set_target(0, 0);
            let _ = channel.set_duty(state.current_duty);
            let _ = state.update(current_time);
        }
    }
    
    pub fn fade_finished(&self, index: usize) -> Option<bool> {
        // returns none if the requested index is out of bounds
        // else it will check if target duty is equal to current duty
        // and return true if they are equal, else false
        if self.channels.len() >= index {
            // the index is valid
            return {
                if self.channels[index].1.current_duty == self.channels[index].1.target_duty {
                    Some(true)
                } else {
                    Some(false)
                }
            }
        }
        else {
            // index was out of range
            return None
        }
    }

    /// sets the `target_duty` of the `PwmLedState` object at `index` `to target`, with
    /// option for a durration to make the change from current initial duty when `set_channel` is
    /// called to `target_duty`, in `duration_ms` miliseconds, else instant if None or 0.
    pub fn set_channel(&mut self, index: usize, target: u16, duration_us: u32) {
        self.channels[index].1.set_target(target, duration_us);
    }
}
