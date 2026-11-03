#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("rust.hpp");

        type Trajectory;
        type InputParameter;
        type OutputParameter;
        type Ruckig;

        // Trajectory
        fn traj_new_direct(dofs: usize) -> UniquePtr<Trajectory>;
        fn traj_new(dofs: usize, waypoints: usize) -> UniquePtr<Trajectory>;
        fn traj_get_dofs(traj: &Trajectory) -> usize;
        fn traj_get_duration(traj: &Trajectory) -> f64;
        unsafe fn traj_at_time(traj: &Trajectory, time: f64, pos: *mut f64, vel: *mut f64, acc: *mut f64, len: usize);

        // InputParameter
        fn input_new(dofs: usize) -> UniquePtr<InputParameter>;
        fn input_get_control_interface(input: &InputParameter) -> i32;
        fn input_set_control_interface(input: Pin<&mut InputParameter>, v: i32);
        fn input_get_synchronization(input: &InputParameter) -> i32;
        fn input_set_synchronization(input: Pin<&mut InputParameter>, v: i32);
        unsafe fn input_set_current_position(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_current_position(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_current_velocity(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_current_velocity(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_current_acceleration(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_current_acceleration(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_target_position(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_target_position(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_target_velocity(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_target_velocity(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_target_acceleration(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_target_acceleration(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_max_velocity(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_max_velocity(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_max_acceleration(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_max_acceleration(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_max_jerk(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_max_jerk(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_max_position(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_max_position(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_min_position(input: Pin<&mut InputParameter>, data: *const f64, len: usize);
        fn input_get_min_position(input: &InputParameter) -> &CxxVector<f64>;
        unsafe fn input_set_intermediate_positions(input: Pin<&mut InputParameter>, data: *const f64, dofs: usize, count: usize);
        fn input_clear_intermediate_positions(input: Pin<&mut InputParameter>);
        unsafe fn input_get_intermediate_positions(input: &InputParameter, buf: *mut f64, capacity: usize) -> usize;

        // OutputParameter
        fn output_new_direct(dofs: usize) -> UniquePtr<OutputParameter>;
        fn output_new(dofs: usize, waypoints: usize) -> UniquePtr<OutputParameter>;
        fn output_get_trajectory(output: &OutputParameter) -> &Trajectory;
        fn output_get_new_position(output: &OutputParameter) -> &CxxVector<f64>;
        fn output_get_new_velocity(output: &OutputParameter) -> &CxxVector<f64>;
        fn output_get_new_acceleration(output: &OutputParameter) -> &CxxVector<f64>;
        fn output_get_time(output: &OutputParameter) -> f64;
        fn output_get_new_section(output: &OutputParameter) -> usize;
        fn output_get_new_calculation(output: &OutputParameter) -> bool;
        fn output_get_calculation_duration(output: &OutputParameter) -> f64;
        fn output_pass_to_input(output: &OutputParameter, input: Pin<&mut InputParameter>);

        // Ruckig
        fn ruckig_new_direct_and_offline(dofs: usize) -> UniquePtr<Ruckig>;
        fn ruckig_new_direct(dofs: usize, delta_time: f64) -> UniquePtr<Ruckig>;
        fn ruckig_new_offline(dofs: usize, waypoints: usize) -> UniquePtr<Ruckig>;
        fn ruckig_new(dofs: usize, delta_time: f64, waypoints: usize) -> UniquePtr<Ruckig>;
        fn ruckig_update(ruckig: Pin<&mut Ruckig>, input: &InputParameter, output: Pin<&mut OutputParameter>) -> i32;
        fn ruckig_update_trajectory(ruckig: Pin<&mut Ruckig>, trajectory: &Trajectory, output: Pin<&mut OutputParameter>) -> i32;
        fn ruckig_calculate(ruckig: Pin<&mut Ruckig>, input: &InputParameter, trajectory: Pin<&mut Trajectory>) -> i32;
        unsafe fn ruckig_calculate_waypoints(ruckig: Pin<&mut Ruckig>, input: &InputParameter, trajectory: Pin<&mut Trajectory>, velocities: *const f64, velocity_count: usize, accelerations: *const f64, acceleration_count: usize) -> i32;
        fn ruckig_optimize_waypoints(ruckig: Pin<&mut Ruckig>, input: &InputParameter, trajectory: Pin<&mut Trajectory>, max_iterations: usize) -> i32;
        fn ruckig_reset(ruckig: Pin<&mut Ruckig>);
    }
}

use cxx::UniquePtr;

pub struct Trajectory(UniquePtr<ffi::Trajectory>);

pub struct TrajectoryRef<'a> {
    inner: &'a ffi::Trajectory,
}

fn traj_at_time_impl(inner: &ffi::Trajectory, time: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let dofs = ffi::traj_get_dofs(inner);
    let mut pos = vec![0.0f64; dofs];
    let mut vel = vec![0.0f64; dofs];
    let mut acc = vec![0.0f64; dofs];
    unsafe {
        ffi::traj_at_time(inner, time, pos.as_mut_ptr(), vel.as_mut_ptr(), acc.as_mut_ptr(), dofs);
    }
    (pos, vel, acc)
}

impl Trajectory {
    pub fn new_direct(dofs: usize) -> Self {
        Self(ffi::traj_new_direct(dofs))
    }

    pub fn new(dofs: usize, waypoints: usize) -> Self {
        Self(ffi::traj_new(dofs, waypoints))
    }

    pub fn get_duration(&self) -> f64 {
        ffi::traj_get_duration(&self.0)
    }

    pub fn at_time(&self, time: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        traj_at_time_impl(&self.0, time)
    }
}

impl<'a> TrajectoryRef<'a> {
    pub fn get_duration(&self) -> f64 {
        ffi::traj_get_duration(self.inner)
    }

    pub fn at_time(&self, time: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        traj_at_time_impl(self.inner, time)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum ControlInterface {
    #[default]
    Position = 0,
    Velocity = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum Synchronization {
    #[default]
    Time = 0,
    TimeIfNecessary = 1,
    Phase = 2,
    None = 3,
}

#[repr(i32)]
#[derive(Debug, PartialEq, Eq)]
pub enum Result {
    Working = 0,
    Finished = 1,
    Error = -1,
    ErrorInvalidInput = -100,
    ErrorTrajectoryDuration = -101,
    ErrorPositionalLimits = -102,
    ErrorZeroLimits = -104,
    ErrorExecutionTimeCalculation = -110,
    ErrorSynchronizationCalculation = -111,
}

impl From<i32> for Result {
    fn from(v: i32) -> Self {
        match v {
            0    => Result::Working,
            1    => Result::Finished,
            -100 => Result::ErrorInvalidInput,
            -101 => Result::ErrorTrajectoryDuration,
            -102 => Result::ErrorPositionalLimits,
            -104 => Result::ErrorZeroLimits,
            -110 => Result::ErrorExecutionTimeCalculation,
            -111 => Result::ErrorSynchronizationCalculation,
            _    => Result::Error,
        }
    }
}

pub struct InputParameter {
    pub degrees_of_freedom: usize,
    pub control_interface: ControlInterface,
    pub synchronization: Synchronization,
    pub current_position: Vec<f64>,
    pub current_velocity: Vec<f64>,
    pub current_acceleration: Vec<f64>,
    pub target_position: Vec<f64>,
    pub target_velocity: Vec<f64>,
    pub target_acceleration: Vec<f64>,
    pub max_velocity: Vec<f64>,
    pub max_acceleration: Vec<f64>,
    pub max_jerk: Vec<f64>,
    pub max_position: Vec<f64>,
    pub min_position: Vec<f64>,
    pub intermediate_positions: Vec<Vec<f64>>,
    inner: UniquePtr<ffi::InputParameter>,
}

impl InputParameter {
    pub fn new(dofs: usize) -> Self {
        Self {
            degrees_of_freedom: dofs,
            control_interface: ControlInterface::Position,
            synchronization: Synchronization::Time,
            current_position: vec![0.0; dofs],
            current_velocity: vec![0.0; dofs],
            current_acceleration: vec![0.0; dofs],
            target_position: vec![0.0; dofs],
            target_velocity: vec![0.0; dofs],
            target_acceleration: vec![0.0; dofs],
            max_velocity: vec![0.0; dofs],
            max_acceleration: vec![f64::INFINITY; dofs],
            max_jerk: vec![f64::INFINITY; dofs],
            max_position: vec![f64::INFINITY; dofs],
            min_position: vec![f64::NEG_INFINITY; dofs],
            intermediate_positions: Vec::new(),
            inner: ffi::input_new(dofs),
        }
    }

    fn sync(&mut self) {
        self.control_interface = unsafe { std::mem::transmute(ffi::input_get_control_interface(&self.inner)) };
        self.synchronization   = unsafe { std::mem::transmute(ffi::input_get_synchronization(&self.inner)) };

        self.current_position     = ffi::input_get_current_position(&self.inner).iter().cloned().collect();
        self.current_velocity     = ffi::input_get_current_velocity(&self.inner).iter().cloned().collect();
        self.current_acceleration = ffi::input_get_current_acceleration(&self.inner).iter().cloned().collect();
        self.target_position      = ffi::input_get_target_position(&self.inner).iter().cloned().collect();
        self.target_velocity      = ffi::input_get_target_velocity(&self.inner).iter().cloned().collect();
        self.target_acceleration  = ffi::input_get_target_acceleration(&self.inner).iter().cloned().collect();
        self.max_velocity         = ffi::input_get_max_velocity(&self.inner).iter().cloned().collect();
        self.max_acceleration     = ffi::input_get_max_acceleration(&self.inner).iter().cloned().collect();
        self.max_jerk             = ffi::input_get_max_jerk(&self.inner).iter().cloned().collect();
        self.max_position         = ffi::input_get_max_position(&self.inner).iter().cloned().collect();
        self.min_position         = ffi::input_get_min_position(&self.inner).iter().cloned().collect();

        let cap = self.degrees_of_freedom * 256;
        let mut flat = vec![0.0f64; cap];
        let written = unsafe { ffi::input_get_intermediate_positions(&self.inner, flat.as_mut_ptr(), cap) };
        self.intermediate_positions = if self.degrees_of_freedom > 0 && written > 0 {
            flat[..written].chunks(self.degrees_of_freedom).map(|c| c.to_vec()).collect()
        } else {
            Vec::new()
        };
    }

    fn apply(&mut self) {
        ffi::input_set_control_interface(self.inner.pin_mut(), self.control_interface as i32);
        ffi::input_set_synchronization(self.inner.pin_mut(),   self.synchronization as i32);

        unsafe {
            ffi::input_set_current_position(self.inner.pin_mut(),     self.current_position.as_ptr(),     self.current_position.len());
            ffi::input_set_current_velocity(self.inner.pin_mut(),     self.current_velocity.as_ptr(),     self.current_velocity.len());
            ffi::input_set_current_acceleration(self.inner.pin_mut(), self.current_acceleration.as_ptr(), self.current_acceleration.len());
            ffi::input_set_target_position(self.inner.pin_mut(),      self.target_position.as_ptr(),      self.target_position.len());
            ffi::input_set_target_velocity(self.inner.pin_mut(),      self.target_velocity.as_ptr(),      self.target_velocity.len());
            ffi::input_set_target_acceleration(self.inner.pin_mut(),  self.target_acceleration.as_ptr(),  self.target_acceleration.len());
            ffi::input_set_max_velocity(self.inner.pin_mut(),         self.max_velocity.as_ptr(),         self.max_velocity.len());
            ffi::input_set_max_acceleration(self.inner.pin_mut(),     self.max_acceleration.as_ptr(),     self.max_acceleration.len());
            ffi::input_set_max_jerk(self.inner.pin_mut(),             self.max_jerk.as_ptr(),             self.max_jerk.len());
            ffi::input_set_max_position(self.inner.pin_mut(),         self.max_position.as_ptr(),         self.max_position.len());
            ffi::input_set_min_position(self.inner.pin_mut(),         self.min_position.as_ptr(),         self.min_position.len());
        }

        if self.intermediate_positions.is_empty() {
            ffi::input_clear_intermediate_positions(self.inner.pin_mut());
        } else {
            let dofs = self.intermediate_positions[0].len();
            let flat: Vec<f64> = self.intermediate_positions.iter().flat_map(|r| r.iter().cloned()).collect();
            unsafe { ffi::input_set_intermediate_positions(self.inner.pin_mut(), flat.as_ptr(), dofs, self.intermediate_positions.len()); }
        }
    }
}

pub struct OutputParameter {
    pub degrees_of_freedom: usize,
    pub new_position: Vec<f64>,
    pub new_velocity: Vec<f64>,
    pub new_acceleration: Vec<f64>,
    pub time: f64,
    pub new_section: usize,
    pub new_calculation: bool,
    pub calculation_duration: f64,
    inner: UniquePtr<ffi::OutputParameter>,
}

impl OutputParameter {
    pub fn new_direct(dofs: usize) -> Self {
        Self {
            degrees_of_freedom: dofs,
            new_position: vec![0.0; dofs],
            new_velocity: vec![0.0; dofs],
            new_acceleration: vec![0.0; dofs],
            time: 0.0,
            new_section: 0,
            new_calculation: false,
            calculation_duration: 0.0,
            inner: ffi::output_new_direct(dofs),
        }
    }

    pub fn new(dofs: usize, waypoints: usize) -> Self {
        Self {
            degrees_of_freedom: dofs,
            new_position: vec![0.0; dofs],
            new_velocity: vec![0.0; dofs],
            new_acceleration: vec![0.0; dofs],
            time: 0.0,
            new_section: 0,
            new_calculation: false,
            calculation_duration: 0.0,
            inner: ffi::output_new(dofs, waypoints),
        }
    }

    pub fn pass_to_input(&self, input: &mut InputParameter) {
        ffi::output_pass_to_input(&self.inner, input.inner.pin_mut());
        input.sync();
    }

    fn sync(&mut self) {
        self.new_position         = ffi::output_get_new_position(&self.inner).iter().cloned().collect();
        self.new_velocity         = ffi::output_get_new_velocity(&self.inner).iter().cloned().collect();
        self.new_acceleration     = ffi::output_get_new_acceleration(&self.inner).iter().cloned().collect();
        self.time                 = ffi::output_get_time(&self.inner);
        self.new_section          = ffi::output_get_new_section(&self.inner);
        self.new_calculation      = ffi::output_get_new_calculation(&self.inner);
        self.calculation_duration = ffi::output_get_calculation_duration(&self.inner);
    }

    pub fn trajectory(&self) -> TrajectoryRef<'_> {
        TrajectoryRef { inner: ffi::output_get_trajectory(&self.inner) }
    }
}

pub struct Ruckig(UniquePtr<ffi::Ruckig>);

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> InputParameter {
        let mut input = InputParameter::new(1);
        input.target_position = vec![3.0];
        input.intermediate_positions = vec![vec![1.0], vec![2.0]];
        input.max_velocity = vec![1.0];
        input.max_acceleration = vec![1.0];
        input.max_jerk = vec![1.0];
        input
    }

    #[test]
    fn local_stop_and_replay() {
        let mut otg = Ruckig::new_direct(1, 0.1);
        let mut input = input();
        let mut trajectory = Trajectory::new_direct(1);
        assert_eq!(otg.calculate_waypoints(&mut input, &mut trajectory), Result::Working);
        let (p, v, a) = trajectory.at_time(trajectory.get_duration() / 3.0);
        assert!((p[0] - 1.0).abs() < 1e-8);
        assert!(v[0].abs() < 1e-8 && a[0].abs() < 1e-8);
        let mut output = OutputParameter::new_direct(1);
        let mut result = Result::Working;
        for _ in 0..1000 {
            result = otg.update_trajectory(&trajectory, &mut output);
            if result != Result::Working { break; }
        }
        assert_eq!(result, Result::Finished);
        assert_eq!(output.time, trajectory.get_duration());
        assert!((output.new_position[0] - 3.0).abs() < 1e-8);
    }

    #[test]
    fn local_prescribed_and_malformed_states() {
        let mut otg = Ruckig::new_direct_and_offline(1);
        let mut input = input();
        let mut trajectory = Trajectory::new_direct(1);
        assert_eq!(otg.calculate_waypoints_with_states(&mut input, &mut trajectory,
            &[vec![0.5], vec![0.5]], &[vec![0.0], vec![0.0]]), Result::Working);
        assert!((trajectory.at_time(trajectory.get_duration()).0[0] - 3.0).abs() < 1e-8);
        assert_eq!(otg.calculate_waypoints_with_states(&mut input, &mut trajectory,
            &[vec![0.5]], &[]), Result::ErrorInvalidInput);
        let mut executor = Ruckig::new_direct(1, 0.1);
        let mut output = OutputParameter::new_direct(1);
        assert_eq!(executor.update_trajectory(&trajectory, &mut output), Result::ErrorInvalidInput);
        input.intermediate_positions[0].clear();
        assert_eq!(otg.calculate_waypoints(&mut input, &mut trajectory), Result::ErrorInvalidInput);
    }

    #[test]
    fn automatic_states_improve_stopping_baseline() {
        let mut otg = Ruckig::new_direct(1, 0.1);
        let mut input = input();
        let mut trajectory = Trajectory::new_direct(1);
        assert_eq!(otg.calculate_waypoints(&mut input, &mut trajectory), Result::Working);
        let baseline = trajectory.get_duration();
        assert_eq!(otg.optimize_waypoints(&mut input, &mut trajectory, 8), Result::Working);
        assert!(trajectory.get_duration() < baseline);
        assert!((trajectory.at_time(trajectory.get_duration()).0[0] - 3.0).abs() < 1e-8);
        assert_eq!(otg.optimize_waypoints(&mut input, &mut trajectory, 65), Result::ErrorInvalidInput);
        input.intermediate_positions[0].clear();
        assert_eq!(otg.optimize_waypoints(&mut input, &mut trajectory, 8), Result::ErrorInvalidInput);
    }
}

impl Ruckig {
    pub fn new_direct_and_offline(dofs: usize) -> Self {
        Self(ffi::ruckig_new_direct_and_offline(dofs))
    }

    pub fn new_direct(dofs: usize, delta_time: f64) -> Self {
        Self(ffi::ruckig_new_direct(dofs, delta_time))
    }

    pub fn new_offline(dofs: usize, waypoints: usize) -> Self {
        Self(ffi::ruckig_new_offline(dofs, waypoints))
    }

    pub fn new(dofs: usize, delta_time: f64, waypoints: usize) -> Self {
        Self(ffi::ruckig_new(dofs, delta_time, waypoints))
    }

    pub fn reset(&mut self) {
        ffi::ruckig_reset(self.0.pin_mut());
    }

    pub fn update(&mut self, input: &mut InputParameter, output: &mut OutputParameter) -> Result {
        input.apply();
        let raw = ffi::ruckig_update(self.0.pin_mut(), &input.inner, output.inner.pin_mut());
        output.sync();
        Result::from(raw)
    }

    pub fn calculate(&mut self, input: &mut InputParameter, trajectory: &mut Trajectory) -> Result {
        input.apply();
        let raw = ffi::ruckig_calculate(self.0.pin_mut(), &input.inner, trajectory.0.pin_mut());
        Result::from(raw)
    }

    pub fn update_trajectory(&mut self, trajectory: &Trajectory, output: &mut OutputParameter) -> Result {
        let raw = ffi::ruckig_update_trajectory(self.0.pin_mut(), &trajectory.0, output.inner.pin_mut());
        if raw >= 0 {
            output.sync();
        }
        Result::from(raw)
    }

    /// Calculate local segments with zero velocity and acceleration at intermediate positions.
    pub fn calculate_waypoints(&mut self, input: &mut InputParameter, trajectory: &mut Trajectory) -> Result {
        self.calculate_waypoints_with_states(input, trajectory, &[], &[])
    }

    /// Calculate local segments with supplied intermediate states; empty arrays default to zero.
    pub fn calculate_waypoints_with_states(&mut self, input: &mut InputParameter, trajectory: &mut Trajectory,
        velocities: &[Vec<f64>], accelerations: &[Vec<f64>]) -> Result {
        let dofs = input.degrees_of_freedom;
        let count = input.intermediate_positions.len();
        if dofs == 0 || input.intermediate_positions.iter().any(|row| row.len() != dofs)
            || (!velocities.is_empty() && velocities.len() != count)
            || (!accelerations.is_empty() && accelerations.len() != count)
            || velocities.iter().chain(accelerations.iter()).any(|row| row.len() != dofs) {
            let trajectory_dofs = ffi::traj_get_dofs(&trajectory.0);
            *trajectory = Trajectory::new_direct(trajectory_dofs);
            return Result::ErrorInvalidInput;
        }
        let v: Vec<f64> = velocities.iter().flatten().copied().collect();
        let a: Vec<f64> = accelerations.iter().flatten().copied().collect();
        input.apply();
        let raw = unsafe {
            ffi::ruckig_calculate_waypoints(self.0.pin_mut(), &input.inner, trajectory.0.pin_mut(),
                v.as_ptr(), v.len(), a.as_ptr(), a.len())
        };
        Result::from(raw)
    }

    /// Automatically choose intermediate states using a bounded offline search (0..=64 iterations).
    pub fn optimize_waypoints(&mut self, input: &mut InputParameter, trajectory: &mut Trajectory, max_iterations: usize) -> Result {
        if input.degrees_of_freedom == 0 || input.intermediate_positions.iter().any(|row| row.len() != input.degrees_of_freedom) {
            let dofs = ffi::traj_get_dofs(&trajectory.0);
            *trajectory = Trajectory::new_direct(dofs);
            return Result::ErrorInvalidInput;
        }
        input.apply();
        Result::from(ffi::ruckig_optimize_waypoints(self.0.pin_mut(), &input.inner, trajectory.0.pin_mut(), max_iterations))
    }
}
