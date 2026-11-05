<div align="center">
  <h1 align="center">Ruckig</h1>
  <h3 align="center">
    Instantaneous Motion Generation for Robots and Machines.
  </h3>
</div>
<p align="center">
  <a href="https://github.com/pantor/ruckig/actions">
    <img src="https://github.com/pantor/ruckig/workflows/CI/badge.svg" alt="CI">
  </a>
  <a href="https://github.com/pantor/ruckig/issues">
    <img src="https://img.shields.io/github/issues/pantor/ruckig.svg" alt="Issues">
  </a>
  <a href="https://github.com/pantor/ruckig/releases">
    <img src="https://img.shields.io/github/v/release/pantor/ruckig.svg?include_prereleases&sort=semver" alt="Releases">
  </a>
  <a href="https://github.com/pantor/ruckig/blob/main/LICENSE">
    <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="MIT">
  </a>
</p>

Ruckig generates trajectories on-the-fly, allowing robots and machines to react instantaneously to sensor input. Ruckig calculates a trajectory to a *target* waypoint (with position, velocity, and acceleration) starting from *any* initial state limited by velocity, acceleration, and jerk constraints. Besides the target state, Ruckig allows to define intermediate positions for waypoint following. For state-to-state motions, Ruckig guarantees a time-optimal solution. With intermediate waypoints, Ruckig calculates the path and its time parametrization jointly, resulting in significantly faster trajectories compared to traditional methods.

More information can be found at [ruckig.com](https://ruckig.com) and in the corresponding paper [Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830). Documentation is published at [docs.ruckig.com](https://docs.ruckig.com).


## Installation

Ruckig has no dependencies (except for testing). To build Ruckig using CMake, just run

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

To install Ruckig in a system-wide directory, you can either use `(sudo) make install` or install it as debian package using cpack by running

```bash
cpack
sudo dpkg -i ruckig*.deb
```

An example of using Ruckig in your CMake project is given by `examples/CMakeLists.txt`. However, you can also include Ruckig as a directory within your project and call `add_subdirectory(ruckig)` in your parent `CMakeLists.txt`.

Ruckig is also available as a Python module, in particular for development or debugging purposes. The Ruckig *Community Version* can be installed from [PyPI](https://pypi.org/project/ruckig/) via
```bash
pip install ruckig
```
When using CMake, the Python module can be built using the `BUILD_PYTHON_MODULE` flag. If you're only interested in the Python module (and not in the C++ library), you can build and install Ruckig via `pip install .`.


## Tutorial

Furthermore, we will explain the basics to get started with online generated trajectories within your application. There is also a [collection of examples](https://docs.ruckig.com/examples.html) that guide you through the most important features of Ruckig. A time-optimal trajectory for a single degree of freedom is shown in the figure below. The [online S-curve calculator](https://ruckig.com/s-curve-trajectory.html) computes and plots a trajectory live in your browser, for any start and target state.

![S-curve Trajectory Profile](https://github.com/pantor/ruckig/raw/main/doc/example_profile.png?raw=true)

### Waypoint-based Trajectory Generation

Ruckig provides three main interface classes: the *Ruckig*, the *InputParameter*, and the *OutputParameter* class.

First, you'll need to create a Ruckig instance with the number of DoFs and the control cycle in the constructor. Ruckig itself is unitless - we recommend e.g. meters or radians for position and seconds for time.

```.cpp
Ruckig ruckig {6, 0.001}; // Degrees of freedom; control cycle
```

The input type has 3 blocks of data: the *current* state, the *target* state and the corresponding kinematic *limits*.

```.cpp
InputParameter input {6}; // Degrees of freedom
input.current_position = {0.2, ...};
input.current_velocity = {0.1, ...};
input.current_acceleration = {0.1, ...};
input.target_position = {0.5, ...};
input.target_velocity = {-0.1, ...};
input.target_acceleration = {0.2, ...};
input.max_velocity = {0.4, ...};
input.max_acceleration = {1.0, ...};
input.max_jerk = {4.0, ...};

OutputParameter output {6}; // Degrees of freedom
```

If you only want to have an acceleration-constrained trajectory, you can also omit the `max_jerk` as well as the `current` and `target_acceleration` value. Given all input and output resources, we can iterate over the trajectory at each discrete time step. For most applications, this loop must run within a real-time thread and controls the actual hardware.

```.cpp
while (ruckig.update(input, output) == Result::Working) {
  // Make use of the new state here!
  // e.g. robot->setJointPositions(output.new_position);

  output.pass_to_input(input); // Don't forget this!
}
```

Within the control loop, you need to update the *current state* of the input parameter according to the calculated trajectory. Therefore, the `pass_to_input` method copies the new kinematic state of the output to the current kinematic state of the input parameter. If (in the next step) the current state is not the expected, pre-calculated trajectory, Ruckig will calculate a new trajectory based on the novel input. When the trajectory has reached the target state, the `update` function will return `Result::Finished`.


### Intermediate Waypoints

The Ruckig Community Version includes built-in support for intermediate waypoints, using our cloud API for remote calculation. Of course, the Ruckig Pro version is fully local. To allocate the necessary memory for a variable number of waypoints beforehand, we need to pass the maximum number of waypoints to Ruckig via
```.cpp
Ruckig ruckig {6, 0.001, 8};
InputParameter input {6, 8};
OutputParameter output {6, 8};
```
The `InputParameter` class takes the number of waypoints as an optional input, however usually you will fill in the values (and therefore reserve its memory) yourself. Then you're ready to set intermediate waypoints by
```.cpp
input.intermediate_positions = {
  {0.2, ...},
  {0.8, ...},
};
```
As soon as intermediate positions are given, the Ruckig Community Version switches to the mentioned (of course, non real-time capable) cloud API. If you require real-time calculation on your own hardware, please contact us for the *Ruckig Pro Version*.

When using *intermediate positions*, both the underlying motion planning problem as well as its calculation changes significantly. In particular, there are some fundamental limitations for jerk-limited online trajectory generation regarding the usage of waypoints. Please find more information about these limitations [here](https://docs.ruckig.com/md_pages_2__intermediate__waypoints.html), and in general we recommend to use
```.cpp
input.intermediate_positions = ruckig.filter_intermediate_positions(input, {0.1, ...});
```
to filter waypoints according to a (high) threshold distance. Setting *interrupt_calculation_duration* makes sure to be real-time capable by refining the solution in the next control invocation. Note that this is a soft interruption of the calculation. Currently, no minimum or discrete durations are supported when using intermediate positions.


### Fully Local Multi-Waypoint Calculation

This fork provides an explicit offline `calculate_waypoints` API, available with `BUILD_CLOUD_CLIENT=OFF` or `ON`. It never calls the cloud service. Set the intermediate positions as usual, then calculate into an ordinary `Trajectory`:

```python
otg = Ruckig(2, 0.001)
inp = InputParameter(2)
inp.current_position = [0.0, 0.0]
inp.target_position = [3.0, 1.5]
inp.max_velocity = [2.0, 2.0]
inp.max_acceleration = [2.0, 2.0]
inp.max_jerk = [4.0, 4.0]
inp.intermediate_positions = [[1.0, 0.5], [2.0, 1.0]]
out = OutputParameter(2)
result = otg.calculate_waypoints(inp, out.trajectory)
```

By default, velocity and acceleration are zero at every intermediate point. To prescribe pass-through states, supply matching arrays:

```python
result = otg.calculate_waypoints(
    inp, out.trajectory,
    intermediate_velocities=[[0.5, 0.25], [0.5, 0.25]],
    intermediate_accelerations=[[0.0, 0.0], [0.0, 0.0]],
)
```

C++ uses the same method and arguments. Rust provides `calculate_waypoints` and `calculate_waypoints_with_states`. Each omitted state array defaults to zero; non-empty arrays must have exactly one vector per intermediate point and one value per DoF. Invalid or infeasible specified states return an error rather than being replaced with stops. Consecutive identical position/velocity/acceleration states share the same time, including when the velocity or acceleration is nonzero. Equal positions with different velocities or accelerations still require a nonzero transition.

To automatically select intermediate velocities and accelerations from just the position list, use:

```python
result = otg.optimize_waypoints(inp, out.trajectory, max_iterations=8)
```

C++ provides the same method, and Rust uses `optimize_waypoints(&mut input, &mut trajectory, 8)`. The optimizer first tries states sampled from endpoint trajectories, checking that every waypoint is visited in order on all axes and re-solving each segment under the original limits. This preserves cruise speed on densely sampled straight routes; refinement stops early when a feasible chain reaches the unconstrained endpoint duration within numerical tolerance. For curved routes, additional seeds use the full path's chord lengths, tangents and curvature to choose a common timing. Consecutive identical intermediate points share one optimized state and retain separate section entries at the same time. It also evaluates stopping, geometry-based, and endpoint-motion seeds, then performs bounded local refinement. Each trial validates arrival/departure feasibility, solves the two affected adjacent segments, and checks the existing kinematic and global position limits. Only a strictly shorter feasible trial replaces the current solution. When the stopping baseline is feasible, the returned trajectory is never longer. Endpoint-motion seeds can also find feasible chains in some cases where stopping at the intermediate points is infeasible.

`max_iterations` defaults to 8, is limited to 64, and controls the refinement sweeps per seed. When the curved-path seed improves the initial solution, both it and the original seed are refined and the shorter result is retained. This can require about twice the refinement work. Zero requests only the stopping baseline. Failed trial states are handled internally, including in Python; if no seed is feasible, the method returns an error and the output trajectory remains invalid. The bounded heuristic search can fail on a feasible problem and does not prove infeasibility. This is an offline operation that allocates memory; refinement solves a bounded number of segment pairs per waypoint and DoF in each sweep.

Automatic states may still reach zero velocity at reversals or repeated coordinates. Local refinement restricts monotone coordinates to velocities in the direction of both neighboring displacements; reversing coordinates use zero velocity at the turning point, with acceleration selected by the search. Inspect the selected states with `trajectory.at_time(t)` at the intermediate durations. Use `calculate_waypoints` to retain explicitly prescribed states; `optimize_waypoints` chooses them afresh. Run `python test/benchmark_waypoints.py` with the built Python module to compare stopping/automatic durations and planning times on fixed straight, curved, and reversing routes.

`filter_intermediate_positions(input, threshold_distance)` is also fully local in C++ and Python, with either cloud configuration. Thresholds must contain one finite nonnegative distance per DoF. Filtering checks every axis, including coordinates that are equal at the candidate segment's endpoints, and retains points that exceed the specified deviation.

After checking the result, replay using `otg.update(out.trajectory, out)`. This form avoids copying the complete chain on every cycle. In Rust, load the trajectory and advance the first cycle with `update_trajectory(&trajectory, &mut output)`, then use `update_current_trajectory(&mut output)` for subsequent cycles to reuse the stored chain and state buffers. No `pass_to_input` call is required for offline replay. For each new replay, calculate into a fresh `OutputParameter` (or reset its `time` and `new_section` first in C++). Examples are provided in `examples/17_local_waypoints.cpp`, `.py`, and `.rs`.

The result supports ordinary time sampling, batch sampling, section indices, position extrema, and position-crossing queries on the complete timeline. Sampling requires a successfully calculated trajectory and finite times; C++/Python reject invalid sampling with `RuckigError`, and Rust `at_time` panics. Finite times outside the timeline retain the existing extrapolation behavior; offline replay clamps to the terminal state. Reading profiles/extrema before a complete calculation also raises `RuckigError`; metadata from a state-to-state calculation rejected only by position bounds remains available for diagnosis. `intermediate_durations` includes every section end and the final duration; `independent_min_durations` sums the per-axis minimum durations of all sections. Metadata getters return `std::vector` in both cloud configurations. The fixed single-section storage is retained when the cloud client is disabled; extra section storage is allocated only for local chains.

This initial implementation requires position control, all axes enabled, finite jerk limits, and global time or phase synchronization. Global position bounds are checked on every segment. Global minimum/discrete duration, per-section limits/durations, per-axis control/synchronization overrides, and interruption are unsupported and are rejected. Computation allocates memory and is intended to run outside the real-time loop.

`calculate_waypoints` optimizes each segment for the caller's fixed states (or zero by default); `optimize_waypoints` additionally searches the intermediate states to reduce total duration. Neither API guarantees a globally shortest multi-waypoint trajectory. Position, velocity, and acceleration match at joins within the existing solver tolerances; jerk may jump within its limits. Passing through joint-space points does not guarantee following the original geometric path or avoiding obstacles. The ordinary `calculate`/online-input API retains its existing cloud routing for intermediate positions.

For explicit local chains, `pass_to_input` also removes every waypoint reached by the output time, including when one cycle crosses several sections. Repeating the call with the same output keeps the remaining list unchanged. Apply it before editing the next target or waypoint list. Before replanning, trim any caller-owned waypoint velocity/acceleration arrays to match the remaining points. Ordinary cloud outputs retain their existing waypoint-removal behavior.

### Input Parameter

To go into more detail, the *InputParameter* type has following members:

```.cpp
using Vector = std::vector<double>;

Vector current_position;
Vector current_velocity; // Initialized to zero
Vector current_acceleration; // Initialized to zero

std::vector<Vector> intermediate_positions; // (via cloud API, fully local in Pro Version)

Vector target_position;
Vector target_velocity; // Initialized to zero
Vector target_acceleration; // Initialized to zero

Vector max_velocity;
Vector max_acceleration;
Vector max_jerk; // Initialized to infinity

Vector max_position; // Initialized to infinity; checked over the calculated trajectory
Vector min_position; // Initialized to -infinity; checked over the calculated trajectory

std::optional<Vector> min_velocity; // If not given, the negative maximum velocity will be used.
std::optional<Vector> min_acceleration; // If not given, the negative maximum acceleration will be used.

std::array<bool, DOFs> enabled; // Initialized to true
std::optional<double> minimum_duration;
std::optional<double> interrupt_calculation_duration; // [µs], (only in Pro Version)

ControlInterface control_interface; // The default position interface controls the full kinematic state.
Synchronization synchronization; // Synchronization behavior of multiple DoFs
DurationDiscretization duration_discretization; // Whether the duration should be a discrete multiple of the control cycle (off by default)

std::optional<Vector<ControlInterface>> per_dof_control_interface; // Sets the control interface for each DoF individually, overwrites global control_interface
std::optional<Vector<Synchronization>> per_dof_synchronization; // Sets the synchronization for each DoF individually, overwrites global synchronization
```

On top of the current state, target state, and constraints, Ruckig allows for a few more advanced settings:
- A *minimum* velocity and acceleration can be specified - these should be a negative number. If they are not given, the negative maximum velocity or acceleration will be used (similar to the jerk limit). For example, this might be useful in human robot collaboration settings with a different velocity limit towards a human. Or, when switching between different moving coordinate frames like picking from a conveyer belt.
- You can overwrite the global kinematic limits to specify limits for each section between two waypoints separately by using e.g. `per_section_max_velocity`.
- If a DoF is not *enabled*, it will be ignored in the calculation. Ruckig will output a trajectory with constant acceleration for those DoFs.
- A *minimum duration* can be optionally given. Note that Ruckig can not guarantee an exact, but only a minimum duration of the trajectory.
- The control interface (position or velocity control) can be switched easily. For example, a stop trajectory or visual servoing can be easily implemented with the velocity interface.
- Different synchronization behaviors (i.a. phase, time, or no synchronization) are implemented. Phase synchronization results in straight-line motions.
- The trajectory duration might be constrained to a multiple of the control cycle. This way, the *exact* state can be reached at a control loop execution.

We refer to the [API documentation](https://docs.ruckig.com/namespaceruckig.html) of the enumerations within the `ruckig` namespace for all available options.


### Input Validation

To check that Ruckig is able to generate a trajectory before the actual calculation step,
```.cpp
ruckig.validate_input(input, check_current_state_within_limits=false, check_target_state_within_limits=true);
// returns true or throws
```
throws an error with a detailed reason if an input is not valid. You can also set the default template parameter to false via `ruckig.validate_input<false>(...)` to just return a boolean true or false. The two boolean arguments check that the current or target state are within the limits. The check includes a typical catch of jerk-limited trajectory generation: When the current state is at maximal velocity, any positive acceleration will inevitably lead to a velocity violation *at a future timestep*. In general, this condition is fulfilled when
```
Abs(acceleration) <= Sqrt(2 * max_jerk * (max_velocity - Abs(velocity))).
```
If both arguments are set to true, the calculated trajectory is guaranteed to be *within the kinematic limits throughout* its duration. Also, note that there are range constraints of the input due to numerical reasons, see below for more details.


### Result Type

The `update` function of the Ruckig class returns a Result type that indicates the current state of the algorithm. This can either be **working**, **finished** if the trajectory has finished, or an **error** type if something went wrong during calculation. The result type can be compared as a standard integer.

State                           | Error Code
------------------------------- | ----------
Working                         | 0
Finished                        | 1
Error                           | -1
ErrorInvalidInput               | -100
ErrorTrajectoryDuration         | -101
ErrorPositionalLimits           | -102
ErrorExecutionTimeCalculation   | -110
ErrorSynchronizationCalculation | -111

With the default C++ `Ruckig` configuration, cloud connection failures, non-200 HTTP responses, and JSON parsing failures return `Result::Error`. `RuckigThrow` and Python continue to report these failures with exceptions.


### Output Parameter

The output class includes the new kinematic state and the overall trajectory.

```.cpp
Vector new_position;
Vector new_velocity;
Vector new_acceleration;

Trajectory trajectory; // The current trajectory
double time; // The current, auto-incremented time. Reset to 0 at a new calculation.

size_t new_section; // Index of the section between two (possibly filtered) intermediate positions.
bool did_section_change; // Was a new section reached in the last cycle?

bool new_calculation; // Whether a new calculation was performed in the last cycle
bool was_calculation_interrupted; // Was the trajectory calculation interrupted? (only in Pro Version)
double calculation_duration; // Duration of the calculation in the last cycle [µs]
```
Moreover, the **trajectory** class has a range of useful parameters and methods.

```.cpp
double duration; // Duration of the trajectory
std::array<double, DOFs> independent_min_durations; // Time-optimal profile for each independent DoF

<...> at_time(double time); // Get the kinematic state of the trajectory at a given time
void get_position_extrema(Vector<Bound>& position_extrema); // Passes information about the position extrema and their times
```
Independent minimum durations are available for state-to-state and explicit local waypoint calculations. The cloud path does not populate this diagnostic; its C++/Python getter raises `RuckigError` for a cloud trajectory. Use local calculation when these values are required.

Again, we refer to the [API documentation](https://docs.ruckig.com) for the exact signatures.


### Offline Calculation

Ruckig also supports an offline approach for calculating a trajectory:
```.cpp
result = ruckig.calculate(input, trajectory);
```
When only using this method, the `Ruckig` constructor does not need a control cycle (`delta_time`) as an argument. However if given, Ruckig supports stepping through the trajectory with
```.cpp
Result step_result;
do {
  step_result = ruckig.update(trajectory, output);
  if (step_result < 0) {
    break; // Handle the error before using the output.
  }
  // Use output.new_position, including the terminal state on Finished.
} while (step_result == Result::Working);
```
starting from the current `output.time`. Use a positive, finite control cycle for stepping. Each call advances by one cycle and clamps to the terminal state, returning `Finished` on reaching the duration. Repeated calls then keep the terminal state, including its velocity and acceleration, without extrapolating past the end. The output includes the supplied trajectory, and `new_calculation` stays false. C++ and Python use `update(trajectory, output)`; Rust uses `update_trajectory(&trajectory, &mut output)`.

### Local Position Bounds

The Community version checks `min_position` and `max_position` locally after calculating a trajectory. The check uses analytical position extrema over the complete interval `[0, duration]`, including braking, constant-acceleration segments, and unsynchronized or disabled axes that continue moving until the overall duration. It applies to both position and velocity control, with an absolute tolerance of `1e-8` in position units.

If the calculated motion crosses a bound, `calculate` or online `update` returns `Result::ErrorPositionalLimits`. Online update emits no new state for the rejected motion; it retries planning on the next call. A rejected trajectory cannot be used with offline `update`. Both online and offline stepping stop at the terminal time so a final nonzero velocity cannot cause an extra control-cycle extrapolation.

This is a feasibility check of the generated motion: it does not search for a different, slower trajectory that avoids an overshoot. Callers must handle a negative result before using the output. Position bounds default to infinity, so existing unconstrained inputs retain their motion profiles. The waypoint optimizer can explore different intermediate states, but does not replace the underlying single-segment positional feasibility check with a position-constrained solver. Per-section position bounds, predictive tracking, speed control, and interruptible optimization remain outside this implementation. This fork implements the local features using the existing MIT-licensed solver and analytical integration; it contains no Ruckig Pro source code.


### Tracking Interface

When following an arbitrary signal with position, velocity, acceleration, and jerk-limitation, the straightforward way would be to pass the current state to Ruckig's target state. However, as the resulting trajectory will take time to catch up, this approach will always lag behind the signal. The tracking interface solves this problem by predicting ahead (e.g. with constant acceleration by default) and is therefore able to follow signals very closely in a time-optimal way. This might be very helpful for (general) tracking, robot servoing, or trajectory post-processing applications.

To use the tracking interface, construct
```.cpp
Trackig trackig {1, 0.01};  // Degrees of freedom; control cycle
```
and set the current state as well as the kinematic constraints via
```.cpp
input.current_position = {0.0};
input.current_velocity = {0.0};
input.current_acceleration = {0.0};
input.max_velocity = {0.8};
input.max_acceleration = {2.0};
input.max_jerk = {5.0};
```
Then, we can track a signal in an online manner within the real-time control loop
```.cpp
for (double t = 0; t < 10.0; t += trackig.delta_time) {
  auto target_state = signal(t); // signal returns position, velocity, and acceleration
  auto res = trackig.update(target_state, input, output);
  // Make use of the smooth target motion here (e.g. output.new_position)

  output.pass_to_input(input);
}
```
Please find a complete example [here](https://github.com/pantor/ruckig/blob/main/examples/13_tracking.cpp). This functionality can also be used in an offline manner, e.g. when the entire signal is known beforehand. Here, we call the
```.cpp
smooth_trajectory = trackig.calculate_trajectory(target_states, input);
```
method with the trajectory given as a `std::vector` of target states. The Tracking interface is available in the Ruckig Pro version.


## Tests and Numerical Stability

The current test suite validates over 5.000.000.000 random trajectories as well as many additional edge cases. The numerical exactness is tested for the final position and final velocity to be within `1e-8`, for the final acceleration to be within `1e-10`, and for the velocity, acceleration and jerk limit to be within a numerical error of `1e-12`. These are absolute values - we suggest to scale your input so that these correspond to your required precision of the system. For example, for most real-world systems we suggest to use input values in `[m]` (instead of e.g. `[mm]`), as `1e-8m` is sufficiently precise for practical trajectory generation. Furthermore, all kinematic limits should be below `1e9`. The maximal supported trajectory duration is `7e3`. Note that Ruckig will also output values outside of this range, there is however no guarantee for correctness.

The Ruckig Pro version has additional tools to increase the numerical range and improve reliability. For example, the `position_scale` and `time_scale` parameter of the `Calculator` class change the internal representation of the input parameters.
```.cpp
Ruckig ruckig {1};  // Works also for Trackig

ruckig.calculator.position_scale = 1e2;  // Scales all positions in the input parameters
ruckig.calculator.time_scale = 1e3;  // Scale all times in the input parameters
```
This way, you can easily achieve the requirements above even for very high jerk limits or very long trajectories. Note that the scale parameters don't affect the resulting trajectory - they are for internal calculation only.


## Benchmark

We report [benchmarking results in our docs](https://docs.ruckig.com/benchmark.html). To the best of our knowledge, the Ruckig Community version is the fastest trajectory or s-curve generator for state-to-state motions and is well-suited for control cycles down to 500 microseconds (2000 Hz). The Pro version is around twice as fast, making it even better suited for short control cycles or less powerful hardware.


## Development

Ruckig is written in C++20. It is continuously tested on `ubuntu-latest`, `macos-latest`, and `windows-latest` against following versions:

- Doctest v2.4 (only for testing)
- Nanobind v2.7 (only for Python wrapper)

A C++17, C++11, and C++03 version of Ruckig is also available - please contact us if you're interested.


## Used By

Ruckig is used by over a hundred research labs, companies, and open-source projects worldwide, including:
- [MoveIt 2](https://moveit.ros.org) for trajectory generation.
- [CoppeliaSim](https://www.coppeliarobotics.com/) starting from version 4.3.
- [Fuzzy Logic Robotics](https://flr.io)
- [LinuxCNC](https://linuxcnc.org)
- [Scanlab](https://www.scanlab.de/de) for controlling lasers.
- [Ocado](https://ocadointelligentautomation.com)
- [Sunday Robotics](https://www.sunday.ai)
- [Arburg](https://www.arburg.com)
- [Gestalt Robotics](https://www.gestalt-robotics.com)
- [AgiBot Genie Sim](https://github.com/AgibotTech/genie_sim)
- [Struckig](https://github.com/stefanbesler/struckig), a port of Ruckig to Structured Text (ST - IEC61131-3) for usage on PLCs.
- [Ruckig-Scl](https://github.com/core-engineering/ruckig-scl), a port of Ruckig to SCL (ST - IEC61131-3) for Siemens PLCs
- [Frankx](https://github.com/pantor/frankx) for controlling the Franka Emika robot arm.
- [TidyBot++](https://github.com/jimmyyhwu/tidybot2)
- [Wiredworks](https://wiredworks.com) made a simple Kivy [GUI application](https://github.com/wiredworks/ruckig-showcase)
- and many others!


## Citation

```
@article{berscheid2021jerk,
  title={Jerk-limited Real-time Trajectory Generation with Arbitrary Target States},
  author={Berscheid, Lars and Kr{\"o}ger, Torsten},
  journal={Robotics: Science and Systems XVII},
  year={2021}
}
```
