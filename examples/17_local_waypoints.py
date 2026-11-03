from ruckig import InputParameter, OutputParameter, Result, Ruckig


otg = Ruckig(2, 0.01)
inp = InputParameter(2)
inp.current_position = [0.0, 0.0]
inp.target_position = [3.0, 1.5]
inp.intermediate_positions = [[1.0, 0.5], [2.0, 1.0]]
inp.max_velocity = [2.0, 2.0]
inp.max_acceleration = [2.0, 2.0]
inp.max_jerk = [4.0, 4.0]
inp.min_position = [0.0, 0.0]
inp.max_position = [3.0, 1.5]
out = OutputParameter(2)

result = otg.calculate_waypoints(inp, out.trajectory)
if result != Result.Working:
    raise RuntimeError(f'Calculation failed: {result}')
print(f'Stop at each waypoint: {out.trajectory.duration:.6f} s')

result = otg.calculate_waypoints(inp, out.trajectory, [[0.5, 0.25], [0.5, 0.25]], [[0.0, 0.0], [0.0, 0.0]])
if result != Result.Working:
    raise RuntimeError(f'Calculation failed: {result}')
print(f'Prescribed pass-through states: {out.trajectory.duration:.6f} s')

result = otg.optimize_waypoints(inp, out.trajectory)
if result != Result.Working:
    raise RuntimeError(f'Optimization failed: {result}')
print(f'Automatic pass-through states: {out.trajectory.duration:.6f} s')
for time in out.trajectory.intermediate_durations[:-1]:
    position, velocity, acceleration = out.trajectory.at_time(time)
    print(f'Waypoint {position}: velocity={velocity}, acceleration={acceleration}')

while True:
    result = otg.update(out.trajectory, out)
    if result == Result.Finished:
        break
    if result != Result.Working:
        raise RuntimeError(f'Execution failed: {result}')
print(f'Finished at {out.new_position}')
