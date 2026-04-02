# Run From VS Code

This project can start two ArduPilot SITL drones and then run your Python scripts from inside VS Code.

## One-time setup

Open the WSL project folder in VS Code:

```bash
code /home/mmicha34/drone_project
```

If your ArduPilot checkout is not at `/home/mmicha34/ardupilot`, set this in the terminal before starting a SITL task:

```bash
export ARDUPILOT_DIR=/path/to/your/ardupilot
```

## Start the simulator

In VS Code:

1. Open `Terminal -> Run Task`.
2. Run `Start both SITL drones`.

That launches:

- drone1 on `udp:127.0.0.1:14550`
- drone2 on `udp:127.0.0.1:14560`

These ports match your Python files.

## Run your Python code

Use either:

- `Run and Debug` -> `Python: dual_status.py`
- `Run and Debug` -> `Python: drone1_to_drone2_follow.py`

Or from `Terminal -> Run Task`:

- `Run dual_status.py`
- `Run drone1_to_drone2_follow.py`

## Notes

- Stop a simulator task with `Ctrl+C` in its terminal.
- If `sim_vehicle.py` is missing, check `/home/mmicha34/ardupilot/Tools/autotest/sim_vehicle.py`.
- The VS Code Python interpreter is set to `.venv/bin/python`.