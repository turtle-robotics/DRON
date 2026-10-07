# Contributing to DRON

Project background lives in the [README](README.md).

## Setup

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04. CI assumes Humble; update `.github/workflows/ros2-ci.yml` and this file if the team moves distros.
2. Clone and build:

   ```bash
   git clone https://github.com/turtle-robotics/DRON.git
   cd DRON
   source /opt/ros/humble/setup.bash
   sudo rosdep init   # once per machine
   rosdep update

   cd ros2_ws
   rosdep install --from-paths src --ignore-src -y \
     --skip-keys "senxor matplotlib cv2 spidev cmapy board busio adafruit_gps"
   colcon build
   source install/setup.bash

   pip install pre-commit && pre-commit install
   ```

Some sensors need packages rosdep cannot install: `cmapy`, Adafruit Blinka (`board`, `busio`, `adafruit_gps`), plus `matplotlib`, `cv2` (OpenCV), and `spidev`, which have no rosdep keys. Install them with pip (`pip install matplotlib opencv-python spidev` covers the last three).

## Branches and commits

Branch from `main`, one branch per task, named `<type>/short-desc`:

```
feat/waypoint-planner
```

Write commits and PR titles as [Conventional Commits](https://www.conventionalcommits.org):

```
<type>(<optional scope>): <short lowercase summary>
```

Types: `feat`, `fix`, `docs`, `test`, `ci`, `refactor`, `perf`, `build`, `chore`, `revert`. Example scopes: `autonomy`, `ros2`, `px4`, `stereovision`, `visualization`, `sensors`, `electrical`, `mechanical`.

```
feat(autonomy): add waypoint planner
```

CI validates the PR title (types above, optional scope). The squash commit reuses the PR title, so keep it descriptive.

## Issues and pull requests

Open an issue before starting non-trivial work: **Bug report**, **Feature request**, **Hardware report**, or **Test report** (file one after every flight test). Quick questions go to the team chat.

For a pull request: branch, push, open against `main` using the PR template, and check only what you actually did. Wait for CI, get one approving review from a team lead, then squash-merge and delete the branch.

## Testing

```bash
pre-commit run --all-files   # same checks CI runs
cd ros2_ws && colcon build && colcon test && colcon test-result --verbose
```

- New ROS nodes should ship with at least one test.
- Changes that need hardware you cannot access: state that in the PR and ask a hardware-team member to validate.
- Propulsion, battery, or control changes need a bench test before a flight test. Follow local FAA rules and the team's flight-safety checklist for every flight.

## Labels

Subsystem labels are applied automatically from touched paths — see `.github/labeler.yml` for the mappings. Maintainers create the label set once:

```bash
for l in feature fix documentation test refactor performance build chore \
         "priority: high" "priority: medium" "priority: low" blocked "needs review" \
         autonomy ros2 px4 stereovision visualization sensors electrical mechanical ci-cd; do
  gh label create "$l" --repo turtle-robotics/DRON
done
```

The set covers issue/PR kind, `priority: high|medium|low`, `blocked`, `needs review`, and subsystems.
