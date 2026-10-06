# Contributing to DRON

Thanks for helping. This guide covers setup, branch and commit rules, and what we expect in a pull request. Project background lives in the [README](README.md).

## Repository layout

| Path | Contents |
| --- | --- |
| `ros2_ws/` | ROS 2 workspace (`camera`, `gps`, `temperature`, `thermal_cam`, `custom_interfaces`) |
| `Automation/` | PX4 offboard control nodes (`px4_ros_com` examples) |
| `Stereovision/` | Stereo camera capture, calibration, and disparity code |
| `senxor/` | Vendored MI48 thermal camera library |
| `DRON_Interface/` | Unity visualization project |
| `PX4-Autopilot/`, `ws_sensor_combined/` | Intended PX4 workspace locations — currently empty clones |

## Setup

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04. (CI assumes Humble; update `ros2-ci.yml` and this file if the team moves distros.)
2. Clone: `git clone https://github.com/turtle-robotics/DRON.git`
3. Build the ROS workspace:
   ```bash
   cd ros2_ws
   rosdep install --from-paths src --ignore-src -y \
     --skip-keys "senxor cmapy board busio adafruit_gps"
   colcon build
   source install/setup.bash
   ```
4. Enable pre-commit hooks (they check merges, YAML, Python syntax, and whitespace before every commit):
   ```bash
   pip install pre-commit
   pre-commit install
   ```

Some sensors need packages rosdep cannot install (`cmapy`, Adafruit Blinka: `board`, `busio`, `adafruit_gps`). Install those with pip on the Raspberry Pi.

## Branches

Branch from `main`. Use lowercase Conventional-Commit-style names:

```
feat/waypoint-planner
fix/gps-frame-drop
docs/electrical-power-notes
ci/ros2-build-cache
```

One branch per task. Keep branches short-lived; rebase or merge `main` into long-lived branches regularly.

## Commits and PR titles

Write commit messages and PR titles as [Conventional Commits](https://www.conventionalcommits.org):

```
<type>(<optional scope>): <short lowercase summary>
```

| Type | Use for |
| --- | --- |
| `feat` | New capability |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `test` | Tests only |
| `ci` | Workflows and CI config |
| `refactor` | No behavior change |
| `perf` | Performance change |
| `build` | Build system or dependencies |
| `chore` | Maintenance |
| `revert` | Reverts a previous commit |

Common scopes: `autonomy`, `ros2`, `px4`, `stereovision`, `visualization`, `sensors`, `electrical`, `mechanical`, `deps`. Examples:

- `feat(autonomy): add waypoint planner`
- `fix(stereovision): correct calibration transform`
- `docs(electrical): document power distribution`

PR titles are validated by the **PR Title / Conventional Commits** check. Invalid titles block merge.

## Issues

Open an issue before starting non-trivial work:

- **Bug report** — something in the software or repository is wrong.
- **Feature request** — you propose a new capability or improvement.
- **Hardware report** — a physical component failed or needs redesign.
- **Test report** — record bench, sensor, integration, autonomy, or flight test results. Open one after every flight test.

For quick questions, ask in the team chat instead.

## Pull requests

1. Create a branch (see above) and push your work.
2. Open the PR against `main` using the PR template. Fill the sections that apply; delete sections that do not.
3. The PR template checkboxes are self-attested. Check only what you actually did.
4. Wait for the checks listed below, then request review.

### Automated checks

| Check | What it does |
| --- | --- |
| `PR Title / Conventional Commits` | Validates the PR title |
| `Repository / Sanity` | Merge markers, YAML/JSON/Python syntax, large files, whitespace |
| `ROS 2 / Build & Test` | Builds `ros2_ws` with colcon and runs `colcon test` |
| `CodeQL` | Security scan of Python and C# |
| `PR Labels / Apply Subsystem Labels` | Labels the PR by touched paths |

### Local validation before opening a PR

```bash
pre-commit run --all-files   # same checks as Repository / Sanity
cd ros2_ws && colcon build && colcon test && colcon test-result --verbose
```

### Testing expectations

- Software changes: build `ros2_ws` and run the package tests locally. New ROS nodes should ship with at least one test.
- Changes that need hardware you cannot access: state that in the PR and ask a hardware-team member to validate.

### Hardware testing expectations

- Anything that flies needs a **Test report** issue afterwards, including anomalies and follow-ups.
- Propulsion, battery, or control changes require a bench test before a flight test.
- Follow local FAA rules and the team's flight-safety checklist for every flight.

### Review expectations

- At least one approving review from a team lead before merge.
- Reviewers: check that validation is honest, the PR is in scope, and safety-relevant changes are called out.
- Respond to review comments by pushing commits, not force-pushing new work silently.

### Merge strategy

We **squash-merge** into `main`. GitHub makes the squash commit subject equal to the PR title, so the final PR title is the commit message — keep it Conventional and descriptive. Linear history is enforced on `main`; delete the branch after merge.

## Labels

Subsystem labels are applied automatically from touched paths:

| Label | Paths |
| --- | --- |
| `ros2` | `ros2_ws/**` |
| `autonomy` | `Automation/**`, `PX4-Autopilot/**`, `ws_sensor_combined/**` |
| `stereovision` | `Stereovision/**`, `SV_In/**`, `calibration_params/**` |
| `sensors` | `senxor/**` |
| `visualization` | `DRON_Interface/**` |
| `documentation` | top-level `*.md` |
| `ci-cd` | `.github/**`, `.pre-commit-config.yaml` |

Maintainers create labels once:

```bash
for l in feature fix documentation test ci-cd refactor performance build chore \
         "priority: high" "priority: medium" "priority: low" blocked "needs review" \
         autonomy ros2 px4 stereovision visualization sensors electrical mechanical documentation ci-cd; do
  gh label create "$l" --repo turtle-robotics/DRON
done
```

| Label | Meaning |
| --- | --- |
| `feature` / `fix` / `documentation` / `test` / `refactor` / `performance` / `build` / `chore` | Issue or PR kind |
| `priority: high|medium|low` | Urgency |
| `blocked` | Waiting on something external |
| `needs review` | Ready for a maintainer |
| `autonomy`, `ros2`, `px4`, `stereovision`, `visualization`, `sensors`, `electrical`, `mechanical`, `ci-cd` | Subsystem (also usable on issues) |

GitHub Actions cannot create labels by themselves without broad write access, so a maintainer runs the loop above once after merge.
