# Contributing

Thanks for helping improve this ROS 2 navigation project.

## Development Environment

Use the same baseline as the project documentation:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Python 3.10

Install dependencies and build with:

```bash
bash scripts/install_dependencies.sh
bash scripts/build_workspace.sh
```

## Local Checks

Run the release preflight before opening a pull request:

```bash
bash scripts/preflight_release.sh
```

For runtime checks in a full ROS desktop environment:

```bash
bash scripts/check_system.sh
bash scripts/run_control_center.sh
```

## Pull Request Guidelines

- Keep generated runtime data out of commits.
- Do not commit `workspace/build/`, `workspace/install/`, `workspace/log/`, saved maps, diagnostics, or local room files.
- Update `README.md` or `INSTALL.md` when changing user-facing commands.
- Prefer small changes that can be tested in simulation.
