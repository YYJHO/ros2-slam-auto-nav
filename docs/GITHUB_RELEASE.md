# GitHub Release Checklist

Use this checklist before pushing the project to a public repository.

## 1. Confirm Public Metadata

- Confirm the README clone URL uses `https://github.com/YYJHO/ros2-slam-auto-nav.git`.
- Confirm ROS package metadata uses the public maintainer name and email.
- Confirm `LICENSE` matches the license declared in `package.xml` and `setup.py`.
- Replace the placeholder contact section in `SECURITY.md`.

## 2. Run Local Preflight

```bash
bash scripts/preflight_release.sh
```

This checks shell syntax, Python syntax, package XML validity, ignored runtime
outputs, and common secret patterns.

## 3. Initialize Git

If this directory is not already a valid Git repository:

```bash
rm -rf .git
git init
git add .
git status
git commit -m "Initial release"
git branch -M main
git remote add origin https://github.com/YYJHO/ros2-slam-auto-nav.git
git push -u origin main
```

Only remove `.git` if you do not need any previous commit history.

## 4. Enable GitHub Security Features

In the repository settings, enable:

- Secret scanning.
- Push protection.
- Dependabot alerts.
- GitHub Actions.

For public repositories, some of these features may appear under
`Settings -> Security & analysis`.

## 5. Suggested Repository Topics

```text
ros2
humble
gazebo
nav2
slam-toolbox
indoor-navigation
robotics
simulation
python
rviz
```
