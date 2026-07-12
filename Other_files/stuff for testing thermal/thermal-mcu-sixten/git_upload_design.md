# Design: Uploading Project to GitHub

This design document outlines the steps to install Git on the Windows system and configure the current project directory to be tracked by Git and uploaded to the GitHub repository `https://github.com/davveMC/thermal-mcu.git`.

## Steps

### 1. Git Installation
- Check for `winget` command availability.
- Install Git using the Windows Package Manager command:
  ```powershell
  winget install --id Git.Git -e --source winget --accept-package-agreements --accept-source-agreements
  ```
- Reload environment path variables to make sure `git` is available.

### 2. Configuration & Initialization
- Initialize a new local Git repository with the default branch `main`:
  ```powershell
  git init -b main
  ```
- Ask for and configure Git user credentials globally (or locally if preferred):
  ```powershell
  git config --global user.name "<name>"
  git config --global user.email "<email>"
  ```
- Inspect current `.gitignore` to prevent tracking of intermediate PlatformIO builds (`.pio`), log/IDE files (`.vscode`, `.cache`), and compiled database files (`compile_commands.json`).
- Stage files:
  ```powershell
  git add .
  ```
- Create the initial commit:
  ```powershell
  git commit -m "Initial commit: PlatformIO ESP32-S3 firmware skeleton"
  ```

### 3. Remote Setup & Initial Push
- Add the remote origin pointing to the user's GitHub repository:
  ```powershell
  git remote add origin https://github.com/davveMC/thermal-mcu.git
  ```
- Push to the remote repository:
  ```powershell
  git push -u origin main
  ```
- Authentication will be handled interactively by the Git Credential Manager/Windows secure pop-up.
