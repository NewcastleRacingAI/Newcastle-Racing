# Simulation

## Ansys

On linux, download the Ansys installer from the official website and launch it.
Note that the installer expects to be run as root.
Either log in as root and run it or follow the steps below.

### (Optional) Running the installer with sudo

If you cannot login as root but your account has sudo priviledges, you can still install the softwre.

- Since the installation is graphical, you may need to give the root user access to the display
  ```bash
  xhost si:localuser:root
  ```
- If you are using firefox from snap, you may run into an issue where, during the installation process, the installer tries to load a webpage but firefox does not find it, despite the file existing.
  The way we solved this was to install [Chrome](https://www.google.com/intl/en_uk/chrome/dr/download/) on the machine, set it as the default browser for root
  ```bash
  sudo xdg-settings set default-web-browser google-chrome.desktop
  ```
  Note that to run Chrome as root, the `--no-sandbox` flag must be added.
  A simple workaround to obtain this result is to edit the Chrome startup script.
  Substitute `$EDITOR` with your favourite (e.g., `code`, `nano`, `vim`)
  ```bash
  $EDITOR $(which google-chrome)
  ```
  and change the last line by adding `--no-sandbox`.
