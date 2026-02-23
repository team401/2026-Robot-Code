# Temperature Monitor

Reads Linux thermal zone temperatures and publishes them to NetworkTables under `/Coprocessor/thermal/`. Each thermal zone's type becomes a DoubleTopic named `{type}_Celsius` (e.g. `x86_pkg_temp_Celsius`).

## Setup

On the coprocessor, install the Python dependencies into a venv:

```bash
cd /home/team401/coprocessor
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Running manually

Connect to a specific NT server:

```bash
.venv/bin/python temperature_monitor.py --server localhost
```

Connect via team number (mDNS):

```bash
.venv/bin/python temperature_monitor.py --team 401
```

## Installing the systemd service

Copy the unit file and enable it:

```bash
sudo cp temperature-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now temperature-monitor
```

The service runs as user `team401` and will automatically restart on failure.

## Checking status

```bash
sudo systemctl status temperature-monitor
journalctl -u temperature-monitor -f
```
