# IS29 contained MQTT broker

A self-contained MQTT broker for the bench / obstacle tests, using
[Aedes](https://github.com/moscajs/aedes). Everything lives in this folder:
no system install, no Windows service, no Docker, no WSL. Native Windows Node
binds straight to the LAN IP the car can reach.

## Use

```bash
cd tools/broker
npm install        # one time -> ./node_modules (nothing system-wide)
npm start          # node broker.js, listens on 0.0.0.0:1883
```

It prints the LAN IPs to put in `config/net_mqtt_config.h`, then logs every
published message, so it also replaces `mosquitto_sub` for watching telemetry.
Stop with Ctrl-C. Remove everything by deleting this folder.

## Firewall (one time, needs admin)

The broker is contained, but the car is an external LAN device, so Windows
Firewall must allow inbound TCP 1883:

```powershell
New-NetFirewallRule -DisplayName "mosquitto 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow
```

Not needed for localhost-only testing.

## Topics

`inf2004/is29/command`, `inf2004/is29/telemetry`, `inf2004/is29/status`.
Anonymous connections are allowed (classroom LAN only).
