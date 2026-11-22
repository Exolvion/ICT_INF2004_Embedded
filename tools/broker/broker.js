// Self-contained MQTT broker for the IS29 bench / obstacle tests.
// Native-Windows Node, so it binds straight to the LAN IP the car can reach
// (no Docker / WSL NAT). It also logs every published message, so it doubles
// as the telemetry viewer and you do not need mosquitto_sub.
//
//   cd tools/broker
//   npm install        # one time, into ./node_modules (nothing system-wide)
//   npm start          # or: node broker.js   (PORT=1883 by default)
//
// Stop with Ctrl-C. Remove it all by deleting the tools/broker folder.

const net = require('net');
const os = require('os');
const aedes = require('aedes')();

const PORT = Number(process.env.PORT || 1883);
const HOST = process.env.HOST || '0.0.0.0';

function stamp() {
  return new Date().toTimeString().slice(0, 8);
}

const server = net.createServer(aedes.handle);

server.listen(PORT, HOST, () => {
  console.log(`[${stamp()}] IS29 broker listening on ${HOST}:${PORT}`);
  const ifaces = os.networkInterfaces();
  const addrs = [];
  for (const name of Object.keys(ifaces)) {
    for (const i of ifaces[name] || []) {
      if (i.family === 'IPv4' && !i.internal) addrs.push(`${i.address}  (${name})`);
    }
  }
  console.log('  set NET_MQTT_BROKER_IPV4 in config/net_mqtt_config.h to one of:');
  for (const a of addrs) console.log(`    ${a}`);
  console.log('  waiting for clients...');
});

server.on('error', (err) => {
  if (err.code === 'EADDRINUSE') {
    console.error(`[${stamp()}] port ${PORT} already in use — another broker is running?`);
  } else {
    console.error(`[${stamp()}] server error:`, err.message);
  }
  process.exit(1);
});

aedes.on('client', (c) => console.log(`[${stamp()}] + connect    ${c ? c.id : '?'}`));
aedes.on('clientDisconnect', (c) => console.log(`[${stamp()}] - disconnect ${c ? c.id : '?'}`));
aedes.on('clientError', (c, e) => console.log(`[${stamp()}] ! error      ${c ? c.id : '?'}: ${e.message}`));
aedes.on('subscribe', (subs, c) =>
  console.log(`[${stamp()}]   subscribe  ${c ? c.id : '?'} -> ${subs.map((s) => s.topic).join(', ')}`));

aedes.on('publish', (packet, client) => {
  if (!client) return; // skip $SYS / internal keep-alive traffic
  const payload = packet.payload ? packet.payload.toString('utf8') : '';
  console.log(`[${stamp()}] ${packet.topic}  ${payload}`);
});

process.on('SIGINT', () => {
  console.log(`\n[${stamp()}] shutting down`);
  aedes.close(() => server.close(() => process.exit(0)));
});
