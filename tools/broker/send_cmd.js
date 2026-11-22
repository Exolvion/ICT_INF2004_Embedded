// Send one command to the IS29 car through the local broker.
//
//   node send_cmd.js map F=A U=Z L=B R=C   letter -> junction move map
//   node send_cmd.js start | stop | reset
//
// The map is the 3-byte binary message of app_program/subsystem1/navmap.h
// (Change Requirement 1, minimum bits): bits 0-4 FORWARD, 5-9 U-TURN,
// 10-14 LEFT, 15-19 RIGHT (each letter 0 = A .. 25 = Z), bits 20-23 the
// pattern 1011b, least significant byte first. The car answers on the
// telemetry topic with "MAP set: forward A, U-turn Z, left B, right C".
// Commands are the one-byte codes of mqttcmd.h (0xA8 | code).
//
// Broker: MQTT_URL (default mqtt://127.0.0.1:1883).

const mqtt = require('mqtt');

const TOPIC = 'inf2004/is29/command';
const URL = process.env.MQTT_URL || 'mqtt://127.0.0.1:1883';
const COMMANDS = { stop: 0xa9, start: 0xaa, reset: 0xab };
const MOVES = ['F', 'U', 'L', 'R']; // message order
const PATTERN = 0xb;

function packMap(args) {
  const letters = {};
  for (const arg of args) {
    const m = /^([FULR])=([A-Za-z])$/i.exec(arg);
    if (!m) throw new Error(`bad map entry "${arg}" (want e.g. L=B)`);
    letters[m[1].toUpperCase()] = m[2].toUpperCase();
  }
  const missing = MOVES.filter((k) => !(k in letters));
  if (missing.length) throw new Error(`map needs all four of F U L R; missing ${missing.join(' ')}`);
  const used = MOVES.map((k) => letters[k]);
  if (new Set(used).size !== 4) throw new Error('the four letters must all be different');
  let word = PATTERN << 20;
  MOVES.forEach((k, i) => {
    word |= (letters[k].charCodeAt(0) - 65) << (5 * i);
  });
  return {
    payload: Buffer.from([word & 0xff, (word >> 8) & 0xff, (word >> 16) & 0xff]),
    text: `forward ${letters.F}, U-turn ${letters.U}, left ${letters.L}, right ${letters.R}`,
  };
}

function main() {
  const [what, ...rest] = process.argv.slice(2);
  let payload;
  let text;
  if (what === 'map') {
    ({ payload, text } = packMap(rest));
  } else if (what && what.toLowerCase() in COMMANDS) {
    payload = Buffer.from([COMMANDS[what.toLowerCase()]]);
    text = what.toUpperCase();
  } else {
    console.error('usage: node send_cmd.js map F=A U=Z L=B R=C | start | stop | reset');
    process.exit(2);
  }
  const client = mqtt.connect(URL, { clientId: `is29-send-${process.pid}`, reconnectPeriod: 0 });
  client.on('connect', () => {
    client.publish(TOPIC, payload, { qos: 0, retain: false }, (err) => {
      if (err) {
        console.error('publish failed:', err.message);
        process.exitCode = 1;
      } else {
        console.log(`sent ${text} (${payload.toString('hex')})`);
      }
      client.end();
    });
  });
  client.on('error', (e) => {
    console.error('broker:', e.message);
    process.exit(1);
  });
}

if (require.main === module) {
  main();
} else {
  module.exports = { packMap };
}
