// Test raw NT4 WebSocket approach (same logic as nt4.ts but for Node)
import { WebSocket } from 'ws';
import { decodeMulti } from '@msgpack/msgpack';

const address = process.argv[2] || 'localhost';
const url = `ws://${address}:5810/nt/test-client`;
console.log(`Connecting to ${url} ...`);

const TOPICS = [
  '/AdvantageKit/RealOutputs/Odometry/Robot',
  '/AdvantageKit/Shooter/LeadMotorInputs/VelocityRadiansPerSecond',
  '/AdvantageKit/Shooter/FollowerMotorInputs/VelocityRadiansPerSecond',
  '/AdvantageKit/RealOutputs/Hood/exitAngleRadians',
];

const ws = new WebSocket(url, ['networktables.first.wpi.edu']);
const topicNames = new Map();
let updateCount = 0;

ws.on('open', () => {
  console.log('Connected!');
  ws.send(JSON.stringify([{
    method: 'subscribe',
    params: { topics: TOPICS, subuid: 1, options: { periodic: 0.1 } },
  }]));
});

ws.on('message', (data, isBinary) => {
  if (!isBinary) {
    const msgs = JSON.parse(data.toString());
    for (const msg of msgs) {
      if (msg.method === 'announce') {
        topicNames.set(msg.params.id, msg.params.name);
        console.log(`  Announced: ${msg.params.name} (type=${msg.params.type})`);
      }
    }
  } else {
    const buf = Buffer.isBuffer(data) ? data : Buffer.concat(data);
    for (const decoded of decodeMulti(buf)) {
      if (Array.isArray(decoded) && decoded.length >= 4) {
        const [topicId, , , value] = decoded;
        const name = topicNames.get(topicId) || `id=${topicId}`;
        if (name.includes('Odometry')) {
          const bytes = value instanceof Uint8Array ? value : new Uint8Array(value);
          const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
          const x = dv.getFloat64(0, true);
          const y = dv.getFloat64(8, true);
          if (updateCount++ < 10) console.log(`  Pose: x=${x.toFixed(4)}, y=${y.toFixed(4)}`);
        } else {
          if (updateCount++ < 10) console.log(`  ${name}: ${value}`);
        }
      }
    }
  }
});

ws.on('error', (err) => console.error('Error:', err.message));

setTimeout(() => {
  console.log(`\n${updateCount} value updates received. Done.`);
  ws.close();
  process.exit(0);
}, 5000);
