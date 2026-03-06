// Raw NT4 WebSocket diagnostic - bypasses ntcore-ts-client entirely
// Usage: node nt4-debug.mjs [address]
import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';

const address = process.argv[2] || 'localhost';
const url = `ws://${address}:5810/nt/nt4-debug`;
console.log(`Connecting to ${url} ...`);

const ws = new WebSocket(url, ['networktables.first.wpi.edu']);

const topics = new Map(); // id -> { name, type }

ws.on('open', () => {
  console.log('Connected!\n');
  // Subscribe to everything with prefix "/"
  const subMsg = [
    { method: 'subscribe', params: { topics: ['/'], subuid: 1, options: { prefix: true, all: true, periodic: 0.1 } } }
  ];
  ws.send(JSON.stringify(subMsg));
});

ws.on('message', (data, isBinary) => {
  if (!isBinary) {
    // Text frame = JSON array of announce/unannounce messages
    const msgs = JSON.parse(data.toString());
    for (const msg of msgs) {
      if (msg.method === 'announce') {
        const { name, id, type } = msg.params;
        topics.set(id, { name, type });
        console.log(`  ANNOUNCE  id=${id}  type=${type.padEnd(20)}  ${name}`);
      }
    }
  } else {
    // Binary frame = msgpack array of [topicId, timestampUs, typeNum, value]
    const buf = Buffer.isBuffer(data) ? data : Buffer.concat(data);
    try {
      const decoded = decode(buf);
      if (Array.isArray(decoded)) {
        const [topicId, tsUs, typeNum, value] = decoded;
        const topic = topics.get(topicId);
        if (topic) {
          let preview;
          if (value instanceof Uint8Array || value instanceof ArrayBuffer) {
            const bytes = value instanceof ArrayBuffer ? new Uint8Array(value) : value;
            if (topic.type.startsWith('struct:') && bytes.length >= 16) {
              const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
              const doubles = [];
              for (let i = 0; i + 8 <= bytes.length; i += 8) {
                doubles.push(dv.getFloat64(i, true).toFixed(4));
              }
              preview = `[${doubles.join(', ')}]  (${bytes.length} bytes as float64 LE)`;
            } else {
              preview = `<${bytes.length} bytes>`;
            }
          } else {
            preview = JSON.stringify(value)?.substring(0, 80);
          }
          console.log(`  VALUE     ${topic.name} = ${preview}`);
        }
      }
    } catch (e) {
      // skip unparseable
    }
  }
});

ws.on('error', (err) => console.error('WS error:', err.message));
ws.on('close', () => { console.log('Disconnected'); process.exit(0); });

setTimeout(() => {
  console.log(`\n--- ${topics.size} topics found ---`);
  const sorted = [...topics.values()].sort((a, b) => a.name.localeCompare(b.name));
  console.log('\nAll topics:');
  for (const t of sorted) {
    console.log(`  ${t.type.padEnd(22)} ${t.name}`);
  }
  const relevant = sorted.filter(t => /odometry|shooter|hood/i.test(t.name));
  if (relevant.length) {
    console.log('\nRelevant (odometry/shooter/hood):');
    for (const t of relevant) {
      console.log(`  ${t.type.padEnd(22)} ${t.name}`);
    }
  }
  ws.close();
}, 5000);
