import { HUB_CENTER } from '../types/ShotTuning';

export interface Telemetry {
  distanceMeters: number;
  shooterRPMRadPerSec: number;
  hoodAngleRadians: number;
  robotPoseX: number;
  robotPoseY: number;
}

export interface NT4Service {
  disconnect(): void;
  addConnectionListener(cb: (connected: boolean) => void): () => void;
  isConnected(): boolean;
}

// AdvantageKit publishes Logger.recordOutput / @AutoLogOutput under this prefix.
const AK_PREFIX = '/AdvantageKit/RealOutputs';

// Actual topics discovered via nt4-debug.mjs against the robot sim:
const TOPIC_POSE = `${AK_PREFIX}/Odometry/Robot`;                                     // struct:Pose2d
// processInputs topics live under /AdvantageKit/<subsystem>/ (no RealOutputs).
const TOPIC_SHOOTER_LEAD = '/AdvantageKit/Shooter/LeadMotorInputs/VelocityRadiansPerSecond';       // double
const TOPIC_SHOOTER_FOLLOWER = '/AdvantageKit/Shooter/FollowerMotorInputs/VelocityRadiansPerSecond'; // double
const TOPIC_HOOD_ANGLE = `${AK_PREFIX}/Hood/exitAngleRadians`;                                      // double

// Minimal raw NT4.1 WebSocket client.
// ntcore-ts-client cannot handle WPILib struct types (crashes on Zod validation
// in v3.1.x, and on ArrayBuffer data validation in v3.2.0-beta.0).

export async function createNT4Service(
  address: string,
  onTelemetry: (update: Telemetry) => void,
): Promise<NT4Service> {
  const { decodeMulti } = await import('@msgpack/msgpack');

  let poseX = 0;
  let poseY = 0;
  let shooterLeadVel = 0;
  let shooterFollowerVel = 0;
  let hoodAngle = 0;

  function emitUpdate() {
    const dx = poseX - HUB_CENTER.x;
    const dy = poseY - HUB_CENTER.y;
    onTelemetry({
      distanceMeters: Math.sqrt(dx * dx + dy * dy),
      shooterRPMRadPerSec: (shooterLeadVel + shooterFollowerVel) / 2,
      hoodAngleRadians: hoodAngle,
      robotPoseX: poseX,
      robotPoseY: poseY,
    });
  }

  // Map server-assigned topic id -> handler
  const topicHandlers = new Map<number, (value: unknown) => void>();

  // Topic name -> handler, wired up when server announces each topic
  const wantedTopics = new Map<string, (value: unknown) => void>();

  wantedTopics.set(TOPIC_POSE, (value) => {
    // struct:Pose2d = 3x float64 LE (x, y, rotation) = 24 bytes
    const bytes = value instanceof Uint8Array ? value : new Uint8Array(value as ArrayBuffer);
    if (bytes.byteLength >= 16) {
      const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      poseX = dv.getFloat64(0, true);
      poseY = dv.getFloat64(8, true);
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_SHOOTER_LEAD, (value) => {
    if (typeof value === 'number') {
      shooterLeadVel = value;
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_SHOOTER_FOLLOWER, (value) => {
    if (typeof value === 'number') {
      shooterFollowerVel = value;
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_HOOD_ANGLE, (value) => {
    if (typeof value === 'number') {
      hoodAngle = value;
      emitUpdate();
    }
  });

  const connectionListeners = new Set<(connected: boolean) => void>();
  let connected = false;
  let ws: WebSocket | null = null;
  let disposed = false;
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

  function setConnected(c: boolean) {
    if (connected === c) return;
    connected = c;
    for (const cb of connectionListeners) cb(c);
  }

  function connect() {
    if (disposed) return;

    const url = `ws://${address}:5810/nt/settings-gui`;
    ws = new WebSocket(url, ['networktables.first.wpi.edu']);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
      setConnected(true);
      // Subscribe to our specific topics
      ws!.send(JSON.stringify([{
        method: 'subscribe',
        params: {
          topics: [...wantedTopics.keys()],
          subuid: 1,
          options: { periodic: 0.1 },
        },
      }]));
    };

    ws.onmessage = (event) => {
      if (typeof event.data === 'string') {
        // Text frame: JSON array of announce/unannounce messages
        try {
          const msgs = JSON.parse(event.data) as Array<{ method: string; params: Record<string, unknown> }>;
          for (const msg of msgs) {
            if (msg.method === 'announce') {
              const name = msg.params.name as string;
              const id = msg.params.id as number;
              const handler = wantedTopics.get(name);
              if (handler) topicHandlers.set(id, handler);
            } else if (msg.method === 'unannounce') {
              topicHandlers.delete(msg.params.id as number);
            }
          }
        } catch { /* ignore */ }
      } else {
        // Binary frame: one or more msgpack arrays [topicId, tsUs, typeNum, value]
        try {
          const buf = new Uint8Array(event.data as ArrayBuffer);
          for (const decoded of decodeMulti(buf)) {
            const arr = decoded as unknown[];
            if (Array.isArray(arr) && arr.length >= 4) {
              const handler = topicHandlers.get(arr[0] as number);
              if (handler) handler(arr[3]);
            }
          }
        } catch { /* ignore */ }
      }
    };

    ws.onclose = () => {
      setConnected(false);
      topicHandlers.clear();
      if (!disposed) {
        reconnectTimer = setTimeout(connect, 1000);
      }
    };

    ws.onerror = () => { /* onclose fires after */ };
  }

  connect();

  return {
    disconnect() {
      disposed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (ws) {
        ws.onclose = null;
        ws.close();
        ws = null;
      }
      setConnected(false);
    },
    addConnectionListener(cb: (connected: boolean) => void) {
      connectionListeners.add(cb);
      cb(connected);
      return () => { connectionListeners.delete(cb); };
    },
    isConnected() {
      return connected;
    },
  };
}
