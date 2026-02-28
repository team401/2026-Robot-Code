import { HUB_CENTER } from '../types/ShotTuning';

export interface Telemetry {
  /** Pose-computed distance to hub (meters) */
  distanceMeters: number;
  /** NT-reported distance to hub from robot (meters); null until first value received */
  distanceToHubMeters: number | null;
  /** Shooter setpoint in RPM (from TunableNumbers) */
  shooterRPM: number;
  /** Hood setpoint in degrees (from TunableNumbers) */
  hoodAngleDegrees: number;
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
// TunableNumbers are logged under NetworkInputs by AdvantageKit.
const TOPIC_SHOOTER_RPM = '/AdvantageKit/NetworkInputs/TunableNumbers/CoordinationLayer/ShotTuning/shooterRPM';       // double, in RPM
const TOPIC_HOOD_DEG    = '/AdvantageKit/NetworkInputs/TunableNumbers/CoordinationLayer/ShotTuning/hoodAngleDegrees'; // double, in degrees
const TOPIC_DISTANCE_HUB = `${AK_PREFIX}/CoordinationLayer/distanceToHub`;                                            // double, in meters

// Minimal raw NT4.1 WebSocket client.
// ntcore-ts-client cannot handle WPILib struct types (crashes on Zod validation
// in v3.1.x, and on ArrayBuffer data validation in v3.2.0-beta.0).

export async function createNT4Service(
  address: string,
  onTelemetry: (update: Telemetry) => void,
  shooterOffset: { x: number; y: number } = { x: 0, y: 0 },
): Promise<NT4Service> {
  const { decodeMulti } = await import('@msgpack/msgpack');

  let poseX = 0;
  let poseY = 0;
  let poseHeading = 0; // radians
  let shooterRPM = 0;
  let hoodAngleDegrees = 0;
  let distanceToHubMeters: number | null = null;

  function emitUpdate() {
    // Rotate the robot-frame shooter offset into field frame using the robot heading.
    const shooterX = poseX + shooterOffset.x * Math.cos(poseHeading) - shooterOffset.y * Math.sin(poseHeading);
    const shooterY = poseY + shooterOffset.x * Math.sin(poseHeading) + shooterOffset.y * Math.cos(poseHeading);
    const dx = shooterX - HUB_CENTER.x;
    const dy = shooterY - HUB_CENTER.y;
    onTelemetry({
      distanceMeters: Math.sqrt(dx * dx + dy * dy),
      distanceToHubMeters,
      shooterRPM,
      hoodAngleDegrees,
      robotPoseX: poseX,
      robotPoseY: poseY,
    });
  }

  // Map server-assigned topic id -> handler
  const topicHandlers = new Map<number, (value: unknown) => void>();

  // Topic name -> handler, wired up when server announces each topic
  const wantedTopics = new Map<string, (value: unknown) => void>();

  wantedTopics.set(TOPIC_POSE, (value) => {
    // struct:Pose2d = 3x float64 LE (x, y, rotation_radians) = 24 bytes
    const bytes = value instanceof Uint8Array ? value : new Uint8Array(value as ArrayBuffer);
    if (bytes.byteLength >= 24) {
      const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      poseX = dv.getFloat64(0, true);
      poseY = dv.getFloat64(8, true);
      poseHeading = dv.getFloat64(16, true);
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_SHOOTER_RPM, (value) => {
    if (typeof value === 'number') {
      shooterRPM = value;
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_HOOD_DEG, (value) => {
    if (typeof value === 'number') {
      hoodAngleDegrees = value;
      emitUpdate();
    }
  });

  wantedTopics.set(TOPIC_DISTANCE_HUB, (value) => {
    if (typeof value === 'number') {
      distanceToHubMeters = value;
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
