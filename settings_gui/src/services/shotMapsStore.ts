// Simple cross-tab signal: set when the tuning tab saves a point to the
// local ShotMaps.json, consumed by ShotMapsEditor on its next mount so it
// reloads from local instead of the robot API.
let _localDirty = false;

export const shotMapsLocalSignal = {
  markDirty() { _localDirty = true; },
  /** Returns true (and clears the flag) if a local save happened since last check. */
  consume(): boolean { const was = _localDirty; _localDirty = false; return was; },
};
