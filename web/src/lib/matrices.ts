// 4x4 row-major homogeneous transform as published by the backend
// (gw JSON convention: number[4][4]).
export type Mat4 = number[][]

export function xyFromMat4(T: Mat4): { x: number; y: number } {
  return { x: T[0][3], y: T[1][3] }
}

// Planar heading about +Z. Matches the server's own pose extraction
// (routes_fusion.cpp: atan2(T[1][0], T[0][0])).
export function yawFromMat4(T: Mat4): number {
  return Math.atan2(T[1][0], T[0][0])
}

// Yaw about +Z from a WPILib quaternion (W,X,Y,Z), Z-up field frame.
export function yawFromQuaternion(w: number, x: number, y: number, z: number): number {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
}

export function isMat4(v: unknown): v is Mat4 {
  return (
    Array.isArray(v) &&
    v.length === 4 &&
    v.every((row) => Array.isArray(row) && row.length === 4 && row.every((n) => typeof n === 'number'))
  )
}
