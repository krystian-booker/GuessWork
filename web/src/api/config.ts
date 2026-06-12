import { ApiError, readError, sendJson } from './http'

export interface ImportSectionReport {
  created: number
  updated: number
  errors: string[]
}

export interface ImportResult {
  ok: boolean
  report: Record<string, ImportSectionReport>
  note?: string
}

// Full robot-identity snapshot (cameras incl. calibration blobs, layouts,
// trigger groups, imu/vio/can/fusion configs). Returned as a Blob so the
// caller can trigger a browser download.
export async function exportConfig(): Promise<{ blob: Blob; filename: string }> {
  const res = await fetch('/api/config/export')
  if (!res.ok) throw new ApiError(await readError(res), res.status)
  const disposition = res.headers.get('Content-Disposition') ?? ''
  const match = /filename="?([^";]+)"?/.exec(disposition)
  return {
    blob: await res.blob(),
    filename: match?.[1] ?? `guesswork-config-${Date.now()}.json`,
  }
}

// Non-destructive merge (cameras matched by serial, groups/layouts by name);
// server propagates to apriltag/vio/fusion and re-pushes CAN mode after.
export async function importConfig(snapshot: unknown): Promise<ImportResult> {
  return sendJson<ImportResult>('/api/config/import', 'POST', snapshot)
}
