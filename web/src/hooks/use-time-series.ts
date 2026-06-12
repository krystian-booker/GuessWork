import { useEffect, useState } from 'react'

export type TimePoint = { t: number } & Record<string, number>

// Module-level buffers survive page navigation, so returning to the
// Dashboard keeps chart history while its queries were paused.
const buffers = new Map<string, { points: TimePoint[]; lastStamp: number }>()

export const TIME_SERIES_WINDOW_MS = 120_000

// Rolling time-series buffer fed by polled query data. Appends one point per
// distinct `updatedAt` (use the query's dataUpdatedAt so refetches that
// return identical values still advance the chart). `values` maps series key
// → current value; null/undefined/NaN entries are skipped for that tick.
export function useTimeSeries(
  bufferKey: string,
  values: Record<string, number | null | undefined>,
  updatedAt: number,
  windowMs: number = TIME_SERIES_WINDOW_MS,
): TimePoint[] {
  const [points, setPoints] = useState<TimePoint[]>(
    () => buffers.get(bufferKey)?.points ?? [],
  )

  useEffect(() => {
    const buf = buffers.get(bufferKey) ?? { points: [], lastStamp: 0 }
    if (updatedAt === 0 || updatedAt === buf.lastStamp) return
    buf.lastStamp = updatedAt

    const point: TimePoint = { t: updatedAt }
    let any = false
    for (const [k, v] of Object.entries(values)) {
      if (v != null && Number.isFinite(v)) {
        point[k] = v
        any = true
      }
    }
    if (!any) return

    const cutoff = updatedAt - windowMs
    const next = [...buf.points.filter((p) => p.t >= cutoff), point]
    buf.points = next
    buffers.set(bufferKey, buf)
    setPoints(next)
    // values is a fresh object every render; updatedAt alone gates appends.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [bufferKey, updatedAt, windowMs])

  return points
}
