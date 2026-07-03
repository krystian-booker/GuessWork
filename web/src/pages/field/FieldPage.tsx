import { useMemo } from 'react'
import { Link } from 'react-router-dom'
import { Map } from 'lucide-react'
import type { AprilTagCameraStatus } from '@/api/apriltag'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { EmptyState } from '@/components/EmptyState'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { FieldView } from '@/components/field/FieldView'
import { parseLayoutText } from '@/components/field/fieldGeometry'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatDeg, formatMeters, formatMs } from '@/lib/format'
import { useAprilTagStatus } from '@/queries/apriltag'
import { useFieldLayout, useFieldLayouts } from '@/queries/fieldLayouts'
import { useFusionStatus } from '@/queries/fusion'
import { LayoutManager } from './LayoutManager'

function DetectionCard({ cam }: { cam: AprilTagCameraStatus }) {
  const skipped =
    cam.skipped_no_tags +
    cam.skipped_ambiguous +
    cam.skipped_high_reproj +
    cam.skipped_no_extrinsics +
    cam.skipped_solve_failed
  return (
    <div className="rounded-md border px-3 py-2">
      <div className="flex items-center justify-between">
        <span className="flex items-center gap-2 text-sm font-medium">
          <StatusDot tone={cam.running ? 'good' : 'bad'} />
          {cam.name}
        </span>
        <span className="font-mono text-xs tabular-nums text-muted-foreground">
          {cam.det_per_s.toFixed(1)} det/s
        </span>
      </div>
      {!cam.running && cam.reason && (
        <p className="mt-1 text-xs text-warning">{cam.reason}</p>
      )}
      {cam.running && (
        <>
          <p className="mt-1 font-mono text-xs tabular-nums text-muted-foreground">
            {formatMs(cam.latency_ewma_ms)} latency · {cam.mean_reproj_err_px.toFixed(2)} px reproj
            {skipped > 0 ? ` · ${skipped} gated` : ''}
          </p>
          {cam.last_tags.length > 0 && (
            <div className="mt-1.5 flex flex-wrap gap-1">
              {cam.last_tags.map((t) => (
                <Badge key={t.id} variant="secondary" className="font-mono text-[11px]">
                  #{t.id}
                  {t.range_m != null ? ` ${formatMeters(t.range_m, 1)}` : ''}
                </Badge>
              ))}
            </div>
          )}
        </>
      )}
    </div>
  )
}

export default function FieldPage() {
  const layouts = useFieldLayouts()
  const apriltag = useAprilTagStatus()
  const fusion = useFusionStatus()

  const activeId = layouts.data?.find((l) => l.active)?.id ?? null
  const layout = useFieldLayout(activeId)
  const model = useMemo(
    () => (layout.data ? parseLayoutText(layout.data.json) : null),
    [layout.data],
  )

  const pose = fusion.data?.pose ?? null
  const stalenessP95 = fusion.data?.latency.pose_staleness.p95_ms
  const trail = useTimeSeries(
    'field-trail',
    { x: pose?.x_m, y: pose?.y_m },
    fusion.dataUpdatedAt,
    60_000,
  )

  const highlight = useMemo(() => {
    const ids = new Set<number>()
    for (const cam of apriltag.data?.cameras ?? []) {
      if (!cam.running) continue
      for (const t of cam.last_tags) ids.add(t.id)
    }
    return ids
  }, [apriltag.data])

  return (
    <div>
      <PageHeader
        title="Field"
        description={
          model
            ? `${layout.data?.name} — live fused pose and tag detections.`
            : 'Activate a field layout to render the field.'
        }
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-3">
        <div className="space-y-3 xl:col-span-2">
          {model ? (
            <>
              <FieldView
                model={model}
                pose={pose ? { x: pose.x_m, y: pose.y_m, thetaRad: pose.theta_rad } : null}
                poseStale={(stalenessP95 ?? 0) > 100}
                trail={trail.filter((p) => p.x != null && p.y != null).map((p) => ({ x: p.x, y: p.y }))}
                highlightTagIds={highlight}
              />
              <div className="flex flex-wrap items-center gap-x-6 gap-y-1 font-mono text-xs tabular-nums text-muted-foreground">
                {pose ? (
                  <>
                    <span>x {formatMeters(pose.x_m)}</span>
                    <span>y {formatMeters(pose.y_m)}</span>
                    <span>θ {formatDeg(pose.theta_rad)}</span>
                    <span>staleness p95 {formatMs(stalenessP95, 0)}</span>
                  </>
                ) : (
                  <span>no fused pose — {fusion.data?.mode || 'fusion not initialized'}</span>
                )}
              </div>
            </>
          ) : (
            <EmptyState
              icon={Map}
              title="No active field layout"
              description="Upload and activate a WPILib AprilTagFieldLayout to render the field."
            />
          )}
        </div>

        <div className="space-y-4">
          <LayoutManager />
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="flex items-center justify-between text-sm">
                AprilTag detection
                <Link to="/apriltag" className="text-xs font-normal text-primary hover:underline">
                  details →
                </Link>
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 px-4">
              {apriltag.data && !apriltag.data.t_robot_imu_set && (
                <p className="text-xs text-warning">
                  T_robot_imu is not set (Robot page) — detections run but field poses can't be
                  published.
                </p>
              )}
              {(apriltag.data?.cameras ?? []).map((cam) => (
                <DetectionCard key={cam.camera_id} cam={cam} />
              ))}
              {apriltag.data?.cameras.length === 0 && (
                <p className="text-sm text-muted-foreground">
                  No cameras have the AprilTag role.
                </p>
              )}
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  )
}
