import { Link } from 'react-router-dom'
import { Camera } from 'lucide-react'
import type { AprilTagCameraStatus } from '@/api/apriltag'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { EmptyState } from '@/components/EmptyState'
import { PageHeader } from '@/components/PageHeader'
import { StatCard } from '@/components/StatCard'
import { StatusDot } from '@/components/StatusDot'
import { useChangeAge } from '@/hooks/use-change-age'
import { formatAgeMs, formatCount, formatDeg, formatMeters, formatMs } from '@/lib/format'
import { xyFromMat4, yawFromMat4 } from '@/lib/matrices'
import { useAprilTagStatus } from '@/queries/apriltag'

const SKIP_REASONS = [
  { key: 'skipped_no_tags', label: 'No tags in frame' },
  { key: 'skipped_ambiguous', label: 'Ambiguous single tag' },
  { key: 'skipped_high_reproj', label: 'High reprojection error' },
  { key: 'skipped_no_extrinsics', label: 'No extrinsics chain' },
  { key: 'skipped_solve_failed', label: 'Solve failed' },
] as const

type SkipKey = (typeof SKIP_REASONS)[number]['key']

// Labeled counts with proportional bars; the dominant nonzero reason is
// highlighted so "why isn't this camera publishing?" reads at a glance.
function SkipBreakdown({ cam }: { cam: AprilTagCameraStatus }) {
  const rows = SKIP_REASONS.map((r) => ({ ...r, count: cam[r.key as SkipKey] }))
  const max = Math.max(...rows.map((r) => r.count))
  return (
    <div className="space-y-1.5">
      <p className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
        Skipped frames
      </p>
      {rows.map((r) => {
        const dominant = r.count > 0 && r.count === max
        return (
          <div key={r.key} className="space-y-0.5" data-testid={`skip-${r.key}`}>
            <div className="flex justify-between text-xs">
              <span className={dominant ? 'font-medium text-warning' : 'text-muted-foreground'}>
                {r.label}
              </span>
              <span
                className={`font-mono tabular-nums ${dominant ? 'font-medium text-warning' : ''}`}
              >
                {formatCount(r.count)}
              </span>
            </div>
            <div className="h-1.5 overflow-hidden rounded-sm bg-muted">
              <div
                className={`h-full rounded-sm ${dominant ? 'bg-warning' : 'bg-chart-2/50'}`}
                style={{ width: max > 0 ? `${(r.count / max) * 100}%` : '0%' }}
              />
            </div>
          </div>
        )
      })}
    </div>
  )
}

function Metric({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div>
      <p className="text-[10px] font-medium tracking-wide text-muted-foreground uppercase">
        {label}
      </p>
      <p className="font-mono text-sm font-semibold tabular-nums">{value}</p>
    </div>
  )
}

function CameraCard({ cam }: { cam: AprilTagCameraStatus }) {
  const poseAge = useChangeAge(cam.last_pose_t_ns)
  const pose = cam.last_pose ? { ...xyFromMat4(cam.last_pose), yaw: yawFromMat4(cam.last_pose) } : null

  return (
    <Card className="py-4 gap-3" data-testid={`apriltag-cam-${cam.camera_id}`}>
      <CardHeader className="px-4">
        <CardTitle className="flex items-center justify-between gap-2 text-sm">
          <span className="flex items-center gap-2">
            <StatusDot tone={cam.running ? 'good' : 'bad'} />
            {cam.name}
          </span>
          {cam.reason ? (
            <Badge variant="outline" className="border-warning/50 font-mono text-[11px] text-warning">
              {cam.reason}
            </Badge>
          ) : (
            cam.running && (
              <Badge variant="outline" className="border-success/50 text-[11px] text-success">
                publishing
              </Badge>
            )
          )}
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-4 px-4">
        {!cam.running ? (
          <p className="text-sm text-muted-foreground">
            Detector is not running{cam.reason ? ` — ${cam.reason}` : ''}.
          </p>
        ) : (
          <>
            <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
              <Metric label="Detections" value={`${cam.det_per_s.toFixed(1)} /s`} />
              <Metric
                label="Latency"
                value={
                  <>
                    {formatMs(cam.last_latency_ms)}{' '}
                    <span className="text-xs font-normal text-muted-foreground">
                      ewma {formatMs(cam.latency_ewma_ms)}
                    </span>
                  </>
                }
              />
              <Metric label="Mean reproj" value={`${cam.mean_reproj_err_px.toFixed(2)} px`} />
              <Metric label="Published" value={formatCount(cam.published)} />
            </div>

            <SkipBreakdown cam={cam} />

            <div>
              <p className="mb-1.5 text-xs font-medium tracking-wide text-muted-foreground uppercase">
                Last tags
              </p>
              {cam.last_tags.length > 0 ? (
                <div className="flex flex-wrap gap-1">
                  {cam.last_tags.map((t) => (
                    <Badge key={t.id} variant="secondary" className="font-mono text-[11px]">
                      #{t.id}
                      {t.range_m != null ? ` · ${formatMeters(t.range_m, 1)}` : ''} · dm{' '}
                      {t.decision_margin.toFixed(0)}
                    </Badge>
                  ))}
                </div>
              ) : (
                <p className="text-xs text-muted-foreground">none</p>
              )}
            </div>

            <div data-testid={`apriltag-pose-${cam.camera_id}`}>
              <p className="mb-1.5 text-xs font-medium tracking-wide text-muted-foreground uppercase">
                Last field pose
              </p>
              {pose ? (
                <div className="flex flex-wrap gap-x-4 gap-y-1 font-mono text-xs tabular-nums">
                  <span>x {formatMeters(pose.x)}</span>
                  <span>y {formatMeters(pose.y)}</span>
                  <span>θ {formatDeg(pose.yaw)}</span>
                  <span className="text-muted-foreground">{formatAgeMs(poseAge)}</span>
                </div>
              ) : (
                <p className="text-xs text-muted-foreground">no pose published yet</p>
              )}
            </div>

            <p className="font-mono text-xs tabular-nums text-muted-foreground">
              {formatCount(cam.frames_seen)} frames · {formatCount(cam.detections_total)}{' '}
              detections total
            </p>
          </>
        )}
      </CardContent>
    </Card>
  )
}

export default function ApriltagPage() {
  const status = useAprilTagStatus()
  const s = status.data
  const cams = s?.cameras ?? []
  const running = cams.filter((c) => c.running).length

  return (
    <div>
      <PageHeader
        title="AprilTag"
        description="Per-camera detection diagnostics — rates, latency, and why poses aren't publishing."
      />

      <div className="grid grid-cols-2 gap-3 md:grid-cols-3">
        <StatCard
          label="Field layout"
          value={s ? (s.active_layout_name ?? 'none') : '—'}
          sub={s?.active_layout_name ? 'active' : 'activate one on the Field page'}
          tone={s ? (s.active_layout_name ? 'good' : 'bad') : 'default'}
          testId="gate-layout"
        />
        <StatCard
          label="T_robot_imu"
          value={s ? (s.t_robot_imu_set ? 'set' : 'not set') : '—'}
          sub={
            s && !s.t_robot_imu_set ? (
              <Link to="/robot" className="text-primary hover:underline">
                set on the Robot page →
              </Link>
            ) : (
              'IMU pose in the robot frame'
            )
          }
          tone={s ? (s.t_robot_imu_set ? 'good' : 'bad') : 'default'}
          testId="gate-t-robot-imu"
        />
        <StatCard
          label="Detectors"
          value={cams.length ? `${running}/${cams.length}` : '—'}
          sub="running"
          tone={cams.length === 0 ? 'default' : running === cams.length ? 'good' : 'warn'}
          testId="gate-detectors"
        />
      </div>

      <div className="mt-4 grid grid-cols-1 gap-4 xl:grid-cols-2">
        {cams.map((cam) => (
          <CameraCard key={cam.camera_id} cam={cam} />
        ))}
      </div>
      {s && cams.length === 0 && (
        <EmptyState
          icon={Camera}
          title="No AprilTag cameras"
          description="Assign the 'apriltag' role to a camera on the Cameras page to start detection."
        />
      )}
    </div>
  )
}
