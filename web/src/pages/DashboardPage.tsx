import { useMemo } from 'react'
import { Link } from 'react-router-dom'
import { Activity, AlertTriangle, Camera, Clock, Cpu, GitMerge, Route } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { PageHeader } from '@/components/PageHeader'
import { RollingChart } from '@/components/RollingChart'
import { StatCard, type StatTone } from '@/components/StatCard'
import { FieldView } from '@/components/field/FieldView'
import { parseLayoutText } from '@/components/field/fieldGeometry'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatHz, formatMs, formatUptime } from '@/lib/format'
import { useAprilTagStatus } from '@/queries/apriltag'
import { useRobotStatus } from '@/queries/robot'
import { useFieldLayout, useFieldLayouts } from '@/queries/fieldLayouts'
import { useFusionStatus } from '@/queries/fusion'
import { useImuStatus } from '@/queries/imu'
import { useStatus } from '@/queries/status'
import { useVioStatus } from '@/queries/vio'

function stalenessTone(p95Ms: number | undefined): StatTone {
  if (p95Ms == null) return 'default'
  if (p95Ms < 50) return 'good'
  if (p95Ms < 100) return 'warn'
  return 'bad'
}

function ChartCard({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <Card className="py-4">
      <CardHeader className="px-4">
        <CardTitle className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
          {title}
        </CardTitle>
      </CardHeader>
      <CardContent className="px-2">{children}</CardContent>
    </Card>
  )
}

export default function DashboardPage() {
  const status = useStatus()
  const fusion = useFusionStatus()
  const vio = useVioStatus()
  const apriltag = useAprilTagStatus()
  const robot = useRobotStatus()
  const imu = useImuStatus()

  const layouts = useFieldLayouts()
  const activeLayoutId = layouts.data?.find((l) => l.active)?.id ?? null
  const layout = useFieldLayout(activeLayoutId)
  const fieldModel = useMemo(
    () => (layout.data ? parseLayoutText(layout.data.json) : null),
    [layout.data],
  )

  const cams = status.data?.cameras ?? []
  const online = cams.filter((c) => c.online).length
  const aggFps = cams.reduce((sum, c) => sum + (c.online ? c.fps_1s : 0), 0)

  const staleness = fusion.data?.latency.pose_staleness.p95_ms

  // Rolling chart buffers (2-minute window, fed by the poll ticks).
  const fpsSeries = useTimeSeries(
    'dash-fps',
    Object.fromEntries(cams.map((c) => [`cam${c.id}`, c.online ? c.fps_1s : null])),
    status.dataUpdatedAt,
  )
  const fusionSeries = useTimeSeries(
    'dash-fusion',
    {
      staleness_p95: fusion.data?.latency.pose_staleness.p95_ms,
      solve: fusion.data?.solve_ms.last,
    },
    fusion.dataUpdatedAt,
  )
  const tagCams = apriltag.data?.cameras ?? []
  const detSeries = useTimeSeries(
    'dash-det',
    Object.fromEntries(tagCams.map((c) => [`cam${c.camera_id}`, c.running ? c.det_per_s : null])),
    apriltag.dataUpdatedAt,
  )
  const vioSeries = useTimeSeries(
    'dash-vio',
    { freq: vio.data?.freq_hz, features: vio.data?.tracked_features },
    vio.dataUpdatedAt,
  )

  // Derived warnings.
  const alerts: string[] = []
  for (const c of cams.filter((c) => !c.online)) alerts.push(`Camera "${c.name}" is offline`)
  if (imu.data && !imu.data.controller_connected) alerts.push('sync controller is not connected')
  if (robot.data && !robot.data.running) alerts.push('Robot link is down')
  if (imu.data?.controller_connected && !imu.data.imu_ok) alerts.push('IMU is unhealthy')
  if (fusion.data?.enabled && fusion.data.initialized && !fusion.data.pose)
    alerts.push('Fusion is initialized but publishing no pose')
  if (fusion.data?.collision_mode) alerts.push('Fusion is in collision mode (tag-gate fallback)')

  return (
    <div>
      <PageHeader title="Dashboard" description="System health at a glance." />

      {alerts.length > 0 && (
        <div
          className="mb-4 flex flex-col gap-1 rounded-lg border border-warning/40 bg-warning/10 px-4 py-3"
          data-testid="alerts-strip"
        >
          {alerts.map((a) => (
            <div key={a} className="flex items-center gap-2 text-sm text-warning">
              <AlertTriangle className="size-4 shrink-0" />
              {a}
            </div>
          ))}
        </div>
      )}

      <div className="grid grid-cols-2 gap-3 md:grid-cols-3 xl:grid-cols-6">
        <StatCard
          label="Uptime"
          icon={Clock}
          value={formatUptime(status.data?.uptime_s)}
          testId="stat-uptime"
        />
        <StatCard
          label="Cameras"
          icon={Camera}
          value={cams.length ? `${online}/${cams.length}` : '—'}
          sub={online > 0 ? `${aggFps.toFixed(0)} fps total` : 'none online'}
          tone={cams.length === 0 ? 'default' : online === cams.length ? 'good' : 'warn'}
          testId="stat-cameras"
        />
        <StatCard
          label="Fusion"
          icon={GitMerge}
          value={formatMs(staleness, 0)}
          sub={fusion.data ? `mode: ${fusion.data.mode || '—'} · staleness p95` : 'status unavailable'}
          tone={stalenessTone(staleness)}
          testId="stat-fusion"
        />
        <StatCard
          label="sync controller"
          icon={Cpu}
          value={imu.data ? (imu.data.controller_connected ? 'online' : 'offline') : '—'}
          sub={
            imu.data?.controller_connected && robot.data
              ? `clock sync ${robot.data.clock_sync.healthy ? 'healthy' : 'unhealthy'}`
              : undefined
          }
          tone={imu.data ? (imu.data.controller_connected ? 'good' : 'bad') : 'default'}
          testId="stat-controller"
        />
        <StatCard
          label="VIO"
          icon={Route}
          value={vio.data ? vio.data.phase || (vio.data.enabled ? '—' : 'off') : '—'}
          sub={vio.data?.running ? `${vio.data.tracked_features} features` : vio.data?.reason}
          tone={vio.data?.running ? 'good' : 'default'}
          testId="stat-vio"
        />
        <StatCard
          label="IMU"
          icon={Activity}
          value={formatHz(imu.data?.rate_hz, 0)}
          sub={imu.data ? (imu.data.imu_ok ? 'healthy' : 'unhealthy') : undefined}
          tone={imu.data ? (imu.data.imu_ok ? 'good' : 'bad') : 'default'}
          testId="stat-imu"
        />
      </div>

      <div className="mt-4 grid grid-cols-1 gap-3 lg:grid-cols-2">
        <ChartCard title="Camera FPS">
          <RollingChart
            points={fpsSeries}
            series={cams.map((c) => ({ key: `cam${c.id}`, label: c.name }))}
          />
        </ChartCard>
        <ChartCard title="Fusion latency (ms)">
          <RollingChart
            points={fusionSeries}
            series={[
              { key: 'staleness_p95', label: 'pose staleness p95' },
              { key: 'solve', label: 'solve' },
            ]}
          />
        </ChartCard>
        <ChartCard title="AprilTag detections / s">
          <RollingChart
            points={detSeries}
            series={tagCams.map((c) => ({ key: `cam${c.camera_id}`, label: c.name }))}
          />
        </ChartCard>
        <ChartCard title="VIO">
          <RollingChart
            points={vioSeries}
            series={[
              { key: 'freq', label: 'rate (Hz)' },
              { key: 'features', label: 'tracked features' },
            ]}
          />
        </ChartCard>
      </div>

      {fieldModel && (
        <Card className="mt-4 py-4">
          <CardHeader className="px-4">
            <CardTitle className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
              <Link to="/field" className="hover:text-foreground">
                Field — live pose →
              </Link>
            </CardTitle>
          </CardHeader>
          <CardContent className="px-4">
            <Link to="/field" className="block max-w-3xl">
              <FieldView
                model={fieldModel}
                pose={
                  fusion.data?.pose
                    ? {
                        x: fusion.data.pose.x_m,
                        y: fusion.data.pose.y_m,
                        thetaRad: fusion.data.pose.theta_rad,
                      }
                    : null
                }
                poseStale={(staleness ?? 0) > 100}
                showIds={false}
              />
            </Link>
          </CardContent>
        </Card>
      )}
    </div>
  )
}
