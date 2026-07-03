import { useState } from 'react'
import { toast } from 'sonner'
import { ChevronDown, RotateCcw } from 'lucide-react'
import type { FusionStatus, LatencyStat } from '@/api/fusion'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from '@/components/ui/collapsible'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { ConfigForm } from '@/components/config/ConfigForm'
import { ConfirmButton } from '@/components/ConfirmButton'
import { Mat4Table } from '@/components/Mat4Table'
import { PageHeader } from '@/components/PageHeader'
import { RollingChart } from '@/components/RollingChart'
import { StatCard, type StatTone } from '@/components/StatCard'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatCount, formatDeg, formatHz, formatMeters, formatMs } from '@/lib/format'
import {
  useFusionConfig,
  useFusionStatus,
  useResetFusion,
  useUpdateFusionConfig,
} from '@/queries/fusion'

const STAGES: Array<{ key: keyof FusionStatus['latency']; label: string }> = [
  { key: 'tag_pulse_to_fusion', label: 'tag pulse → fusion' },
  { key: 'queue_wait', label: 'queue wait' },
  { key: 'solve', label: 'solve' },
  { key: 'pose_staleness', label: 'pose staleness' },
]

function stalenessTone(p95: number | undefined): StatTone {
  if (p95 == null) return 'default'
  return p95 < 50 ? 'good' : p95 < 100 ? 'warn' : 'bad'
}

// state.quality is the headline confidence byte (255 = fully converged).
function qualityTone(q: number | undefined): StatTone {
  if (q == null) return 'default'
  return q >= 200 ? 'good' : q >= 100 ? 'warn' : 'bad'
}

// p50-ish (last) + p95 horizontal bars per pipeline stage; pose_staleness is
// the trigger-pulse → pose-on-CAN headline (<50 ms target).
function LatencyStages({ latency }: { latency: FusionStatus['latency'] }) {
  const max = Math.max(10, ...STAGES.map((s) => (latency[s.key] as LatencyStat).p95_ms))
  return (
    <div className="space-y-2">
      {STAGES.map(({ key, label }) => {
        const v = latency[key]
        return (
          <div key={key} className="space-y-0.5">
            <div className="flex justify-between font-mono text-xs tabular-nums">
              <span className="text-muted-foreground">{label}</span>
              <span>
                {formatMs(v.last_ms)} <span className="text-muted-foreground">· p95 {formatMs(v.p95_ms)}</span>
              </span>
            </div>
            <div className="relative h-2 overflow-hidden rounded-sm bg-muted">
              <div
                className="absolute inset-y-0 left-0 rounded-sm bg-chart-2/40"
                style={{ width: `${Math.min(100, (v.p95_ms / max) * 100)}%` }}
              />
              <div
                className="absolute inset-y-0 left-0 rounded-sm bg-chart-2"
                style={{ width: `${Math.min(100, (v.last_ms / max) * 100)}%` }}
              />
            </div>
          </div>
        )
      })}
    </div>
  )
}

function SourceRow({
  name,
  rate,
  ageMs,
  busDropped,
  detail,
}: {
  name: string
  rate: number | undefined
  ageMs: number | null | undefined
  busDropped: number
  detail: string
}) {
  return (
    <TableRow>
      <TableCell className="font-medium">{name}</TableCell>
      <TableCell className="font-mono text-xs tabular-nums">{formatHz(rate)}</TableCell>
      <TableCell className="font-mono text-xs tabular-nums">
        {ageMs != null ? formatMs(ageMs, 0) : '—'}
      </TableCell>
      <TableCell
        className={`font-mono text-xs tabular-nums ${busDropped > 0 ? 'text-warning' : 'text-muted-foreground'}`}
      >
        {formatCount(busDropped)}
      </TableCell>
      <TableCell className="font-mono text-xs tabular-nums text-muted-foreground">{detail}</TableCell>
    </TableRow>
  )
}

export default function FusionPage() {
  const status = useFusionStatus()
  const config = useFusionConfig()
  const update = useUpdateFusionConfig()
  const reset = useResetFusion()
  const [tOpen, setTOpen] = useState(false)

  const s = status.data
  const staleness = s?.latency.pose_staleness.p95_ms
  const series = useTimeSeries(
    'fusion-page',
    { staleness_p95: staleness, solve: s?.solve_ms.last },
    status.dataUpdatedAt,
  )

  return (
    <div>
      <PageHeader
        title="Fusion"
        description="GTSAM fixed-lag smoother fusing AprilTags, VIO and chassis odometry."
        actions={
          <ConfirmButton
            variant="outline"
            size="sm"
            title="Reset fusion?"
            description="Drops the smoother state; the pose re-initializes from the next AprilTag observations."
            confirmLabel="Reset"
            onConfirm={() =>
              reset.mutate(undefined, { onSuccess: () => toast.success('Fusion reset') })
            }
          >
            <RotateCcw /> Reset
          </ConfirmButton>
        }
      />

      {s && !s.enabled && (
        <div className="mb-4 rounded-lg border border-warning/40 bg-warning/10 px-4 py-3 text-sm text-warning">
          Fusion disabled: {s.reason}
        </div>
      )}
      {s?.collision_mode && (
        <div className="mb-4 rounded-lg border border-destructive/40 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          Collision mode — tag gate opened, VIO/odom noise inflated until tags re-converge.
        </div>
      )}

      <div className="grid grid-cols-2 gap-3 md:grid-cols-4 xl:grid-cols-7">
        <StatCard
          label="Mode"
          value={
            <Badge variant="outline" className="font-mono">
              {s?.mode || '—'}
            </Badge>
          }
          sub={s?.initialized ? 'initialized' : 'waiting for tags'}
        />
        <StatCard
          label="Quality"
          value={s?.quality ?? '—'}
          sub="confidence 0–255"
          tone={qualityTone(s?.quality)}
          testId="stat-quality"
        />
        <StatCard
          label="Pose"
          value={
            s?.pose ? (
              <span className="text-base">
                {formatMeters(s.pose.x_m)} · {formatMeters(s.pose.y_m)} · {formatDeg(s.pose.theta_rad, 0)}
              </span>
            ) : (
              '—'
            )
          }
          testId="stat-pose"
        />
        <StatCard
          label="Staleness p95"
          value={formatMs(staleness, 0)}
          sub="target < 50 ms"
          tone={stalenessTone(staleness)}
          testId="stat-staleness"
        />
        <StatCard label="Solve" value={formatMs(s?.solve_ms.last)} sub={`p95 ${formatMs(s?.solve_ms.p95)}`} />
        <StatCard
          label="Lag window"
          value={s ? `${s.lag.states}` : '—'}
          sub={s ? `states · ${s.lag.lag_s.toFixed(1)} s` : undefined}
        />
        <StatCard
          label="Teensy-now"
          value={s ? (s.teensy_now.healthy ? 'healthy' : 'unwarmed') : '—'}
          sub={s ? `offset ${s.teensy_now.offset_ms.toFixed(1)} ms` : undefined}
          tone={s?.teensy_now.healthy ? 'good' : 'warn'}
        />
      </div>

      <div className="mt-4 grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
                Latency stages
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4 px-4">
              {s && <LatencyStages latency={s.latency} />}
              <RollingChart
                points={series}
                series={[
                  { key: 'staleness_p95', label: 'staleness p95 (ms)' },
                  { key: 'solve', label: 'solve (ms)' },
                ]}
                height={120}
              />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Sources</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Source</TableHead>
                    <TableHead>Rate</TableHead>
                    <TableHead>Age</TableHead>
                    <TableHead>Bus drops</TableHead>
                    <TableHead>Counters</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {s && (
                    <>
                      <SourceRow
                        name="Tags"
                        rate={s.sources.tag.rate_hz}
                        ageMs={s.sources.tag.last_age_ms}
                        busDropped={s.sources.tag.bus_dropped}
                        detail={`${formatCount(s.sources.tag.accepted)} ok · ${formatCount(
                          s.sources.tag.rejected_gate,
                        )} gated · ${formatCount(s.sources.tag.rejected_stale + s.sources.tag.rejected_clock)} dropped`}
                      />
                      <SourceRow
                        name="VIO"
                        rate={s.sources.vio.rate_hz}
                        ageMs={s.sources.vio.last_age_ms}
                        busDropped={s.sources.vio.bus_dropped}
                        detail={
                          s.sources.vio.enabled
                            ? `${formatCount(s.sources.vio.fused_intervals)} fused · ${formatCount(
                                s.sources.vio.skipped_epoch + s.sources.vio.skipped_unhealthy,
                              )} skipped`
                            : (s.sources.vio.reason ?? 'disabled')
                        }
                      />
                      <SourceRow
                        name="Odom"
                        rate={s.sources.odom.rate_hz}
                        ageMs={s.sources.odom.last_age_ms}
                        busDropped={s.sources.odom.bus_dropped}
                        detail={`${formatCount(s.sources.odom.fused_intervals)} fused · ${formatCount(
                          s.sources.odom.slip,
                        )} slip · ${formatCount(s.sources.odom.stale)} stale`}
                      />
                    </>
                  )}
                </TableBody>
              </Table>
              {s && (
                <div className="mt-3 grid grid-cols-2 gap-x-6 gap-y-1 font-mono text-xs tabular-nums text-muted-foreground">
                  <span>poses sent: {formatCount(s.output.sent)}</span>
                  <span>send errors: {formatCount(s.output.send_errors)}</span>
                  <span>queue dropped: {formatCount(s.output.queue_dropped)}</span>
                  <span>bridge factors: {formatCount(s.output.bridge_factors)}</span>
                  <span>gate reopens: {formatCount(s.output.gate_reopens)}</span>
                  <span>solver throws: {formatCount(s.output.update_exceptions)}</span>
                  <span>reinits: {formatCount(s.reinits)}</span>
                </div>
              )}
              {s?.T_field_robot && (
                <Collapsible open={tOpen} onOpenChange={setTOpen} className="mt-3">
                  <CollapsibleTrigger className="flex items-center gap-1 text-xs text-muted-foreground hover:text-foreground">
                    <ChevronDown className={`size-3 transition-transform ${tOpen ? 'rotate-180' : ''}`} />
                    T_field_robot
                  </CollapsibleTrigger>
                  <CollapsibleContent className="pt-2">
                    <Mat4Table matrix={s.T_field_robot} />
                  </CollapsibleContent>
                </Collapsible>
              )}
            </CardContent>
          </Card>
        </div>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="text-sm">Configuration</CardTitle>
          </CardHeader>
          <CardContent className="px-4">
            <ConfigForm
              value={
                config.data
                  ? Object.fromEntries(
                      Object.entries(config.data).filter(([k]) => k !== 'updated_at'),
                    ) as Record<string, number | boolean>
                  : undefined
              }
              version={config.data?.updated_at}
              saving={update.isPending}
              onSave={(patch) =>
                update.mutate(patch, {
                  onSuccess: (r) =>
                    toast.success(r.restarted ? 'Saved — engine rebuilt' : 'Saved (live-applied)'),
                })
              }
              groups={[
                {
                  fields: [{ kind: 'switch', key: 'enabled', label: 'Enabled' }],
                },
                {
                  title: 'Timing',
                  fields: [
                    { kind: 'number', key: 'lag_s', label: 'Smoother lag', unit: 's', min: 0.1 },
                    { kind: 'number', key: 'min_state_dt_ms', label: 'Min state spacing', unit: 'ms', int: true, min: 1 },
                    { kind: 'number', key: 'output_hz', label: 'Output rate', unit: 'Hz', int: true, min: 1, help: 'live-applied' },
                    { kind: 'number', key: 'max_extrapolation_ms', label: 'Max extrapolation', unit: 'ms', int: true, min: 0, help: 'live-applied' },
                  ],
                },
                {
                  title: 'Tag gating',
                  fields: [
                    { kind: 'number', key: 'tag_gate_chi2', label: 'χ² gate', min: 0 },
                    { kind: 'number', key: 'tag_huber_k', label: 'Huber k', min: 0 },
                  ],
                },
                {
                  title: 'VIO noise',
                  fields: [
                    { kind: 'number', key: 'vio_sigma_rot', label: 'σ rotation', unit: 'rad', min: 0 },
                    { kind: 'number', key: 'vio_sigma_trans', label: 'σ translation', unit: 'm', min: 0 },
                    { kind: 'number', key: 'vio_huber_k', label: 'Huber k', min: 0 },
                  ],
                },
                {
                  title: 'Odometry noise',
                  fields: [
                    { kind: 'number', key: 'odom_sigma_vx', label: 'σ vx', unit: 'm/s', min: 0 },
                    { kind: 'number', key: 'odom_sigma_vy', label: 'σ vy', unit: 'm/s', min: 0 },
                    { kind: 'number', key: 'odom_sigma_omega', label: 'σ ω', unit: 'rad/s', min: 0 },
                    { kind: 'number', key: 'odom_cauchy_k', label: 'Cauchy k', min: 0 },
                  ],
                },
                {
                  title: 'Collision & reinit',
                  fields: [
                    { kind: 'number', key: 'collision_inflation', label: 'Collision inflation', min: 1 },
                    { kind: 'number', key: 'collision_window', label: 'Collision window', int: true, min: 1 },
                    { kind: 'number', key: 'reinit_pos_std_m', label: 'Reinit pos σ', unit: 'm', min: 0 },
                  ],
                },
              ]}
            />
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
