import { toast } from 'sonner'
import { RefreshCcw } from 'lucide-react'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
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
import { PageHeader } from '@/components/PageHeader'
import { RollingChart } from '@/components/RollingChart'
import { StatCard } from '@/components/StatCard'
import { StatusDot } from '@/components/StatusDot'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatCount, formatHz, formatMeters } from '@/lib/format'
import { useRestartVio, useUpdateVioConfig, useVioConfig, useVioStatus } from '@/queries/vio'

function phaseBadge(phase: string, running: boolean): string {
  if (!running) return 'border-muted-foreground/50 text-muted-foreground'
  if (/track/i.test(phase)) return 'border-success/50 text-success'
  if (/init/i.test(phase)) return 'border-chart-2/60 text-chart-2'
  return 'border-warning/50 text-warning'
}

export default function VioPage() {
  const status = useVioStatus()
  const config = useVioConfig()
  const update = useUpdateVioConfig()
  const restart = useRestartVio()

  const s = status.data
  const series = useTimeSeries(
    'vio-page',
    { freq: s?.freq_hz, features: s?.tracked_features },
    status.dataUpdatedAt,
  )

  return (
    <div>
      <PageHeader
        title="VIO"
        description="OpenVINS stereo+IMU odometry on the vio_left / vio_right pair."
        actions={
          <ConfirmButton
            variant="outline"
            size="sm"
            title="Restart VIO?"
            description="Forces a reinitialization — the odometry origin resets and the epoch increments."
            confirmLabel="Restart"
            onConfirm={() =>
              restart.mutate(undefined, {
                onSuccess: (r) => toast.success(`VIO restarted (epoch ${r.epoch})`),
              })
            }
          >
            <RefreshCcw /> Restart
          </ConfirmButton>
        }
      />

      {s && !s.enabled && (
        <div className="mb-4 rounded-lg border border-warning/40 bg-warning/10 px-4 py-3 text-sm text-warning">
          VIO disabled: {s.reason}
        </div>
      )}

      <div className="grid grid-cols-2 gap-3 md:grid-cols-4 xl:grid-cols-6">
        <StatCard
          label="Phase"
          value={
            <Badge variant="outline" className={phaseBadge(s?.phase ?? '', s?.running ?? false)}>
              {s?.phase || (s?.enabled ? '—' : 'off')}
            </Badge>
          }
          sub={s?.initialized ? 'initialized' : 'not initialized'}
        />
        <StatCard label="Rate" value={formatHz(s?.freq_hz)} sub="stereo pairs" />
        <StatCard
          label="Features"
          value={formatCount(s?.tracked_features)}
          tone={s && s.running && s.tracked_features < 20 ? 'warn' : 'default'}
        />
        <StatCard label="Epoch" value={s?.epoch ?? '—'} sub={`${s?.reinits ?? 0} reinits`} />
        <StatCard
          label="Pos σ"
          value={formatMeters(s?.cov_pos_std_m)}
          tone={s && s.cov_pos_std_m > 0.5 ? 'warn' : 'default'}
        />
        <StatCard label="IMU rate" value={formatHz(s?.imu_rate_hz, 0)} />
      </div>

      <div className="mt-4 grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
                Rate &amp; features
              </CardTitle>
            </CardHeader>
            <CardContent className="px-2">
              <RollingChart
                points={series}
                series={[
                  { key: 'freq', label: 'rate (Hz)' },
                  { key: 'features', label: 'features' },
                ]}
              />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Cameras &amp; pairing</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Camera</TableHead>
                    <TableHead>Role</TableHead>
                    <TableHead>Feeder</TableHead>
                    <TableHead>Reproj σ</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {(s?.cameras ?? []).map((c) => (
                    <TableRow key={c.camera_id}>
                      <TableCell>{c.name}</TableCell>
                      <TableCell>
                        <Badge variant="secondary" className="font-mono text-[11px]">
                          {c.role}
                        </Badge>
                      </TableCell>
                      <TableCell>
                        <span className="flex items-center gap-1.5 text-xs">
                          <StatusDot tone={c.feeder_running ? 'good' : 'bad'} />
                          {c.feeder_running ? 'running' : 'stopped'}
                        </span>
                      </TableCell>
                      <TableCell className="font-mono text-xs tabular-nums">
                        {c.reproj_std_px != null ? `${c.reproj_std_px.toFixed(2)} px` : '—'}
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
              {s && (
                <div className="mt-3 grid grid-cols-2 gap-x-6 gap-y-1 font-mono text-xs tabular-nums text-muted-foreground">
                  <span>paired: {formatCount(s.counters.paired)}</span>
                  <span>frames fed: {formatCount(s.counters.frames_fed)}</span>
                  <span>unmatched: {formatCount(s.counters.dropped_unmatched)}</span>
                  <span>imu fed: {formatCount(s.counters.imu_fed)}</span>
                  <span>zero-ts drops: {formatCount(s.counters.dropped_zero_ts)}</span>
                  <span>queue drops: {formatCount(s.counters.dropped_pair_queue)}</span>
                </div>
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
                  ? {
                      enabled: config.data.enabled,
                      num_pts: config.data.num_pts,
                      fast_threshold: config.data.fast_threshold,
                      downsample: config.data.downsample,
                      max_reproj_std_px: config.data.max_reproj_std_px,
                      auto_reinit: config.data.auto_reinit,
                      reinit_min_features: config.data.reinit_min_features,
                      reinit_window_frames: config.data.reinit_window_frames,
                      reinit_max_pos_std_m: config.data.reinit_max_pos_std_m,
                    }
                  : undefined
              }
              version={config.data?.updated_at}
              saving={update.isPending}
              onSave={(patch) =>
                update.mutate(patch, {
                  onSuccess: (r) =>
                    toast.success(r.restarted ? 'Saved — VIO restarted' : 'Saved'),
                })
              }
              groups={[
                {
                  fields: [
                    { kind: 'switch', key: 'enabled', label: 'Enabled' },
                    { kind: 'switch', key: 'downsample', label: 'Downsample', help: 'Halve resolution before tracking' },
                  ],
                },
                {
                  title: 'Tracking',
                  fields: [
                    { kind: 'number', key: 'num_pts', label: 'Feature count', int: true, min: 1 },
                    { kind: 'number', key: 'fast_threshold', label: 'FAST threshold', int: true, min: 1 },
                    {
                      kind: 'number',
                      key: 'max_reproj_std_px',
                      label: 'Max calibration reproj σ',
                      unit: 'px',
                      min: 0,
                      help: 'Gate: worse calibrations disable VIO',
                    },
                  ],
                },
                {
                  title: 'Auto-reinit',
                  fields: [
                    { kind: 'switch', key: 'auto_reinit', label: 'Auto-reinit on divergence' },
                    { kind: 'number', key: 'reinit_min_features', label: 'Min features', int: true, min: 0 },
                    { kind: 'number', key: 'reinit_window_frames', label: 'Window', unit: 'frames', int: true, min: 1 },
                    { kind: 'number', key: 'reinit_max_pos_std_m', label: 'Max pos σ', unit: 'm', min: 0 },
                  ],
                },
              ]}
            />
            <p className="mt-3 text-xs text-muted-foreground">
              Saving an engine-relevant change rebuilds the runner (odometry epoch resets).
            </p>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
