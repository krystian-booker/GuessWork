import { useState } from 'react'
import { Link } from 'react-router-dom'
import { toast } from 'sonner'
import { Send } from 'lucide-react'
import type { ClockSyncHop, RobotConfigPatch } from '@/api/robot'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { Textarea } from '@/components/ui/textarea'
import { ConfigForm } from '@/components/config/ConfigForm'
import { ImuAttitudeViewer } from '@/components/imu/ImuAttitudeViewer'
import { Mat4Table } from '@/components/Mat4Table'
import { PageHeader } from '@/components/PageHeader'
import { RollingChart } from '@/components/RollingChart'
import { StatusDot } from '@/components/StatusDot'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatCount, formatHz, formatMs } from '@/lib/format'
import { isMat4 } from '@/lib/matrices'
import { useBenchPose, useRobotConfig, useRobotStatus, useUpdateRobotConfig } from '@/queries/robot'
import { useImuConfig, useImuStatus, useUpdateImuConfig } from '@/queries/imu'

function KV({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-baseline justify-between gap-4 text-sm">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right font-mono text-xs tabular-nums">{value}</span>
    </div>
  )
}

// One hop of the rio↔host↔teensy timestamp chain.
function SyncHopRow({ label, hop }: { label: string; hop: ClockSyncHop | undefined }) {
  return (
    <div className="rounded-md border px-3 py-2">
      <div className="flex items-center justify-between">
        <span className="flex items-center gap-2 text-sm font-medium">
          <StatusDot tone={hop ? (hop.healthy ? 'good' : 'warn') : 'idle'} />
          {label}
        </span>
        <span className={`text-xs ${hop?.healthy ? 'text-success' : 'text-warning'}`}>
          {hop ? (hop.healthy ? 'healthy' : 'warming up') : '—'}
        </span>
      </div>
      {hop && (
        <div className="mt-1.5 grid grid-cols-2 gap-x-6 gap-y-1 font-mono text-xs tabular-nums text-muted-foreground">
          <span>offset {formatCount(Math.round(hop.offset_us))} µs</span>
          <span>drift {hop.drift_ppm.toFixed(1)} ppm</span>
          <span>{formatCount(hop.samples)} samples</span>
          <span>{formatCount(hop.resets)} resets</span>
        </div>
      )}
    </div>
  )
}

function BenchPoseSender({ disabled }: { disabled: boolean }) {
  const bench = useBenchPose()
  const [x, setX] = useState('0')
  const [y, setY] = useState('0')
  const [thetaDeg, setThetaDeg] = useState('0')

  const valid = [x, y, thetaDeg].every((v) => v.trim() !== '' && Number.isFinite(Number(v)))

  return (
    <div className="space-y-3">
      <p className="text-xs text-muted-foreground">
        Bench downlink: hand-craft a pose packet to verify controller-side reception.
      </p>
      <div className="grid grid-cols-3 gap-2">
        <div className="space-y-1">
          <Label htmlFor="bp-x" className="text-xs">
            x (m)
          </Label>
          <Input id="bp-x" value={x} onChange={(e) => setX(e.target.value)} className="font-mono text-xs" />
        </div>
        <div className="space-y-1">
          <Label htmlFor="bp-y" className="text-xs">
            y (m)
          </Label>
          <Input id="bp-y" value={y} onChange={(e) => setY(e.target.value)} className="font-mono text-xs" />
        </div>
        <div className="space-y-1">
          <Label htmlFor="bp-t" className="text-xs">
            θ (deg)
          </Label>
          <Input id="bp-t" value={thetaDeg} onChange={(e) => setThetaDeg(e.target.value)} className="font-mono text-xs" />
        </div>
      </div>
      <Button
        size="sm"
        variant="secondary"
        disabled={disabled || !valid || bench.isPending}
        onClick={() =>
          bench.mutate(
            { x: Number(x), y: Number(y), theta: (Number(thetaDeg) * Math.PI) / 180 },
            { onSuccess: () => toast.success('Pose sent') },
          )
        }
      >
        <Send /> Send pose
      </Button>
    </div>
  )
}

// t_imu_robot editor: the stored schema is {"T_robot_imu": [[4x4 row-major]]}.
function TRobotImuEditor() {
  const config = useImuConfig()
  const update = useUpdateImuConfig()
  const [editing, setEditing] = useState(false)
  const [text, setText] = useState('')

  const current = config.data?.t_imu_robot ?? null

  let parsed: number[][] | null = null
  let parseError: string | null = null
  if (editing && text.trim()) {
    try {
      const v = JSON.parse(text)
      if (isMat4(v)) parsed = v
      else parseError = 'expected a 4×4 number array [[r0],[r1],[r2],[r3]]'
    } catch {
      parseError = 'invalid JSON'
    }
  }

  return (
    <div className="space-y-3">
      <p className="text-xs text-muted-foreground">
        T_robot_imu — IMU pose in the robot frame. Unset disables AprilTag publishing and fusion
        VIO ingestion.
      </p>
      {current ? (
        <Mat4Table matrix={current.T_robot_imu} />
      ) : (
        <Badge variant="outline" className="border-warning/50 text-warning">
          not set
        </Badge>
      )}
      {!editing ? (
        <div className="flex gap-2">
          <Button
            size="sm"
            variant="secondary"
            onClick={() => {
              setText(current ? JSON.stringify(current.T_robot_imu) : '')
              setEditing(true)
            }}
          >
            {current ? 'Edit' : 'Set transform'}
          </Button>
          {current && (
            <Button
              size="sm"
              variant="outline"
              onClick={() =>
                update.mutate(
                  { t_imu_robot: null },
                  { onSuccess: () => toast.success('T_robot_imu cleared') },
                )
              }
            >
              Clear
            </Button>
          )}
        </div>
      ) : (
        <div className="space-y-2">
          <Textarea
            value={text}
            onChange={(e) => setText(e.target.value)}
            placeholder="[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]"
            className="min-h-24 font-mono text-xs"
          />
          {parseError && <p className="text-xs text-destructive">{parseError}</p>}
          <div className="flex gap-2">
            <Button
              size="sm"
              disabled={!parsed || update.isPending}
              onClick={() =>
                update.mutate(
                  { t_imu_robot: { T_robot_imu: parsed! } },
                  {
                    onSuccess: () => {
                      toast.success('T_robot_imu updated')
                      setEditing(false)
                    },
                  },
                )
              }
            >
              Save
            </Button>
            <Button size="sm" variant="ghost" onClick={() => setEditing(false)}>
              Cancel
            </Button>
          </div>
        </div>
      )}
    </div>
  )
}

export default function RobotPage() {
  const robotStatus = useRobotStatus()
  const robotConfig = useRobotConfig()
  const updateRobot = useUpdateRobotConfig()
  const imuStatus = useImuStatus()
  const imuConfig = useImuConfig()
  const updateImu = useUpdateImuConfig()

  const s = robotStatus.data
  const cfg = robotConfig.data
  const imu = imuStatus.data

  const odomSeries = useTimeSeries(
    'robot-odom',
    { rate: s?.odom.rate_hz },
    robotStatus.dataUpdatedAt,
  )
  const imuSeries = useTimeSeries('imu-rate', { rate: imu?.rate_hz }, imuStatus.dataUpdatedAt)

  return (
    <div>
      <PageHeader
        title="Robot"
        description="UDP robot link: chassis-speed odometry uplink and pose downlink, plus the onboard IMU."
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        {/* Robot link column */}
        <div className="space-y-4">
          <Card className="py-4 gap-3" data-testid="robot-link-card">
            <CardHeader className="px-4">
              <CardTitle className="flex items-center gap-2 text-sm">
                Robot link
                <StatusDot tone={s ? (s.running ? 'good' : 'bad') : 'idle'} />
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <div className="space-y-1.5">
                <KV label="Bind port" value={s?.bind_port ?? '—'} />
                <KV
                  label="Robot address"
                  value={
                    s?.robot_addr ? (
                      s.robot_addr
                    ) : cfg && cfg.robot_ip !== '' ? (
                      `${cfg.robot_ip} (configured)`
                    ) : (
                      <span className="text-warning">learning from inbound packets…</span>
                    )
                  }
                />
              </div>
              <div className="flex items-baseline gap-4">
                <span className="font-mono text-2xl font-semibold tabular-nums">
                  {formatHz(s?.odom.rate_hz, 0)}
                </span>
                <span className="text-xs text-muted-foreground">
                  {formatCount(s?.odom.packets)} packets · {formatCount(s?.odom.rejected)} rejected
                  · {formatCount(s?.odom.counter_gaps)} counter gaps
                  {s?.odom.last_age_ms != null ? ` · last ${formatMs(s.odom.last_age_ms, 0)} ago` : ''}
                </span>
              </div>
              {s?.odom.last && (
                <div className="grid grid-cols-3 gap-2 font-mono text-xs tabular-nums text-muted-foreground">
                  <span>vx {s.odom.last.vx_mps.toFixed(2)} m/s</span>
                  <span>vy {s.odom.last.vy_mps.toFixed(2)} m/s</span>
                  <span>ω {s.odom.last.omega_radps.toFixed(2)} rad/s</span>
                </div>
              )}
              <RollingChart points={odomSeries} series={[{ key: 'rate', label: 'odom rate (Hz)' }]} height={100} />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3" data-testid="clock-sync-card">
            <CardHeader className="px-4">
              <CardTitle className="flex items-center justify-between gap-2 text-sm">
                Clock sync
                <Badge
                  variant="outline"
                  className={
                    s?.clock_sync.healthy
                      ? 'border-success/50 text-success'
                      : 'border-warning/50 text-warning'
                  }
                >
                  {s ? (s.clock_sync.healthy ? 'chain healthy' : 'chain warming up') : '—'}
                </Badge>
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 px-4">
              <p className="text-xs text-muted-foreground">
                Timestamp chain: RIO sample stamps map to the host clock over UDP, then to the
                Teensy clock over USB. Both hops must be healthy for fused timestamps.
              </p>
              <SyncHopRow label="RIO ↔ host" hop={s?.clock_sync.rio_host} />
              <SyncHopRow label="Host ↔ Teensy" hop={s?.clock_sync.host_teensy} />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3" data-testid="pose-downlink-card">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Pose downlink</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <div className="space-y-1.5">
                <KV label="Poses sent" value={formatCount(s?.pose.sent)} />
                <KV
                  label="Send errors"
                  value={
                    <span className={s && s.pose.send_errors > 0 ? 'text-warning' : undefined}>
                      {formatCount(s?.pose.send_errors)}
                    </span>
                  }
                />
                <KV
                  label="No destination"
                  value={
                    <span className={s && s.pose.no_dest > 0 ? 'text-warning' : undefined}>
                      {formatCount(s?.pose.no_dest)}
                    </span>
                  }
                />
              </div>
              <BenchPoseSender disabled={!s?.running} />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Link configuration</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <ConfigForm
                value={
                  cfg
                    ? {
                        enabled: cfg.enabled,
                        bind_port: cfg.bind_port,
                        robot_port: cfg.robot_port,
                        robot_ip: cfg.robot_ip,
                      }
                    : undefined
                }
                version={cfg?.updated_at}
                saving={updateRobot.isPending}
                onSave={(patch) =>
                  updateRobot.mutate(patch as RobotConfigPatch, {
                    onSuccess: (r) => {
                      if (r.restarted) toast.success('Saved — robot link restarted')
                      else
                        toast.warning(
                          `Saved, but the link restart failed${r.restart_error ? `: ${r.restart_error}` : ''}`,
                        )
                    },
                  })
                }
                groups={[
                  {
                    fields: [
                      { kind: 'switch', key: 'enabled', label: 'Enabled' },
                      { kind: 'number', key: 'bind_port', label: 'Bind port', int: true, min: 1024 },
                      {
                        kind: 'number',
                        key: 'robot_port',
                        label: 'Robot port',
                        int: true,
                        min: 1024,
                        help: 'Destination port for pose packets',
                      },
                      {
                        kind: 'text',
                        key: 'robot_ip',
                        label: 'Robot IP',
                        placeholder: 'auto',
                        help: "leave empty to learn the robot's address automatically",
                      },
                    ],
                  },
                ]}
              />
            </CardContent>
          </Card>
        </div>

        {/* IMU column */}
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="flex items-center gap-2 text-sm">
                IMU
                <StatusDot tone={imu?.imu_ok ? 'good' : 'bad'} />
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <div className="flex items-baseline gap-4">
                <span className="font-mono text-2xl font-semibold tabular-nums">
                  {formatHz(imu?.rate_hz, 0)}
                </span>
                <span className="text-xs text-muted-foreground">
                  {formatCount(imu?.samples)} samples · {formatCount(imu?.fw_drops)} drops ·{' '}
                  {formatCount(imu?.crc_errors)} CRC errors
                </span>
              </div>
              <RollingChart points={imuSeries} series={[{ key: 'rate', label: 'IMU rate (Hz)' }]} height={100} />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3" data-testid="imu-attitude-card">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">IMU attitude</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <ImuAttitudeViewer />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">IMU noise model</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <ConfigForm
                value={
                  imuConfig.data
                    ? {
                        rate_hz: imuConfig.data.rate_hz,
                        accel_noise_density: imuConfig.data.accel_noise_density,
                        accel_random_walk: imuConfig.data.accel_random_walk,
                        gyro_noise_density: imuConfig.data.gyro_noise_density,
                        gyro_random_walk: imuConfig.data.gyro_random_walk,
                      }
                    : undefined
                }
                version={imuConfig.data?.updated_at}
                saving={updateImu.isPending}
                onSave={(patch) =>
                  updateImu.mutate(patch, { onSuccess: () => toast.success('IMU config saved') })
                }
                groups={[
                  {
                    fields: [
                      { kind: 'number', key: 'rate_hz', label: 'Sample rate', unit: 'Hz', min: 1 },
                      { kind: 'number', key: 'accel_noise_density', label: 'Accel noise density (N)' },
                      { kind: 'number', key: 'accel_random_walk', label: 'Accel random walk (K)' },
                      { kind: 'number', key: 'gyro_noise_density', label: 'Gyro noise density (N)' },
                      { kind: 'number', key: 'gyro_random_walk', label: 'Gyro random walk (K)' },
                    ],
                  },
                ]}
              />
              <p className="mt-3 text-xs text-muted-foreground">
                Refine these from a static log on the{' '}
                <Link to="/calibration/allan" className="text-primary hover:underline">
                  Allan variance
                </Link>{' '}
                page.
              </p>
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Robot–IMU mounting</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <TRobotImuEditor />
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  )
}
