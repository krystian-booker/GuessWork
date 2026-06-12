import { useState } from 'react'
import { Link } from 'react-router-dom'
import { toast } from 'sonner'
import { Send } from 'lucide-react'
import type { CanMode } from '@/api/can'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import { Textarea } from '@/components/ui/textarea'
import { ConfigForm } from '@/components/config/ConfigForm'
import { Mat4Table } from '@/components/Mat4Table'
import { PageHeader } from '@/components/PageHeader'
import { RollingChart } from '@/components/RollingChart'
import { StatusDot } from '@/components/StatusDot'
import { useTimeSeries } from '@/hooks/use-time-series'
import { formatCount, formatHz, formatMs } from '@/lib/format'
import { isMat4 } from '@/lib/matrices'
import { useBenchPose, useCanConfig, useCanStatus, useUpdateCanConfig } from '@/queries/can'
import { useImuConfig, useImuStatus, useUpdateImuConfig } from '@/queries/imu'

const MODE_LABELS: Record<CanMode, string> = {
  off: 'Off',
  roborio: 'roboRIO — classic CAN 2.0 @ 1 Mbps',
  systemcore: 'SystemCore — CAN FD 1/4 Mbps',
}

function KV({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-baseline justify-between gap-4 text-sm">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right font-mono text-xs tabular-nums">{value}</span>
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
        Bench downlink: hand-craft a POSE frame to verify controller-side reception.
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

export default function CanPage() {
  const canStatus = useCanStatus()
  const canConfig = useCanConfig()
  const updateCan = useUpdateCanConfig()
  const imuStatus = useImuStatus()
  const imuConfig = useImuConfig()
  const updateImu = useUpdateImuConfig()

  const s = canStatus.data
  const imu = imuStatus.data

  const odomSeries = useTimeSeries(
    'can-odom',
    { rate: s?.odom.rate_hz },
    canStatus.dataUpdatedAt,
  )
  const imuSeries = useTimeSeries('imu-rate', { rate: imu?.rate_hz }, imuStatus.dataUpdatedAt)

  return (
    <div>
      <PageHeader
        title="CAN / IMU"
        description="Teensy bridge: chassis-speed odometry uplink, pose downlink, and the onboard IMU."
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        {/* CAN / Teensy column */}
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">CAN mode</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 px-4">
              <Select
                value={canConfig.data?.mode ?? ''}
                onValueChange={(mode) =>
                  updateCan.mutate(mode as CanMode, {
                    onSuccess: (r) => {
                      if (r.pushed) toast.success(`CAN mode set to ${r.mode}`)
                      else
                        toast.warning(
                          `Saved, but push to Teensy failed${r.push_error ? `: ${r.push_error}` : ''} — it re-syncs on reconnect`,
                        )
                    },
                  })
                }
              >
                <SelectTrigger className="w-full" aria-label="CAN mode">
                  <SelectValue placeholder="Loading…" />
                </SelectTrigger>
                <SelectContent>
                  {(Object.keys(MODE_LABELS) as CanMode[]).map((m) => (
                    <SelectItem key={m} value={m}>
                      {MODE_LABELS[m]}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              <p className="text-xs text-muted-foreground">
                Pushed to the Teensy immediately and on every reconnect (firmware ≥ 3).
              </p>
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="flex items-center gap-2 text-sm">
                Teensy
                <StatusDot tone={s?.teensy_connected ? 'good' : 'bad'} />
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-1.5 px-4">
              <KV label="Firmware" value={s?.fw_version ?? '—'} />
              <KV label="Firmware CAN mode" value={s?.fw_mode ?? '—'} />
              <KV
                label="CAN bus"
                value={
                  <span className={s?.can_ok ? 'text-success' : 'text-destructive'}>
                    {s ? (s.can_ok ? 'ok' : 'fault') : '—'}
                  </span>
                }
              />
              <KV
                label="Clock sync"
                value={
                  s ? (
                    <span className={s.clock_sync.healthy ? 'text-success' : 'text-warning'}>
                      {s.clock_sync.healthy ? 'healthy' : 'warming up'} ·{' '}
                      {s.clock_sync.drift_ppm.toFixed(1)} ppm · {s.clock_sync.resets} resets
                    </span>
                  ) : (
                    '—'
                  )
                }
              />
              <KV
                label="RX / drops / CRC"
                value={
                  s
                    ? `${formatCount(s.counters.can_rx)} / ${formatCount(s.counters.can_rx_drops)} / ${formatCount(s.counters.odom_crc_errors)}`
                    : '—'
                }
              />
              <KV
                label="Poses sent / errors"
                value={
                  s
                    ? `${formatCount(s.counters.pose_sent)} / ${formatCount(s.counters.pose_send_errors)}`
                    : '—'
                }
              />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Chassis odometry</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <div className="flex items-baseline gap-4">
                <span className="font-mono text-2xl font-semibold tabular-nums">
                  {formatHz(s?.odom.rate_hz, 0)}
                </span>
                <span className="text-xs text-muted-foreground">
                  {formatCount(s?.odom.packets)} packets
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

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Bench pose downlink</CardTitle>
            </CardHeader>
            <CardContent className="px-4">
              <BenchPoseSender disabled={!s?.teensy_connected} />
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
