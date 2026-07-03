import { useState } from 'react'
import { Link } from 'react-router-dom'
import { toast } from 'sonner'
import { AlertTriangle, Circle, Square } from 'lucide-react'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Progress } from '@/components/ui/progress'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { ConfirmButton } from '@/components/ConfirmButton'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { formatCount, formatUnixSeconds } from '@/lib/format'
import {
  useAllanStatus,
  useAnalyzeAllan,
  useApplyAllan,
  useImuStatus,
  useStartAllanRecording,
  useStopAllanRecording,
} from '@/queries/imu'

const DURATIONS = [
  { label: '1 hour (quick check)', value: 3600 },
  { label: '3 hours (minimum for credible K)', value: 3 * 3600 },
  { label: '8 hours (overnight, recommended)', value: 8 * 3600 },
  { label: '12 hours', value: 12 * 3600 },
]

function sci(v: number): string {
  return v.toExponential(3)
}

export default function AllanPage() {
  const imu = useImuStatus()
  const allan = useAllanStatus()
  const start = useStartAllanRecording()
  const stop = useStopAllanRecording()
  const analyze = useAnalyzeAllan()
  const apply = useApplyAllan()

  const [duration, setDuration] = useState(8 * 3600)

  const rec = allan.data?.recording
  const analysis = allan.data?.last_analysis
  const recording = rec?.recording ?? false
  const progress =
    recording && rec && duration > 0
      ? Math.min(100, Math.max(0, ((duration - rec.remaining_s) / duration) * 100))
      : 0

  return (
    <div>
      <PageHeader
        title="Allan variance"
        description="Record a long static IMU log, then fit noise density (N) and random walk (K) for the Kalibr/VIO noise model."
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">1 · Record static log</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <div className="flex items-center gap-2 text-sm">
                <StatusDot tone={imu.data?.imu_ok ? 'good' : 'bad'} />
                IMU {imu.data?.imu_ok ? `healthy — ${imu.data.rate_hz.toFixed(0)} Hz` : 'unhealthy'}
              </div>
              <p className="text-xs text-muted-foreground">
                The robot must sit completely still on a rigid surface for the entire recording —
                overnight is ideal. Motion invalidates the fit (warnings will flag it).
              </p>
              {!recording ? (
                <div className="flex items-center gap-2">
                  <Select value={String(duration)} onValueChange={(v) => setDuration(Number(v))}>
                    <SelectTrigger className="w-64" aria-label="Recording duration">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {DURATIONS.map((d) => (
                        <SelectItem key={d.value} value={String(d.value)}>
                          {d.label}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  <Button
                    disabled={start.isPending || !(imu.data?.imu_ok ?? false)}
                    onClick={() =>
                      start.mutate(duration, {
                        onSuccess: () => toast.success('Recording started'),
                      })
                    }
                  >
                    <Circle className="fill-destructive text-destructive" /> Start
                  </Button>
                </div>
              ) : (
                <div className="space-y-3">
                  <Progress value={progress} />
                  <div className="flex items-center gap-4 font-mono text-xs tabular-nums text-muted-foreground">
                    <span>{formatCount(rec?.samples)} samples</span>
                    <span>{rec ? `${Math.ceil(rec.remaining_s / 60)} min remaining` : ''}</span>
                  </div>
                  <ConfirmButton
                    size="sm"
                    variant="outline"
                    title="Stop the Allan recording?"
                    description="The log so far is kept and can still be analyzed, but short logs give an unreliable random-walk fit."
                    confirmLabel="Stop recording"
                    onConfirm={() => stop.mutate()}
                  >
                    <Square /> Stop early
                  </ConfirmButton>
                </div>
              )}
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">2 · Analyze</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <p className="text-xs text-muted-foreground">
                Runs overlapping ADEV on the most recent log: N at τ=1 s on the −1/2 slope, K at
                τ=3 s on the +1/2 slope (Kalibr units).
              </p>
              <Button
                variant="secondary"
                disabled={recording || analyze.isPending}
                onClick={() =>
                  analyze.mutate(undefined, {
                    onSuccess: () => toast.success('Analysis complete'),
                  })
                }
              >
                {analyze.isPending ? 'Analyzing…' : 'Analyze latest log'}
              </Button>
            </CardContent>
          </Card>
        </div>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="flex items-center justify-between text-sm">
              <span>3 · Results</span>
              {analysis && (
                <Badge variant="outline" className="font-normal">
                  {formatUnixSeconds(analysis.analyzed_at)}
                </Badge>
              )}
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-3 px-4">
            {!analysis ? (
              <p className="text-sm text-muted-foreground">No analysis yet.</p>
            ) : (
              <>
                {analysis.warnings.length > 0 && (
                  <div className="space-y-1 rounded-md border border-warning/40 bg-warning/10 p-3">
                    {analysis.warnings.map((w) => (
                      <div key={w} className="flex items-center gap-2 text-xs text-warning">
                        <AlertTriangle className="size-3.5 shrink-0" /> {w}
                      </div>
                    ))}
                  </div>
                )}
                <p className="text-xs text-muted-foreground">
                  {formatCount(analysis.samples)} samples · {(analysis.duration_s / 3600).toFixed(1)} h ·{' '}
                  {analysis.rate_hz.toFixed(0)} Hz
                </p>
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead>Axis</TableHead>
                      <TableHead>Noise density</TableHead>
                      <TableHead>Random walk</TableHead>
                      <TableHead>Fit</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {Object.entries(analysis.axes).map(([axis, r]) => (
                      <TableRow key={axis}>
                        <TableCell className="font-mono text-xs">{axis}</TableCell>
                        <TableCell
                          className={`font-mono text-xs tabular-nums ${r.noise_density_ok ? '' : 'text-warning'}`}
                        >
                          {sci(r.noise_density)}
                        </TableCell>
                        <TableCell
                          className={`font-mono text-xs tabular-nums ${r.random_walk_ok ? '' : 'text-warning'}`}
                        >
                          {sci(r.random_walk)}
                        </TableCell>
                        <TableCell className="text-xs text-muted-foreground">
                          {r.fit_quality}
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
                <div className="rounded-md border p-3 text-xs">
                  <p className="mb-1 font-medium">Suggested (worst axis):</p>
                  <div className="grid grid-cols-2 gap-1 font-mono tabular-nums text-muted-foreground">
                    <span>accel N: {sci(analysis.suggested.accel_noise_density)}</span>
                    <span>accel K: {sci(analysis.suggested.accel_random_walk)}</span>
                    <span>gyro N: {sci(analysis.suggested.gyro_noise_density)}</span>
                    <span>gyro K: {sci(analysis.suggested.gyro_random_walk)}</span>
                  </div>
                </div>
                <ConfirmButton
                  variant="default"
                  title="Apply Allan results to the IMU config?"
                  description="Overwrites the four noise parameters and reloads VIO/fusion. The previous values are visible on the Robot page until then."
                  confirmLabel="Apply"
                  onConfirm={() =>
                    apply.mutate(undefined, {
                      onSuccess: () => toast.success('IMU noise parameters updated'),
                    })
                  }
                >
                  Apply to IMU config
                </ConfirmButton>
                <p className="text-xs text-muted-foreground">
                  Current values are on the{' '}
                  <Link to="/robot" className="text-primary hover:underline">
                    Robot
                  </Link>{' '}
                  page.
                </p>
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
