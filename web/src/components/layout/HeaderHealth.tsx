import { Badge } from '@/components/ui/badge'
import { StatusDot } from '@/components/StatusDot'
import { useFusionStatus } from '@/queries/fusion'
import { useImuStatus } from '@/queries/imu'
import { useStatus } from '@/queries/status'

// Global health cluster in the header. Shares query keys with the Dashboard,
// so showing it everywhere costs no extra requests when the Dashboard is up.
export function HeaderHealth() {
  const status = useStatus()
  const imu = useImuStatus()
  const fusion = useFusionStatus()

  const cams = status.data?.cameras ?? []
  const online = cams.filter((c) => c.online).length
  const camTone = cams.length === 0 ? 'idle' : online === cams.length ? 'good' : online > 0 ? 'warn' : 'bad'

  const teensyOk = imu.data?.teensy_connected ?? false

  return (
    <div className="flex items-center gap-4 text-xs text-muted-foreground" data-testid="header-health">
      <span className="flex items-center gap-1.5" data-testid="header-cameras">
        <StatusDot tone={camTone} />
        <span className="font-mono tabular-nums">
          {online}/{cams.length}
        </span>
        <span className="hidden sm:inline">cameras</span>
      </span>
      <span className="flex items-center gap-1.5" data-testid="header-teensy">
        <StatusDot tone={teensyOk ? 'good' : 'bad'} />
        <span className="hidden sm:inline">Teensy</span>
      </span>
      {fusion.data && (
        <Badge variant="outline" className="font-mono text-[11px]" data-testid="header-fusion-mode">
          {fusion.data.mode || (fusion.data.enabled ? 'initializing' : 'fusion off')}
        </Badge>
      )}
    </div>
  )
}
