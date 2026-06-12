import { Link } from 'react-router-dom'
import { Activity, Camera as CameraIcon, Crosshair, Move3d } from 'lucide-react'
import { calibrationQuality } from '@/api/cameras'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { EmptyState } from '@/components/EmptyState'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { formatUnixSeconds } from '@/lib/format'
import { useCameras } from '@/queries/cameras'
import { useAllanStatus } from '@/queries/imu'

export default function CalibrationHubPage() {
  const cameras = useCameras()
  const allan = useAllanStatus({ poll: false })
  const cams = cameras.data ?? []

  return (
    <div>
      <PageHeader
        title="Calibration"
        description="Per-camera intrinsics, camera–IMU extrinsics, and IMU noise refinement."
      />

      <div className="grid grid-cols-1 gap-4 lg:grid-cols-2 xl:grid-cols-3">
        <Card className="py-4 gap-3 lg:col-span-2 xl:col-span-1">
          <CardHeader className="px-4">
            <CardTitle className="flex items-center gap-2 text-sm">
              <Crosshair className="size-4 text-primary" /> Camera intrinsics
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 px-4">
            {cams.length === 0 ? (
              <EmptyState icon={CameraIcon} title="No cameras registered" />
            ) : (
              cams.map((c) => {
                const q = calibrationQuality(c.reprojection_error_px)
                return (
                  <div
                    key={c.id}
                    className="flex items-center justify-between gap-2 rounded-md border px-3 py-2"
                  >
                    <span className="flex items-center gap-2 text-sm">
                      <StatusDot tone={c.online ? 'good' : 'bad'} />
                      {c.name}
                    </span>
                    <span className="flex items-center gap-2">
                      {c.calibrated_at ? (
                        <Badge
                          variant="outline"
                          className={
                            q === 'poor'
                              ? 'border-destructive/50 text-destructive'
                              : 'border-success/50 text-success'
                          }
                        >
                          {c.reprojection_error_px != null
                            ? `${c.reprojection_error_px.toFixed(2)} px`
                            : 'calibrated'}
                        </Badge>
                      ) : (
                        <Badge variant="outline">uncalibrated</Badge>
                      )}
                      <Button asChild size="sm" variant="secondary">
                        <Link to={`/calibration/intrinsics/${c.id}`}>Calibrate</Link>
                      </Button>
                    </span>
                  </div>
                )
              })
            )}
          </CardContent>
        </Card>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="flex items-center gap-2 text-sm">
              <Move3d className="size-4 text-primary" /> Camera–IMU extrinsics
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-3 px-4">
            <p className="text-xs text-muted-foreground">
              One hardware-synced multi-camera + IMU recording; required for VIO and for AprilTag
              field poses.
            </p>
            <div className="space-y-1">
              {cams.map((c) => (
                <div key={c.id} className="flex items-center justify-between text-sm">
                  <span>{c.name}</span>
                  <Badge variant="outline">
                    {c.extrinsics_calibrated_at
                      ? formatUnixSeconds(c.extrinsics_calibrated_at)
                      : 'none'}
                  </Badge>
                </div>
              ))}
            </div>
            <Button asChild size="sm">
              <Link to="/calibration/extrinsics">Run extrinsics</Link>
            </Button>
          </CardContent>
        </Card>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="flex items-center gap-2 text-sm">
              <Activity className="size-4 text-primary" /> IMU noise (Allan variance)
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-3 px-4">
            <p className="text-xs text-muted-foreground">
              Long static recording → noise density + random walk for the Kalibr/VIO noise model.
            </p>
            <div className="flex items-center justify-between text-sm">
              <span>Last analysis</span>
              <Badge variant="outline">
                {allan.data?.last_analysis
                  ? formatUnixSeconds(allan.data.last_analysis.analyzed_at)
                  : 'never'}
              </Badge>
            </div>
            {allan.data?.recording.recording && (
              <p className="text-xs text-warning">Recording in progress…</p>
            )}
            <Button asChild size="sm">
              <Link to="/calibration/allan">Run Allan</Link>
            </Button>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
