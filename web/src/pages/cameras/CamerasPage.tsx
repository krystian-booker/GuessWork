import { useNavigate } from 'react-router-dom'
import { Camera as CameraIcon } from 'lucide-react'
import { calibrationQuality } from '@/api/cameras'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent } from '@/components/ui/card'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { EmptyState } from '@/components/EmptyState'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { useCameras } from '@/queries/cameras'
import { useStatus } from '@/queries/status'
import { AddCameraDialog } from './AddCameraDialog'
import { roleLabel } from './RoleSelect'

function CalibBadge({ reproj }: { reproj: number | null }) {
  const q = calibrationQuality(reproj)
  if (q === 'unknown') return <Badge variant="outline">uncalibrated</Badge>
  return (
    <Badge
      variant="outline"
      className={q === 'good' ? 'border-primary/50 text-primary' : 'border-destructive/50 text-destructive'}
    >
      {reproj!.toFixed(2)} px
    </Badge>
  )
}

export default function CamerasPage() {
  const navigate = useNavigate()
  const cameras = useCameras()
  const status = useStatus()

  const fpsById = new Map(status.data?.cameras.map((c) => [c.id, c]) ?? [])
  const rows = cameras.data ?? []

  return (
    <div>
      <PageHeader
        title="Cameras"
        description="Registered FLIR cameras and their pipeline roles."
        actions={<AddCameraDialog />}
      />

      {rows.length === 0 && !cameras.isLoading ? (
        <EmptyState
          icon={CameraIcon}
          title="No cameras registered"
          description="Plug a camera in and use Add camera to register it."
        />
      ) : (
        <Card className="py-0">
          <CardContent className="px-0">
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead className="w-10" />
                  <TableHead>Name</TableHead>
                  <TableHead>Serial</TableHead>
                  <TableHead>Mode</TableHead>
                  <TableHead>Role</TableHead>
                  <TableHead>Calibration</TableHead>
                  <TableHead className="text-right">FPS</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {rows.map((cam) => {
                  const live = fpsById.get(cam.id)
                  const online = live?.online ?? cam.online
                  return (
                    <TableRow
                      key={cam.id}
                      className="cursor-pointer"
                      data-testid={`camera-row-${cam.id}`}
                      onClick={() => navigate(`/cameras/${cam.id}`)}
                    >
                      <TableCell>
                        <StatusDot tone={online ? 'good' : 'bad'} />
                      </TableCell>
                      <TableCell className="font-medium">{cam.name}</TableCell>
                      <TableCell className="font-mono text-xs text-muted-foreground">
                        {cam.serial}
                      </TableCell>
                      <TableCell className="text-xs text-muted-foreground">
                        {cam.mode_width && cam.mode_height
                          ? `${cam.mode_width}×${cam.mode_height}${cam.mode_max_fps ? ` @ ${Math.round(cam.mode_max_fps)}` : ''}`
                          : (cam.mode ?? '—')}
                      </TableCell>
                      <TableCell>
                        {cam.role ? (
                          <Badge variant="secondary">{roleLabel(cam.role)}</Badge>
                        ) : (
                          <span className="text-xs text-muted-foreground">—</span>
                        )}
                      </TableCell>
                      <TableCell>
                        <CalibBadge reproj={cam.reprojection_error_px} />
                      </TableCell>
                      <TableCell className="text-right font-mono text-xs tabular-nums">
                        {online && live ? live.fps_1s.toFixed(1) : '—'}
                      </TableCell>
                    </TableRow>
                  )
                })}
              </TableBody>
            </Table>
          </CardContent>
        </Card>
      )}
    </div>
  )
}
