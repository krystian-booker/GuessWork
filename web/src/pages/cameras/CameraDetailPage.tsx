import { useEffect, useState } from 'react'
import { Link, useNavigate, useParams } from 'react-router-dom'
import { toast } from 'sonner'
import { ArrowLeft, Crosshair } from 'lucide-react'
import { CAMERA_ORIENTATIONS, calibrationQuality, type CameraOrientation } from '@/api/cameras'
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
import { Switch } from '@/components/ui/switch'
import { ConfirmButton } from '@/components/ConfirmButton'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { WebRtcPlayer } from '@/components/stream/WebRtcPlayer'
import { formatUnixSeconds } from '@/lib/format'
import {
  useCamera,
  useCameraModes,
  useDeleteCamera,
  useUpdateCamera,
} from '@/queries/cameras'
import { useDeleteCalibration } from '@/queries/calibration'
import { useStatus } from '@/queries/status'
import { CameraSettingsPanel } from './CameraSettingsPanel'
import { ModeSelect } from './ModeSelect'
import { RoleSelect } from './RoleSelect'

const PINS = [1, 2, 3, 4, 5, 6]

function SectionCard({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <Card className="py-4 gap-3">
      <CardHeader className="px-4">
        <CardTitle className="text-sm">{title}</CardTitle>
      </CardHeader>
      <CardContent className="px-4">{children}</CardContent>
    </Card>
  )
}

export default function CameraDetailPage() {
  const { id } = useParams()
  const cameraId = Number(id)
  const navigate = useNavigate()

  const camera = useCamera(cameraId)
  const status = useStatus()
  const modes = useCameraModes(cameraId, camera.data?.online ?? false)
  const update = useUpdateCamera(cameraId)
  const deleteCamera = useDeleteCamera()
  const deleteCalibration = useDeleteCalibration(cameraId)

  const cam = camera.data

  // Identity edits are explicit-save (name/focal), unlike the live sliders.
  const [name, setName] = useState('')
  const [focal, setFocal] = useState('')
  useEffect(() => {
    if (cam) {
      setName(cam.name)
      setFocal(String(cam.focal_length_mm))
    }
  }, [cam?.id, cam?.name, cam?.focal_length_mm]) // eslint-disable-line react-hooks/exhaustive-deps

  if (camera.isLoading) return <p className="text-sm text-muted-foreground">Loading…</p>
  if (!cam) return <p className="text-sm text-destructive">Camera not found.</p>

  const identityDirty = name !== cam.name || Number(focal) !== cam.focal_length_mm
  const aspect =
    cam.mode_width && cam.mode_height ? `${cam.mode_width} / ${cam.mode_height}` : '4 / 3'
  const startError = status.data?.cameras.find((c) => c.id === cam.id)?.last_start_error

  return (
    <div>
      <PageHeader
        title={cam.name}
        description={
          <span className="flex items-center gap-2">
            <StatusDot tone={cam.online ? 'good' : 'bad'} />
            {cam.online ? 'online' : 'offline'} · <span className="font-mono">{cam.serial}</span>
          </span>
        }
        actions={
          <Button variant="ghost" size="sm" onClick={() => navigate('/cameras')}>
            <ArrowLeft /> All cameras
          </Button>
        }
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-3">
        <div className="xl:col-span-2">
          {cam.online ? (
            <WebRtcPlayer cameraId={cam.id} aspectRatio={aspect} orientation={cam.orientation} />
          ) : (
            <div
              className="flex flex-col items-center justify-center gap-2 rounded-lg border bg-card px-6 text-sm text-muted-foreground"
              style={{ aspectRatio: aspect }}
            >
              Camera offline — no preview
              {startError && (
                <p className="max-w-full text-center text-xs text-destructive" data-testid="start-error">
                  Last start attempt failed: {startError}
                </p>
              )}
            </div>
          )}
        </div>

        <div className="space-y-4">
          <SectionCard title="Identity">
            <div className="space-y-3">
              <div className="space-y-1.5">
                <Label htmlFor="name">Name</Label>
                <Input id="name" value={name} onChange={(e) => setName(e.target.value)} />
              </div>
              <div className="space-y-1.5">
                <Label htmlFor="focal">Focal length (mm)</Label>
                <Input
                  id="focal"
                  type="number"
                  step="0.1"
                  value={focal}
                  onChange={(e) => setFocal(e.target.value)}
                />
              </div>
              {identityDirty && (
                <Button
                  size="sm"
                  disabled={update.isPending || !name.trim() || !(Number(focal) > 0)}
                  onClick={() =>
                    update.mutate(
                      { name: name.trim(), focal_length_mm: Number(focal) },
                      { onSuccess: () => toast.success('Camera updated') },
                    )
                  }
                >
                  Save
                </Button>
              )}
            </div>
          </SectionCard>

          <SectionCard title="Video mode">
            <ModeSelect
              value={cam.mode}
              options={modes.data?.options ?? []}
              loading={cam.online && modes.isLoading}
              error={null}
              disabled={!cam.online}
              onChange={(mode) =>
                update.mutate({ mode }, { onSuccess: () => toast.success('Mode updated') })
              }
              hint={
                cam.online
                  ? 'This camera doesn’t expose a video-mode node.'
                  : (cam.mode ?? 'Camera offline — mode unavailable.')
              }
              disabledHint="Camera offline — mode can’t be changed."
            />
          </SectionCard>

          <SectionCard title="Orientation">
            <div className="space-y-2">
              <Select
                value={String(cam.orientation)}
                onValueChange={(v) =>
                  update.mutate(
                    { orientation: Number(v) as CameraOrientation },
                    { onSuccess: () => toast.success('Orientation updated') },
                  )
                }
              >
                <SelectTrigger className="w-full" aria-label="Orientation">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {CAMERA_ORIENTATIONS.map((deg) => (
                    <SelectItem key={deg} value={String(deg)}>
                      {deg === 0 ? '0° — upright' : `${deg}° clockwise`}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              <p className="text-xs text-muted-foreground">
                Rotates the live preview. For VIO cameras, 180° also flips the frames fed to
                VIO and recorded for its calibration, so an upside-down-mounted stereo camera
                matches its partner — set before calibrating. AprilTag detection always uses
                the raw sensor image.
              </p>
            </div>
          </SectionCard>

          <SectionCard title="Pipeline role">
            <RoleSelect
              value={cam.role}
              onChange={(role) =>
                update.mutate({ role }, { onSuccess: () => toast.success('Role updated') })
              }
            />
            <p className="mt-2 text-xs text-muted-foreground">
              AprilTag cameras feed field-pose measurements; the VIO pair feeds stereo odometry.
            </p>
          </SectionCard>

          <SectionCard title="Camera settings">
            <CameraSettingsPanel camera={cam} />
          </SectionCard>

          <SectionCard title="Hardware sync">
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <p className="text-sm">Teensy-triggered capture</p>
                <Switch
                  checked={cam.hardware_sync_enabled}
                  onCheckedChange={(enabled) => {
                    if (enabled && cam.trigger_output_pin == null) {
                      toast.error('Pick a trigger pin below first')
                      return
                    }
                    update.mutate({ hardware_sync_enabled: enabled })
                  }}
                  aria-label="Hardware sync"
                />
              </div>
              <div className="space-y-1.5">
                <Label>Trigger output pin</Label>
                <Select
                  value={cam.trigger_output_pin?.toString() ?? ''}
                  onValueChange={(v) =>
                    update.mutate({
                      hardware_sync_enabled: true,
                      trigger_output_pin: Number(v),
                    })
                  }
                >
                  <SelectTrigger className="w-full" aria-label="Trigger output pin">
                    <SelectValue placeholder="Not wired" />
                  </SelectTrigger>
                  <SelectContent>
                    {PINS.map((p) => (
                      <SelectItem key={p} value={p.toString()}>
                        Pin {p}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
              <p className="text-xs text-muted-foreground">
                Pulse rates are configured per group on the{' '}
                <Link to="/hardware-sync" className="text-primary hover:underline">
                  Hardware Sync
                </Link>{' '}
                page.
              </p>
            </div>
          </SectionCard>

          <SectionCard title="Calibration">
            <div className="space-y-2 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-muted-foreground">Intrinsics</span>
                {cam.calibrated_at ? (
                  <Badge
                    variant="outline"
                    className={
                      calibrationQuality(cam.reprojection_error_px) === 'poor'
                        ? 'border-destructive/50 text-destructive'
                        : 'border-success/50 text-success'
                    }
                  >
                    {cam.reprojection_error_px != null
                      ? `${cam.reprojection_error_px.toFixed(2)} px`
                      : 'calibrated'}
                  </Badge>
                ) : (
                  <Badge variant="outline">none</Badge>
                )}
              </div>
              {cam.calibrated_at && (
                <p className="text-xs text-muted-foreground">
                  Uploaded {formatUnixSeconds(cam.calibrated_at)}
                </p>
              )}
              <div className="flex items-center justify-between">
                <span className="text-muted-foreground">Cam-IMU extrinsics</span>
                <Badge variant="outline">
                  {cam.extrinsics_calibrated_at
                    ? formatUnixSeconds(cam.extrinsics_calibrated_at)
                    : 'none'}
                </Badge>
              </div>
              <div className="flex gap-2 pt-2">
                <Button asChild size="sm" variant="secondary">
                  <Link to={`/calibration/intrinsics/${cam.id}`}>
                    <Crosshair /> Calibrate
                  </Link>
                </Button>
                {cam.calibrated_at && (
                  <ConfirmButton
                    size="sm"
                    variant="outline"
                    title="Delete calibration?"
                    description="AprilTag detection and VIO for this camera stop until a new calibration is uploaded."
                    confirmLabel="Delete"
                    onConfirm={() =>
                      deleteCalibration.mutate(undefined, {
                        onSuccess: () => toast.success('Calibration deleted'),
                      })
                    }
                  >
                    Delete calibration
                  </ConfirmButton>
                )}
              </div>
            </div>
          </SectionCard>

          <SectionCard title="Danger zone">
            <ConfirmButton
              title={`Delete camera "${cam.name}"?`}
              description="Removes the camera, its calibration and role assignments. The physical camera can be re-added later."
              confirmLabel="Delete camera"
              onConfirm={() =>
                deleteCamera.mutate(cam.id, {
                  onSuccess: () => {
                    toast.success(`Camera "${cam.name}" deleted`)
                    navigate('/cameras')
                  },
                })
              }
            >
              Delete camera
            </ConfirmButton>
          </SectionCard>
        </div>
      </div>
    </div>
  )
}
