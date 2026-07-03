import { useState } from 'react'
import { toast } from 'sonner'
import { Plus } from 'lucide-react'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import { CAMERA_ORIENTATIONS, type CameraOrientation, type CameraRole } from '@/api/cameras'
import { useAvailableCameraModes, useAvailableCameras, useCreateCamera } from '@/queries/cameras'
import { ModeSelect } from './ModeSelect'
import { RoleSelect } from './RoleSelect'

export function AddCameraDialog() {
  const [open, setOpen] = useState(false)
  const [serial, setSerial] = useState<string | null>(null)
  const [name, setName] = useState('')
  const [focal, setFocal] = useState('')
  const [mode, setMode] = useState<string | null>(null)
  const [role, setRole] = useState<CameraRole | null>(null)
  const [orientation, setOrientation] = useState<CameraOrientation | null>(null)

  // Poll while open so plugging a camera in shows up live.
  const available = useAvailableCameras(open)
  const modes = useAvailableCameraModes(open ? serial : null)
  const create = useCreateCamera()

  const reset = () => {
    setSerial(null)
    setName('')
    setFocal('')
    setMode(null)
    setRole(null)
    setOrientation(null)
  }

  const focalNum = Number(focal)
  const canSubmit =
    serial != null &&
    name.trim().length > 0 &&
    Number.isFinite(focalNum) &&
    focalNum > 0 &&
    focalNum < 1000

  const submit = () => {
    if (!canSubmit || serial == null) return
    create.mutate(
      {
        name: name.trim(),
        serial,
        focal_length_mm: focalNum,
        mode: mode ?? undefined,
        // Only sent when the user picked a value — omitted fields keep the
        // server defaults (no role, orientation 0).
        role: role ?? undefined,
        orientation: orientation ?? undefined,
      },
      {
        onSuccess: (cam) => {
          toast.success(`Camera "${cam.name}" added`)
          setOpen(false)
          reset()
        },
      },
    )
  }

  return (
    <Dialog
      open={open}
      onOpenChange={(o) => {
        setOpen(o)
        if (!o) reset()
      }}
    >
      <DialogTrigger asChild>
        <Button data-testid="add-camera">
          <Plus /> Add camera
        </Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-md">
        <DialogHeader>
          <DialogTitle>Add camera</DialogTitle>
          <DialogDescription>
            Register a connected Spinnaker camera. Unplugged cameras don't appear here.
          </DialogDescription>
        </DialogHeader>

        {/* min-w-0: DialogContent is a grid; without it the select triggers'
            nowrap labels set the track's min-content width and overflow the
            dialog instead of truncating. */}
        <div className="min-w-0 space-y-4">
          <div className="space-y-1.5">
            <Label>Detected camera</Label>
            <Select value={serial ?? ''} onValueChange={(s) => setSerial(s)}>
              <SelectTrigger className="w-full" aria-label="Detected camera">
                <SelectValue
                  placeholder={
                    available.isLoading
                      ? 'Scanning…'
                      : (available.data?.length ?? 0) === 0
                        ? 'No unregistered cameras found'
                        : 'Select a camera'
                  }
                />
              </SelectTrigger>
              <SelectContent>
                {(available.data ?? []).map((c) => (
                  <SelectItem key={c.serial} value={c.serial}>
                    {c.vendor} {c.model} — {c.serial}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div className="space-y-1.5">
            <Label htmlFor="cam-name">Name</Label>
            <Input
              id="cam-name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="e.g. front-left"
            />
          </div>

          <div className="space-y-1.5">
            <Label htmlFor="cam-focal">Focal length (mm)</Label>
            <Input
              id="cam-focal"
              type="number"
              min={0}
              step="0.1"
              value={focal}
              onChange={(e) => setFocal(e.target.value)}
              placeholder="e.g. 6.0"
            />
            <p className="text-xs text-muted-foreground">
              Lens focal length — drives the Kalibr camera-model hint.
            </p>
          </div>

          {serial && (
            <div className="space-y-1.5">
              <Label>Video mode</Label>
              <ModeSelect
                value={mode ?? modes.data?.current ?? null}
                options={modes.data?.options ?? []}
                loading={modes.isLoading}
                error={modes.error ? String(modes.error) : null}
                onChange={setMode}
                hint="This camera doesn't expose a video-mode node."
              />
            </div>
          )}

          <div className="space-y-1.5">
            <Label>Role</Label>
            <RoleSelect value={role} onChange={setRole} />
            <p className="text-xs text-muted-foreground">
              Optional — can also be changed later on the camera page.
            </p>
          </div>

          <div className="space-y-1.5">
            <Label>Orientation</Label>
            <Select
              value={orientation != null ? String(orientation) : ''}
              onValueChange={(v) => setOrientation(Number(v) as CameraOrientation)}
            >
              <SelectTrigger className="w-full" aria-label="Orientation">
                <SelectValue placeholder="0° — upright (default)" />
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
              Physical mounting rotation. Set it before calibrating a VIO camera.
            </p>
          </div>

        </div>

        <DialogFooter>
          <Button variant="ghost" onClick={() => setOpen(false)}>
            Cancel
          </Button>
          <Button onClick={submit} disabled={!canSubmit || create.isPending} data-testid="add-camera-submit">
            {create.isPending ? 'Adding…' : 'Add camera'}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
