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
import { Switch } from '@/components/ui/switch'
import { useAvailableCameraModes, useAvailableCameras, useCreateCamera } from '@/queries/cameras'
import { ModeSelect } from './ModeSelect'

const PINS = [1, 2, 3, 4, 5, 6]

export function AddCameraDialog() {
  const [open, setOpen] = useState(false)
  const [serial, setSerial] = useState<string | null>(null)
  const [name, setName] = useState('')
  const [focal, setFocal] = useState('')
  const [mode, setMode] = useState<string | null>(null)
  const [hwSync, setHwSync] = useState(false)
  const [pin, setPin] = useState<number | null>(null)

  // Poll while open so plugging a camera in shows up live.
  const available = useAvailableCameras(open)
  const modes = useAvailableCameraModes(open ? serial : null)
  const create = useCreateCamera()

  const reset = () => {
    setSerial(null)
    setName('')
    setFocal('')
    setMode(null)
    setHwSync(false)
    setPin(null)
  }

  const focalNum = Number(focal)
  const canSubmit =
    serial != null &&
    name.trim().length > 0 &&
    Number.isFinite(focalNum) &&
    focalNum > 0 &&
    focalNum < 1000 &&
    (!hwSync || pin != null)

  const submit = () => {
    if (!canSubmit || serial == null) return
    create.mutate(
      {
        name: name.trim(),
        serial,
        focal_length_mm: focalNum,
        mode: mode ?? undefined,
        hardware_sync_enabled: hwSync,
        trigger_output_pin: hwSync ? (pin ?? undefined) : undefined,
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

        <div className="space-y-4">
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

          <div className="flex items-center justify-between">
            <div>
              <Label>Hardware sync</Label>
              <p className="text-xs text-muted-foreground">Trigger from a Teensy output pin</p>
            </div>
            <Switch checked={hwSync} onCheckedChange={setHwSync} aria-label="Hardware sync" />
          </div>
          {hwSync && (
            <div className="space-y-1.5">
              <Label>Trigger output pin</Label>
              <Select value={pin?.toString() ?? ''} onValueChange={(v) => setPin(Number(v))}>
                <SelectTrigger className="w-full" aria-label="Trigger output pin">
                  <SelectValue placeholder="Select pin" />
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
          )}
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
