import { useEffect, useState } from 'react'
import { toast } from 'sonner'
import type { TriggerGroup } from '@/api/hardwareSync'
import { Button } from '@/components/ui/button'
import { Checkbox } from '@/components/ui/checkbox'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { useCreateTriggerGroup, useUpdateTriggerGroup } from '@/queries/hardwareSync'

const PINS = [1, 2, 3, 4, 5, 6]

function pinsClaimedByOthers(groups: TriggerGroup[], excludeId: number | null): Set<number> {
  const out = new Set<number>()
  for (const g of groups) {
    if (g.id === excludeId) continue
    for (const p of g.output_pins) out.add(p)
  }
  return out
}

// Create (group=null) / edit dialog for a trigger group. Client-side pin
// conflict warnings are advisory — the server 409 is authoritative.
export function TriggerGroupDialog({
  open,
  onOpenChange,
  group,
  groups,
}: {
  open: boolean
  onOpenChange: (open: boolean) => void
  group: TriggerGroup | null
  groups: TriggerGroup[]
}) {
  const create = useCreateTriggerGroup()
  const update = useUpdateTriggerGroup()

  const [name, setName] = useState('')
  const [fps, setFps] = useState('30')
  const [pins, setPins] = useState<Set<number>>(new Set())

  useEffect(() => {
    if (open) {
      setName(group?.name ?? '')
      setFps(String(group?.fps ?? 30))
      setPins(new Set(group?.output_pins ?? []))
    }
  }, [open, group])

  const claimed = pinsClaimedByOthers(groups, group?.id ?? null)
  const fpsNum = Number(fps)
  const conflict = [...pins].some((p) => claimed.has(p))
  const canSubmit =
    name.trim().length > 0 && Number.isFinite(fpsNum) && fpsNum > 0 && pins.size > 0

  const submit = () => {
    const payload = { name: name.trim(), fps: fpsNum, output_pins: [...pins].sort() }
    const opts = {
      onSuccess: () => {
        toast.success(group ? 'Group updated' : 'Group created')
        onOpenChange(false)
      },
    }
    if (group) update.mutate({ id: group.id, patch: payload }, opts)
    else create.mutate(payload, opts)
  }

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="sm:max-w-sm">
        <DialogHeader>
          <DialogTitle>{group ? `Edit "${group.name}"` : 'New trigger group'}</DialogTitle>
          <DialogDescription>
            All pins in a group pulse in lockstep at the group FPS.
          </DialogDescription>
        </DialogHeader>
        <div className="space-y-4">
          <div className="space-y-1.5">
            <Label htmlFor="group-name">Name</Label>
            <Input
              id="group-name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="e.g. stereo-pair"
            />
          </div>
          <div className="space-y-1.5">
            <Label htmlFor="group-fps">FPS</Label>
            <Input
              id="group-fps"
              type="number"
              min={1}
              step="1"
              value={fps}
              onChange={(e) => setFps(e.target.value)}
            />
          </div>
          <div className="space-y-1.5">
            <Label>Output pins</Label>
            <div className="flex flex-wrap gap-3">
              {PINS.map((p) => (
                <label key={p} className="flex items-center gap-1.5 text-sm">
                  <Checkbox
                    aria-label={`Pin ${p}`}
                    checked={pins.has(p)}
                    onCheckedChange={(c) => {
                      const next = new Set(pins)
                      if (c) next.add(p)
                      else next.delete(p)
                      setPins(next)
                    }}
                  />
                  {p}
                  {claimed.has(p) && <span className="text-xs text-warning">(in use)</span>}
                </label>
              ))}
            </div>
            {conflict && (
              <p className="text-xs text-warning">
                A selected pin is already claimed by another group — the server will reject this.
              </p>
            )}
          </div>
        </div>
        <DialogFooter>
          <Button variant="ghost" onClick={() => onOpenChange(false)}>
            Cancel
          </Button>
          <Button onClick={submit} disabled={!canSubmit || create.isPending || update.isPending}>
            {group ? 'Save' : 'Create'}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
