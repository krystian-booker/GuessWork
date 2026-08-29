import { useState } from 'react'
import { Link } from 'react-router-dom'
import { toast } from 'sonner'
import { Pencil, Plus, Trash2, Zap } from 'lucide-react'
import type { TriggerGroup } from '@/api/hardwareSync'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { ConfirmButton } from '@/components/ConfirmButton'
import { EmptyState } from '@/components/EmptyState'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { formatCount } from '@/lib/format'
import { useCameras } from '@/queries/cameras'
import {
  useArmHwSync,
  useDeleteTriggerGroup,
  useHwSyncStatus,
  useStopHwSync,
  useTriggerGroups,
} from '@/queries/hardwareSync'
import { TriggerGroupDialog } from './TriggerGroupDialog'

export default function HardwareSyncPage() {
  const status = useHwSyncStatus()
  const groups = useTriggerGroups()
  const cameras = useCameras()
  const armMut = useArmHwSync()
  const stopMut = useStopHwSync()
  const deleteGroup = useDeleteTriggerGroup()

  const [dialogOpen, setDialogOpen] = useState(false)
  const [editing, setEditing] = useState<TriggerGroup | null>(null)

  const s = status.data
  const groupList = groups.data ?? []
  const syncedCams = (cameras.data ?? []).filter((c) => c.hardware_sync_enabled)

  return (
    <div>
      <PageHeader
        title="Hardware Sync"
        description="sync controller trigger groups — pulse cameras in lockstep on a shared clock."
        actions={
          <Button
            onClick={() => {
              setEditing(null)
              setDialogOpen(true)
            }}
          >
            <Plus /> New group
          </Button>
        }
      />

      <Card className="mb-4 py-4">
        <CardContent className="flex flex-wrap items-center gap-x-6 gap-y-2 px-4 text-sm">
          <span className="flex items-center gap-2">
            <StatusDot tone={s?.connected ? 'good' : 'bad'} />
            sync controller {s?.connected ? 'connected' : 'disconnected'}
            {s?.port && <span className="font-mono text-xs text-muted-foreground">{s.port}</span>}
          </span>
          <span className="flex items-center gap-2">
            <StatusDot tone={s?.armed ? 'good' : 'idle'} pulse={s?.armed ?? false} />
            {s?.armed ? 'Armed' : 'Disarmed'}
          </span>
          <span className="font-mono text-xs tabular-nums text-muted-foreground">
            {formatCount(s?.total_pulses)} pulses
          </span>
          {s?.last_error && <span className="text-xs text-destructive">{s.last_error}</span>}
          <span className="ml-auto flex gap-2">
            <Button
              size="sm"
              disabled={!s?.connected || groupList.length === 0 || armMut.isPending}
              onClick={() =>
                armMut.mutate(undefined, {
                  onSuccess: () => toast.success('Trigger outputs armed'),
                })
              }
            >
              <Zap /> Arm
            </Button>
            {s?.armed ? (
              <ConfirmButton
                size="sm"
                variant="outline"
                title="Stop trigger outputs?"
                description="Hardware-synced cameras stop receiving pulses (and stop producing frames) until re-armed."
                confirmLabel="Stop outputs"
                onConfirm={() =>
                  stopMut.mutate(undefined, {
                    onSuccess: () => toast.success('Trigger outputs stopped'),
                  })
                }
              >
                Stop
              </ConfirmButton>
            ) : (
              <Button size="sm" variant="outline" disabled>
                Stop
              </Button>
            )}
          </span>
        </CardContent>
      </Card>

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        <Card className="py-4 gap-3">
          <CardHeader className="px-4">
            <CardTitle className="text-sm">Trigger groups</CardTitle>
          </CardHeader>
          <CardContent className="px-4">
            {groupList.length === 0 ? (
              <EmptyState
                icon={Zap}
                title="No trigger groups"
                description="Create a group, wire its pins to camera trigger inputs, then arm."
              />
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Name</TableHead>
                    <TableHead>FPS</TableHead>
                    <TableHead>Pins</TableHead>
                    <TableHead className="w-20" />
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {groupList.map((g) => (
                    <TableRow key={g.id}>
                      <TableCell className="font-medium">{g.name}</TableCell>
                      <TableCell className="font-mono tabular-nums">{g.fps}</TableCell>
                      <TableCell>
                        <span className="flex gap-1">
                          {g.output_pins.map((p) => (
                            <Badge key={p} variant="secondary" className="font-mono text-[11px]">
                              {p}
                            </Badge>
                          ))}
                        </span>
                      </TableCell>
                      <TableCell>
                        <span className="flex justify-end gap-1">
                          <Button
                            size="icon"
                            variant="ghost"
                            aria-label={`Edit ${g.name}`}
                            onClick={() => {
                              setEditing(g)
                              setDialogOpen(true)
                            }}
                          >
                            <Pencil />
                          </Button>
                          <ConfirmButton
                            size="icon"
                            variant="ghost"
                            aria-label={`Delete ${g.name}`}
                            title={`Delete group "${g.name}"?`}
                            description="Re-arm afterwards to push the change to the sync controller."
                            confirmLabel="Delete"
                            onConfirm={() => deleteGroup.mutate(g.id)}
                          >
                            <Trash2 className="text-destructive" />
                          </ConfirmButton>
                        </span>
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            )}
            <p className="mt-3 text-xs text-muted-foreground">
              Changes are stored immediately but only pushed to the sync controller when you arm.
            </p>
          </CardContent>
        </Card>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="text-sm">Camera wiring</CardTitle>
          </CardHeader>
          <CardContent className="px-4">
            {syncedCams.length === 0 ? (
              <p className="text-sm text-muted-foreground">
                No cameras have hardware sync enabled. Enable it per camera on its{' '}
                <Link to="/cameras" className="text-primary hover:underline">
                  detail page
                </Link>
                .
              </p>
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Camera</TableHead>
                    <TableHead>Trigger pin</TableHead>
                    <TableHead>Group</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {syncedCams.map((c) => {
                    const group = groupList.find((g) =>
                      c.trigger_output_pin != null && g.output_pins.includes(c.trigger_output_pin),
                    )
                    return (
                      <TableRow key={c.id}>
                        <TableCell>{c.name}</TableCell>
                        <TableCell className="font-mono tabular-nums">
                          {c.trigger_output_pin ?? '—'}
                        </TableCell>
                        <TableCell>
                          {group ? (
                            <Badge variant="secondary">
                              {group.name} @ {group.fps} fps
                            </Badge>
                          ) : (
                            <span className="text-xs text-warning">no group claims this pin</span>
                          )}
                        </TableCell>
                      </TableRow>
                    )
                  })}
                </TableBody>
              </Table>
            )}
          </CardContent>
        </Card>
      </div>

      <TriggerGroupDialog
        open={dialogOpen}
        onOpenChange={setDialogOpen}
        group={editing}
        groups={groupList}
      />
    </div>
  )
}
