import { useEffect, useState } from 'react'
import { toast } from 'sonner'
import { ChevronDown, Circle, Square } from 'lucide-react'
import { useQueryClient } from '@tanstack/react-query'
import type { Camera } from '@/api/cameras'
import type { JobState } from '@/api/calibration'
import { EXTRINSICS_JOB_LOG_URL } from '@/api/extrinsics'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Checkbox } from '@/components/ui/checkbox'
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from '@/components/ui/collapsible'
import { ConfirmButton } from '@/components/ConfirmButton'
import { LogViewer } from '@/components/LogViewer'
import { PageHeader } from '@/components/PageHeader'
import { StatusDot } from '@/components/StatusDot'
import { useEventSource } from '@/hooks/use-event-source'
import { formatCount, formatElapsedMs, formatUnixSeconds } from '@/lib/format'
import { useCameras } from '@/queries/cameras'
import {
  useCameraExtrinsics,
  useCancelExtrinsicsJob,
  useDeleteCameraExtrinsics,
  useExtrinsicsJob,
  useExtrinsicsRecording,
  useStartExtrinsicsRecording,
  useStopExtrinsicsRecording,
} from '@/queries/extrinsics'
import { useHwSyncStatus } from '@/queries/hardwareSync'
import { useImuStatus } from '@/queries/imu'
import { qk } from '@/queries/keys'
import { CamchainView } from './CamchainView'

function PreflightRow({ ok, label, detail }: { ok: boolean; label: string; detail?: string }) {
  return (
    <div className="flex items-center gap-2 text-sm">
      <StatusDot tone={ok ? 'good' : 'bad'} />
      <span>{label}</span>
      {detail && <span className="text-xs text-muted-foreground">{detail}</span>}
    </div>
  )
}

// One camera's stored extrinsics: lazy-loads the YAML when expanded.
function ExtrinsicsResult({ camera }: { camera: Camera }) {
  const [open, setOpen] = useState(false)
  const extrinsics = useCameraExtrinsics(camera.id)
  const del = useDeleteCameraExtrinsics()

  return (
    <Collapsible open={open} onOpenChange={setOpen} className="rounded-md border px-3 py-2">
      <CollapsibleTrigger className="flex w-full items-center justify-between text-sm">
        <span className="flex items-center gap-2">
          <ChevronDown className={`size-3 transition-transform ${open ? 'rotate-180' : ''}`} />
          {camera.name}
        </span>
        <Badge variant="outline">
          {camera.extrinsics_calibrated_at
            ? formatUnixSeconds(camera.extrinsics_calibrated_at)
            : 'none'}
        </Badge>
      </CollapsibleTrigger>
      <CollapsibleContent className="space-y-2 pt-3">
        {extrinsics.data?.extrinsics ? (
          <>
            <CamchainView yamlText={extrinsics.data.extrinsics} />
            <ConfirmButton
              size="sm"
              variant="outline"
              title={`Delete extrinsics for "${camera.name}"?`}
              description="VIO and AprilTag publishing fall back to intrinsics-only behavior."
              confirmLabel="Delete"
              onConfirm={() =>
                del.mutate(camera.id, {
                  onSuccess: () => toast.success('Extrinsics deleted'),
                })
              }
            >
              Delete extrinsics
            </ConfirmButton>
          </>
        ) : (
          <p className="text-sm text-muted-foreground">
            {extrinsics.isLoading ? 'Loading…' : 'No stored extrinsics for this camera.'}
          </p>
        )}
      </CollapsibleContent>
    </Collapsible>
  )
}

export default function ExtrinsicsPage() {
  const qc = useQueryClient()
  const cameras = useCameras()
  const imu = useImuStatus()
  const hwSync = useHwSyncStatus()
  const recording = useExtrinsicsRecording()
  const job = useExtrinsicsJob()

  const start = useStartExtrinsicsRecording()
  const stop = useStopExtrinsicsRecording()
  const cancel = useCancelExtrinsicsJob()

  // Pre-check the VIO pair — the most common extrinsics target.
  const [selected, setSelected] = useState<Set<number> | null>(null)
  const cams = cameras.data ?? []
  const effective =
    selected ??
    new Set(cams.filter((c) => c.role === 'vio_left' || c.role === 'vio_right').map((c) => c.id))

  const [log, setLog] = useState('')
  const isRecording = recording.data != null
  const jobState = job.data?.state
  const jobActive = jobState === 'pending' || jobState === 'running'

  // Mirror of the intrinsics wizard: a dropped SSE must not leave the stored
  // results stale — the job poll's terminal transition also refreshes them.
  useEffect(() => {
    if (jobState === 'succeeded' || jobState === 'failed' || jobState === 'cancelled') {
      // Prefix match covers the camera list and every per-camera subkey,
      // including ['cameras', id, 'extrinsics'].
      void qc.invalidateQueries({ queryKey: qk.cameras })
    }
  }, [jobState, qc])

  useEventSource(jobActive ? EXTRINSICS_JOB_LOG_URL : null, {
    onMessage: (line) => setLog((l) => l + line + '\n'),
    namedEvents: {
      done: (data) => {
        try {
          const summary = JSON.parse(data) as { state: JobState; upload_error: string | null }
          if (summary.state === 'succeeded') toast.success('Extrinsics stored')
          else if (summary.state === 'failed')
            toast.error(summary.upload_error ?? 'Kalibr failed — see log')
        } catch {
          // job poll picks up the terminal state
        }
        void qc.invalidateQueries({ queryKey: qk.extrinsicsJob })
        void qc.invalidateQueries({ queryKey: qk.cameras })
        for (const id of effective) void qc.invalidateQueries({ queryKey: qk.cameraExtrinsics(id) })
      },
    },
  })

  const toggle = (id: number) => {
    const next = new Set(effective)
    if (next.has(id)) next.delete(id)
    else next.add(id)
    setSelected(next)
  }

  const hwSyncOk = cams.length > 0 && [...effective].every(
    (id) => cams.find((c) => c.id === id)?.hardware_sync_enabled,
  )

  return (
    <div>
      <PageHeader
        title="Camera–IMU extrinsics"
        description="Records the selected cameras + IMU into one bag on the sync controller clock, then runs kalibr_calibrate_imu_camera."
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Preflight</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 px-4">
              <PreflightRow
                ok={imu.data?.imu_ok ?? false}
                label="IMU healthy"
                detail={imu.data ? `${imu.data.rate_hz.toFixed(0)} Hz` : undefined}
              />
              <PreflightRow
                ok={hwSync.data?.armed ?? false}
                label="Hardware sync armed"
                detail="cameras must be pulsed on the sync controller clock"
              />
              <PreflightRow ok={hwSyncOk} label="Selected cameras have hw-sync enabled" />
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Cameras to record</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 px-4">
              {cams.map((c) => (
                <label key={c.id} className="flex items-center gap-3 text-sm">
                  <Checkbox
                    checked={effective.has(c.id)}
                    disabled={isRecording || jobActive}
                    onCheckedChange={() => toggle(c.id)}
                  />
                  <StatusDot tone={c.online ? 'good' : 'bad'} />
                  {c.name}
                  {c.role && (
                    <Badge variant="secondary" className="text-[11px]">
                      {c.role}
                    </Badge>
                  )}
                </label>
              ))}
              {cams.length === 0 && (
                <p className="text-sm text-muted-foreground">No cameras registered.</p>
              )}

              {!isRecording ? (
                <Button
                  className="mt-2"
                  disabled={effective.size === 0 || start.isPending || jobActive}
                  onClick={() =>
                    start.mutate([...effective], { onSuccess: () => setLog('') })
                  }
                >
                  <Circle className="fill-destructive text-destructive" /> Start recording
                </Button>
              ) : (
                <div className="mt-2 space-y-3">
                  <div className="flex items-center gap-4 font-mono text-sm tabular-nums">
                    <span className="flex items-center gap-2">
                      <span className="size-2 animate-pulse rounded-full bg-destructive" />
                      {formatElapsedMs(recording.data!.elapsed_ms)}
                    </span>
                    <span>{formatCount(recording.data!.imu_written)} IMU samples</span>
                  </div>
                  <div className="space-y-1 text-xs text-muted-foreground">
                    {recording.data!.cameras.map((c) => (
                      <div key={c.camera_id} className="font-mono tabular-nums">
                        {c.topic}: {formatCount(c.frames_written)} frames
                        {c.frames_dropped > 0 ? ` (${c.frames_dropped} dropped)` : ''}
                      </div>
                    ))}
                  </div>
                  <p className="text-xs text-muted-foreground">
                    Excite all IMU axes: rotate about each axis and translate along each — smooth,
                    no shocks. ~90 seconds.
                  </p>
                  <Button
                    variant="secondary"
                    disabled={stop.isPending}
                    onClick={() =>
                      stop.mutate(undefined, {
                        onSuccess: (resp) => {
                          setLog('')
                          if (resp.job_error) toast.error(resp.job_error)
                        },
                      })
                    }
                  >
                    <Square /> Stop &amp; calibrate
                  </Button>
                </div>
              )}
            </CardContent>
          </Card>

          {job.data && (
            <Card className="py-4 gap-3">
              <CardHeader className="px-4">
                <CardTitle className="flex items-center gap-2 text-sm">
                  Kalibr IMU job
                  <Badge variant="outline" className="font-mono text-[11px]">
                    {job.data.state}
                  </Badge>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3 px-4">
                <LogViewer text={log} />
                {jobActive && (
                  <ConfirmButton
                    size="sm"
                    variant="outline"
                    title="Cancel the extrinsics job?"
                    description="The recorded bag stays on disk for a manual re-run."
                    confirmLabel="Cancel job"
                    onConfirm={() => cancel.mutate()}
                  >
                    Cancel job
                  </ConfirmButton>
                )}
              </CardContent>
            </Card>
          )}
        </div>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="text-sm">Stored extrinsics</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 px-4">
            {cams.map((c) => (
              <ExtrinsicsResult key={c.id} camera={c} />
            ))}
            {cams.length === 0 && (
              <p className="text-sm text-muted-foreground">No cameras registered.</p>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
