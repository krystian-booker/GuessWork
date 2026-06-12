import { useEffect, useState } from 'react'
import { Link, useParams } from 'react-router-dom'
import { toast } from 'sonner'
import { ArrowLeft, Circle, Square } from 'lucide-react'
import type { JobState } from '@/api/calibration'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from '@/components/ui/collapsible'
import { Textarea } from '@/components/ui/textarea'
import { ConfirmButton } from '@/components/ConfirmButton'
import { LogViewer } from '@/components/LogViewer'
import { PageHeader } from '@/components/PageHeader'
import { WebRtcPlayer } from '@/components/stream/WebRtcPlayer'
import { useEventSource } from '@/hooks/use-event-source'
import { formatCount, formatElapsedMs, formatUnixSeconds } from '@/lib/format'
import { cn } from '@/lib/utils'
import { useCamera } from '@/queries/cameras'
import {
  useCalibration,
  useCalibrationJob,
  useCalibrationRecording,
  useCancelJob,
  useStartRecording,
  useStopRecording,
  useUploadCalibration,
} from '@/queries/calibration'
import { useQueryClient } from '@tanstack/react-query'
import { qk } from '@/queries/keys'
import { CamchainView } from './CamchainView'

const JOB_BADGE: Record<JobState, string> = {
  pending: 'border-muted-foreground/50 text-muted-foreground',
  running: 'border-chart-2/60 text-chart-2',
  succeeded: 'border-success/50 text-success',
  failed: 'border-destructive/50 text-destructive',
  cancelled: 'border-warning/50 text-warning',
}

function StepDot({ n, active, done }: { n: number; active: boolean; done: boolean }) {
  return (
    <span
      className={cn(
        'flex size-6 items-center justify-center rounded-full border font-mono text-xs',
        done && 'border-primary bg-primary text-primary-foreground',
        active && !done && 'border-primary text-primary',
        !active && !done && 'border-border text-muted-foreground',
      )}
    >
      {n}
    </span>
  )
}

export default function IntrinsicsWizardPage() {
  const { id } = useParams()
  const cameraId = Number(id)
  const qc = useQueryClient()

  const camera = useCamera(cameraId)
  const recording = useCalibrationRecording(cameraId)
  const job = useCalibrationJob(cameraId)
  const calibration = useCalibration(cameraId)

  const start = useStartRecording(cameraId)
  const stop = useStopRecording(cameraId)
  const cancel = useCancelJob(cameraId)
  const upload = useUploadCalibration(cameraId)

  const [log, setLog] = useState('')
  const [manualYaml, setManualYaml] = useState('')

  const isRecording = recording.data != null
  const jobState = job.data?.state
  const jobActive = jobState === 'pending' || jobState === 'running'

  // Step derivation from server state — refresh-safe.
  const step = isRecording ? 1 : jobActive ? 2 : 0

  // The SSE done event is the primary completion signal, but if the stream
  // drops, the job poll still observes the terminal transition — refresh the
  // stored calibration then too so the result card never goes stale.
  useEffect(() => {
    if (jobState === 'succeeded' || jobState === 'failed' || jobState === 'cancelled') {
      void qc.invalidateQueries({ queryKey: qk.cameraCalibration(cameraId) })
      void qc.invalidateQueries({ queryKey: qk.cameras })
    }
  }, [jobState, cameraId, qc])

  useEventSource(
    jobActive ? `/api/cameras/${cameraId}/calibration/job/log` : null,
    {
      onMessage: (line) => setLog((l) => l + line + '\n'),
      namedEvents: {
        done: (data) => {
          try {
            const summary = JSON.parse(data) as {
              state: JobState
              calibration_stored: boolean
              upload_error: string | null
            }
            if (summary.state === 'succeeded' && summary.calibration_stored) {
              toast.success('Calibration stored')
            } else if (summary.state === 'failed') {
              toast.error(summary.upload_error ?? 'Kalibr failed — see log')
            }
          } catch {
            // malformed done payload — the job poll will pick up the state
          }
          void qc.invalidateQueries({ queryKey: qk.cameraJob(cameraId) })
          void qc.invalidateQueries({ queryKey: qk.cameraCalibration(cameraId) })
          void qc.invalidateQueries({ queryKey: qk.cameras })
        },
      },
    },
  )

  if (camera.isLoading) return <p className="text-sm text-muted-foreground">Loading…</p>
  if (!camera.data) return <p className="text-sm text-destructive">Camera not found.</p>
  const cam = camera.data

  return (
    <div>
      <PageHeader
        title={`Intrinsics — ${cam.name}`}
        description="Record an AprilGrid sequence, then Kalibr computes the camera model in Docker."
        actions={
          <Button asChild variant="ghost" size="sm">
            <Link to="/calibration">
              <ArrowLeft /> Calibration
            </Link>
          </Button>
        }
      />

      <div className="mb-4 flex items-center gap-3 text-sm">
        <StepDot n={1} active={step === 0 || step === 1} done={step === 2 || jobActive} />
        <span className={step <= 1 ? '' : 'text-muted-foreground'}>Record</span>
        <div className="h-px w-8 bg-border" />
        <StepDot n={2} active={step === 2} done={job.data?.state === 'succeeded'} />
        <span className={step === 2 ? '' : 'text-muted-foreground'}>Kalibr</span>
        <div className="h-px w-8 bg-border" />
        <StepDot n={3} active={false} done={cam.calibrated_at != null} />
        <span className="text-muted-foreground">Result</span>
      </div>

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">1 · Record AprilGrid sequence</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              {!isRecording ? (
                <>
                  <p className="text-sm text-muted-foreground">
                    Hold the AprilGrid in view and move it slowly through the frame — tilt it,
                    cover the corners, vary the distance. 60–90 seconds is plenty.
                  </p>
                  <Button
                    onClick={() =>
                      start.mutate(undefined, {
                        onSuccess: () => setLog(''),
                      })
                    }
                    disabled={!cam.online || start.isPending || jobActive}
                    data-testid="start-recording"
                  >
                    <Circle className="fill-destructive text-destructive" /> Start recording
                  </Button>
                  {!cam.online && (
                    <p className="text-xs text-warning">Camera is offline — recording unavailable.</p>
                  )}
                </>
              ) : (
                <>
                  <div className="flex items-center gap-4 font-mono text-sm tabular-nums">
                    <span className="flex items-center gap-2">
                      <span className="size-2 animate-pulse rounded-full bg-destructive" />
                      {formatElapsedMs(recording.data!.elapsed_ms)}
                    </span>
                    <span>{formatCount(recording.data!.frames_written)} frames</span>
                    {recording.data!.frames_dropped > 0 && (
                      <span className="text-warning">
                        {formatCount(recording.data!.frames_dropped)} dropped
                      </span>
                    )}
                  </div>
                  <Button
                    variant="secondary"
                    onClick={() =>
                      stop.mutate(undefined, {
                        onSuccess: (resp) => {
                          setLog('')
                          if (resp.job_error) toast.error(resp.job_error)
                        },
                      })
                    }
                    disabled={stop.isPending}
                    data-testid="stop-recording"
                  >
                    <Square /> Stop &amp; calibrate
                  </Button>
                </>
              )}
            </CardContent>
          </Card>

          {(jobActive || (job.data && job.data.state !== 'succeeded')) && job.data && (
            <Card className="py-4 gap-3">
              <CardHeader className="px-4">
                <CardTitle className="flex items-center gap-2 text-sm">
                  2 · Kalibr
                  <Badge variant="outline" className={JOB_BADGE[job.data.state]}>
                    {job.data.state}
                  </Badge>
                  <Badge variant="outline" className="font-mono text-[11px]">
                    {job.data.model}
                  </Badge>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3 px-4">
                <LogViewer text={log} />
                {job.data.state === 'failed' && (
                  <p className="text-xs text-destructive">
                    Exit code {job.data.exit_code}
                    {job.data.upload_error ? ` — ${job.data.upload_error}` : ''}. The recorded
                    dataset is kept; you can record again or upload a camchain manually.
                  </p>
                )}
                {jobActive && (
                  <ConfirmButton
                    size="sm"
                    variant="outline"
                    title="Cancel the Kalibr job?"
                    description="The recorded dataset stays on disk for a manual re-run."
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

        <div className="space-y-4">
          {cam.online && (
            <WebRtcPlayer cameraId={cam.id} aspectRatio="4 / 3" orientation={cam.orientation} />
          )}

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Current calibration</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              {calibration.data?.calibration ? (
                <>
                  <p className="text-xs text-muted-foreground">
                    Uploaded {formatUnixSeconds(calibration.data.calibrated_at)}
                  </p>
                  <CamchainView yamlText={calibration.data.calibration} />
                </>
              ) : (
                <p className="text-sm text-muted-foreground">No calibration uploaded yet.</p>
              )}

              <Collapsible>
                <CollapsibleTrigger className="text-xs text-muted-foreground hover:text-foreground">
                  Advanced: upload camchain YAML manually
                </CollapsibleTrigger>
                <CollapsibleContent className="space-y-2 pt-2">
                  <Textarea
                    value={manualYaml}
                    onChange={(e) => setManualYaml(e.target.value)}
                    placeholder="cam0:&#10;  camera_model: pinhole&#10;  …"
                    className="min-h-32 font-mono text-xs"
                  />
                  <Button
                    size="sm"
                    variant="secondary"
                    disabled={!manualYaml.includes('cam0') || upload.isPending}
                    onClick={() =>
                      upload.mutate(manualYaml, {
                        onSuccess: () => {
                          toast.success('Calibration uploaded')
                          setManualYaml('')
                        },
                      })
                    }
                  >
                    Upload
                  </Button>
                </CollapsibleContent>
              </Collapsible>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  )
}
