import { useEffect, useRef, useState } from 'react'
import { Loader2, RefreshCcw, VideoOff } from 'lucide-react'
import type { CameraOrientation } from '@/api/cameras'
import { postOffer } from '@/api/stream'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'

type ConnState = 'idle' | 'connecting' | 'streaming' | 'failed'

// "2048 / 1536" → 2048/1536. Falls back to 4:3 on anything unparsable.
function parseAspect(aspect: string): number {
  const [w, h] = aspect.split('/').map((s) => Number(s.trim()))
  return Number.isFinite(w) && Number.isFinite(h) && w > 0 && h > 0 ? w / h : 4 / 3
}

// Non-trickle signaling: libdatachannel also blocks on gathering complete.
function waitForIceComplete(pc: RTCPeerConnection): Promise<void> {
  return new Promise((resolve) => {
    if (pc.iceGatheringState === 'complete') {
      resolve()
      return
    }
    const check = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', check)
        resolve()
      }
    }
    pc.addEventListener('icegatheringstatechange', check)
  })
}

export function WebRtcPlayer({
  cameraId,
  aspectRatio = '4 / 3',
  orientation = 0,
  className,
}: {
  cameraId: number
  // CSS aspect-ratio of the SOURCE stream, e.g. from mode_width/mode_height
  // (un-rotated — the player handles the 90/270 swap itself).
  aspectRatio?: string
  // Mounting rotation applied as a pure CSS transform (display-only; the
  // stream itself stays sensor-oriented). 90/270 swap the container aspect.
  // (On-sensor ReverseX/Y flip was tried and is a firmware no-op on the
  // Chameleon3 — see spinnaker_producer.cpp.)
  orientation?: CameraOrientation
  className?: string
}) {
  const videoRef = useRef<HTMLVideoElement>(null)
  const [state, setState] = useState<ConnState>('idle')
  const [error, setError] = useState<string | null>(null)
  const [attempt, setAttempt] = useState(0)

  useEffect(() => {
    let pc: RTCPeerConnection | null = null
    let cancelled = false
    const w = window as unknown as { __pc?: RTCPeerConnection | null }

    const start = async () => {
      setState('connecting')
      setError(null)

      pc = new RTCPeerConnection()
      w.__pc = pc // E2E test introspection (getStats etc.)
      pc.addTransceiver('video', { direction: 'recvonly' })

      pc.ontrack = (e) => {
        if (videoRef.current && e.streams[0]) {
          videoRef.current.srcObject = e.streams[0]
        }
      }

      pc.onconnectionstatechange = () => {
        if (!pc) return
        if (pc.connectionState === 'connected') setState('streaming')
        else if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
          setState('failed')
        }
      }

      try {
        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await waitForIceComplete(pc)
        if (cancelled) return

        const answerSdp = await postOffer(cameraId, pc.localDescription!.sdp)
        await pc.setRemoteDescription({ type: 'answer', sdp: answerSdp })
      } catch (e) {
        if (!cancelled) {
          setError(e instanceof Error ? e.message : String(e))
          setState('failed')
        }
      }
    }

    void start()

    return () => {
      cancelled = true
      w.__pc = null
      if (pc) {
        pc.ontrack = null
        pc.onconnectionstatechange = null
        pc.close()
        pc = null
      }
    }
  }, [cameraId, attempt])

  // For 90/270 the container takes the rotated (swapped) aspect, and the
  // video is centered at the container's swapped dimensions so the rotated
  // content fills it exactly (see the width/aspect math below).
  const sourceRatio = parseAspect(aspectRatio)
  const sideways = orientation === 90 || orientation === 270
  const containerAspect = sideways ? String(1 / sourceRatio) : aspectRatio

  return (
    <div
      className={cn('relative overflow-hidden rounded-lg border bg-black', className)}
      style={{ aspectRatio: containerAspect }}
      data-testid="webrtc-player"
      data-stream-state={state}
      data-orientation={orientation}
    >
      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className={cn(
          'object-contain',
          sideways ? 'absolute left-1/2 top-1/2' : 'h-full w-full',
        )}
        style={
          sideways
            ? {
                // Pre-rotation width = container height (containerWidth ×
                // sourceRatio, since the container aspect is 1/sourceRatio);
                // height follows from the source aspect. After rotation the
                // video exactly fills the container.
                width: `${sourceRatio * 100}%`,
                aspectRatio: String(sourceRatio),
                transform: `translate(-50%, -50%) rotate(${orientation}deg)`,
              }
            : orientation === 180
              ? { transform: 'rotate(180deg)' }
              : undefined
        }
      />
      {state === 'connecting' && (
        <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 text-muted-foreground">
          <Loader2 className="size-6 animate-spin" />
          <span className="text-xs">Connecting…</span>
        </div>
      )}
      {state === 'failed' && (
        <div className="absolute inset-0 flex flex-col items-center justify-center gap-3 bg-background/80 p-4 text-center">
          <VideoOff className="size-6 text-destructive" />
          <p className="max-w-xs text-xs text-muted-foreground">{error ?? 'Stream disconnected'}</p>
          <Button size="sm" variant="secondary" onClick={() => setAttempt((n) => n + 1)}>
            <RefreshCcw /> Retry
          </Button>
        </div>
      )}
    </div>
  )
}
