import { useEffect, useRef, useState } from 'react'
import { Loader2, RefreshCcw, VideoOff } from 'lucide-react'
import { postOffer } from '@/api/stream'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'

type ConnState = 'idle' | 'connecting' | 'streaming' | 'failed'

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
  className,
}: {
  cameraId: number
  // CSS aspect-ratio, e.g. from mode_width/mode_height.
  aspectRatio?: string
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

  return (
    <div
      className={cn('relative overflow-hidden rounded-lg border bg-black', className)}
      style={{ aspectRatio }}
      data-testid="webrtc-player"
      data-stream-state={state}
    >
      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className="h-full w-full object-contain"
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
