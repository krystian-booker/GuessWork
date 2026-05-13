import { useEffect, useRef, useState } from 'react'
import { postOffer } from './api/stream'

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

interface StreamProps {
  cameraId: number
}

export default function Stream({ cameraId }: StreamProps) {
  const videoRef = useRef<HTMLVideoElement>(null)
  const [state, setState] = useState<ConnState>('idle')
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let pc: RTCPeerConnection | null = null
    let cancelled = false
    const w = window as unknown as { __pc?: RTCPeerConnection | null }

    const start = async () => {
      setState('connecting')
      setError(null)

      pc = new RTCPeerConnection()
      w.__pc = pc  // E2E test introspection (getStats etc.)
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

    start()

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
  }, [cameraId])

  return (
    <div style={{ marginBottom: 24 }}>
      <h2 style={{ marginBottom: 8 }}>Live camera</h2>
      <div style={{
        background: '#111',
        borderRadius: 8,
        overflow: 'hidden',
        aspectRatio: '4 / 3',
        maxWidth: 720,
      }}>
        <video
          ref={videoRef}
          autoPlay
          playsInline
          muted
          style={{ width: '100%', height: '100%', objectFit: 'contain' }}
        />
      </div>
      <p style={{ marginTop: 8, color: '#666', fontSize: 13 }}>
        State: <strong>{state}</strong>
        {error && <span style={{ color: 'crimson', marginLeft: 12 }}>{error}</span>}
      </p>
    </div>
  )
}
