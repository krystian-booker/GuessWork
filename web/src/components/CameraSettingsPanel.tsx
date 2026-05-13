import { useCallback, useEffect, useRef, useState } from 'react'
import {
  CameraOfflineError,
  getCamera,
  getCameraSettingsLimits,
  updateCamera,
  type Camera,
  type CameraSettingsLimits,
  type CameraSettingsPatch,
  type SettingRange,
} from '../api/cameras'
import { useDebouncedCallback } from '../util/debounce'

const cardStyle: React.CSSProperties = {
  background: '#fff',
  border: '1px solid #e5e7eb',
  borderRadius: 8,
  padding: 16,
  display: 'flex',
  flexDirection: 'column',
  gap: 14,
}

const sectionLabelStyle: React.CSSProperties = {
  fontSize: 13,
  fontWeight: 600,
  color: '#374151',
  marginBottom: 6,
}

const toggleRowStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: 8,
  marginBottom: 6,
  fontSize: 13,
  color: '#374151',
}

const sliderRowStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: 8,
}

const sliderStyle: React.CSSProperties = {
  flex: 1,
  minWidth: 0,
}

const valueStyle: React.CSSProperties = {
  fontSize: 12,
  color: '#374151',
  width: 96,
  textAlign: 'right',
  fontVariantNumeric: 'tabular-nums',
}

function stepFor(r: SettingRange): number {
  const span = r.max - r.min
  if (span <= 0) return 1
  return span / 1000
}

function formatValue(v: number | null, unit: string): string {
  if (v == null) return '?'
  const abs = Math.abs(v)
  let digits = 3
  if (abs >= 100) digits = 0
  else if (abs >= 10) digits = 1
  else if (abs >= 1) digits = 2
  return `${v.toFixed(digits)}${unit ? ' ' + unit : ''}`
}

type Props = {
  cameraId: number | null
}

export default function CameraSettingsPanel({ cameraId }: Props) {
  const [camera, setCamera] = useState<Camera | null>(null)
  const [limits, setLimits] = useState<CameraSettingsLimits | null>(null)
  const [error, setError] = useState<string | null>(null)

  // Pending overlay so slider drags feel instant. Cleared when the PUT
  // response arrives and we replace state with the server's authoritative copy.
  const [pending, setPending] = useState<CameraSettingsPatch>({})
  const patchRef = useRef<CameraSettingsPatch>({})

  useEffect(() => {
    if (cameraId == null) {
      setCamera(null)
      setLimits(null)
      setError(null)
      return
    }
    let cancelled = false
    setError(null)
    setPending({})
    patchRef.current = {}
    ;(async () => {
      try {
        const [c, lim] = await Promise.all([
          getCamera(cameraId),
          getCameraSettingsLimits(cameraId).catch((e) => {
            if (e instanceof CameraOfflineError) return null
            throw e
          }),
        ])
        if (cancelled) return
        setCamera(c)
        setLimits(lim)
      } catch (e) {
        if (!cancelled) setError(e instanceof Error ? e.message : String(e))
      }
    })()
    return () => {
      cancelled = true
    }
  }, [cameraId])

  const sendPatch = useCallback(async () => {
    if (cameraId == null) return
    const body = patchRef.current
    patchRef.current = {}
    if (Object.keys(body).length === 0) return
    try {
      const updated = await updateCamera(cameraId, body)
      setCamera(updated)
      setPending({})
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    }
  }, [cameraId])

  const debouncedSend = useDebouncedCallback(sendPatch, 100)

  const queue = useCallback(
    (patch: CameraSettingsPatch) => {
      setPending((p) => ({ ...p, ...patch }))
      patchRef.current = { ...patchRef.current, ...patch }
      debouncedSend()
    },
    [debouncedSend],
  )

  if (cameraId == null) {
    return (
      <div style={cardStyle}>
        <p style={{ margin: 0, color: '#6b7280', fontSize: 13 }}>
          No camera selected.
        </p>
      </div>
    )
  }

  if (!camera) {
    return (
      <div style={cardStyle}>
        <p style={{ margin: 0, color: '#6b7280', fontSize: 13 }}>Loading settings…</p>
      </div>
    )
  }

  const eff = {
    gain_auto:     pending.gain_auto     ?? camera.gain_auto,
    gain:          pending.gain          ?? camera.gain,
    exposure_auto: pending.exposure_auto ?? camera.exposure_auto,
    exposure:      pending.exposure      ?? camera.exposure,
  }

  const online = camera.online

  return (
    <div style={cardStyle}>
      <h2 style={{ margin: 0, fontSize: 16 }}>Camera Settings</h2>
      {error && (
        <p style={{ margin: 0, color: 'crimson', fontSize: 12 }}>{error}</p>
      )}
      {!online && (
        <p style={{ margin: 0, color: '#6b7280', fontSize: 12 }}>
          Camera offline. Settings cannot be adjusted until the camera comes online.
        </p>
      )}

      <SettingSection
        label="Gain"
        autoValue={eff.gain_auto}
        value={eff.gain}
        range={limits?.gain ?? null}
        online={online}
        onAutoChange={(v) => queue({ gain_auto: v })}
        onValueChange={(v) => queue({ gain: v })}
      />
      <SettingSection
        label="Exposure"
        autoValue={eff.exposure_auto}
        value={eff.exposure}
        range={limits?.exposure ?? null}
        online={online}
        onAutoChange={(v) => queue({ exposure_auto: v })}
        onValueChange={(v) => queue({ exposure: v })}
      />
    </div>
  )
}

function SettingSection({
  label,
  autoValue,
  value,
  range,
  online,
  onAutoChange,
  onValueChange,
}: {
  label: string
  autoValue: boolean | null | undefined
  value: number | null | undefined
  range: SettingRange | null
  online: boolean
  onAutoChange: (v: boolean) => void
  onValueChange: (v: number) => void
}) {
  const sliderDisabled = !online || autoValue !== false || !range
  return (
    <div>
      <div style={sectionLabelStyle}>{label}</div>
      <label style={toggleRowStyle}>
        <input
          type="checkbox"
          checked={autoValue === true}
          disabled={!online}
          onChange={(e) => onAutoChange(e.target.checked)}
        />
        Auto (Continuous)
      </label>
      <RangeRow
        value={value ?? null}
        range={range}
        disabled={sliderDisabled}
        onChange={onValueChange}
      />
    </div>
  )
}

function RangeRow({
  value,
  range,
  disabled,
  onChange,
}: {
  value: number | null
  range: SettingRange | null
  disabled: boolean
  onChange: (v: number) => void
}) {
  const sliderValue =
    range && value != null
      ? Math.max(range.min, Math.min(range.max, value))
      : range?.min ?? 0
  return (
    <div style={sliderRowStyle}>
      <input
        type="range"
        style={sliderStyle}
        min={range?.min ?? 0}
        max={range?.max ?? 1}
        step={range ? stepFor(range) : 1}
        value={sliderValue}
        disabled={disabled || !range}
        onChange={(e) => onChange(Number(e.target.value))}
      />
      <span style={valueStyle}>{formatValue(value, range?.unit ?? '')}</span>
    </div>
  )
}
