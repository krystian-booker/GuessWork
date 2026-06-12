import { useCallback, useRef, useState } from 'react'
import type { Camera, CameraSettingsPatch, SettingRange } from '@/api/cameras'
import { Label } from '@/components/ui/label'
import { Slider } from '@/components/ui/slider'
import { Switch } from '@/components/ui/switch'
import { useCameraLimits, useUpdateCamera } from '@/queries/cameras'
import { useDebouncedCallback } from '@/util/debounce'

function stepFor(r: SettingRange): number {
  const span = r.max - r.min
  return span <= 0 ? 1 : span / 1000
}

function formatValue(v: number | null | undefined, unit: string): string {
  if (v == null) return '?'
  const abs = Math.abs(v)
  const digits = abs >= 100 ? 0 : abs >= 10 ? 1 : abs >= 1 ? 2 : 3
  return `${v.toFixed(digits)}${unit ? ' ' + unit : ''}`
}

// Live gain/exposure controls. Slider drags overlay a pending patch so the
// UI feels instant; the debounced PUT replaces it with the server's
// authoritative copy on response.
export function CameraSettingsPanel({ camera }: { camera: Camera }) {
  const limits = useCameraLimits(camera.id, camera.online)
  const update = useUpdateCamera(camera.id)

  const [pending, setPending] = useState<CameraSettingsPatch>({})
  const patchRef = useRef<CameraSettingsPatch>({})

  const sendPatch = useCallback(() => {
    const body = patchRef.current
    patchRef.current = {}
    if (Object.keys(body).length === 0) return
    update.mutate(body, { onSettled: () => setPending({}) })
  }, [update])

  const debouncedSend = useDebouncedCallback(sendPatch, 150)

  const queue = useCallback(
    (patch: CameraSettingsPatch) => {
      setPending((p) => ({ ...p, ...patch }))
      patchRef.current = { ...patchRef.current, ...patch }
      debouncedSend()
    },
    [debouncedSend],
  )

  const eff = {
    gain_auto: pending.gain_auto ?? camera.gain_auto,
    gain: pending.gain ?? camera.gain,
    exposure_auto: pending.exposure_auto ?? camera.exposure_auto,
    exposure: pending.exposure ?? camera.exposure,
  }

  return (
    <div className="space-y-4">
      {!camera.online && (
        <p className="text-xs text-muted-foreground">
          Camera offline — settings can't be adjusted until it comes online.
        </p>
      )}
      <SettingSection
        label="Gain"
        autoValue={eff.gain_auto}
        value={eff.gain}
        range={limits.data?.gain ?? null}
        online={camera.online}
        onAutoChange={(v) => queue({ gain_auto: v })}
        onValueChange={(v) => queue({ gain: v })}
      />
      <SettingSection
        label="Exposure"
        autoValue={eff.exposure_auto}
        value={eff.exposure}
        range={limits.data?.exposure ?? null}
        online={camera.online}
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
  const sliderValue =
    range && value != null ? Math.max(range.min, Math.min(range.max, value)) : (range?.min ?? 0)
  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between">
        <Label className="text-sm">{label}</Label>
        <label className="flex items-center gap-2 text-xs text-muted-foreground">
          Auto
          <Switch
            checked={autoValue === true}
            disabled={!online}
            onCheckedChange={onAutoChange}
            aria-label={`${label} auto`}
          />
        </label>
      </div>
      <div className="flex items-center gap-3">
        <Slider
          className="flex-1"
          min={range?.min ?? 0}
          max={range?.max ?? 1}
          step={range ? stepFor(range) : 1}
          value={[sliderValue]}
          disabled={sliderDisabled}
          onValueChange={([v]) => onValueChange(v)}
          aria-label={label}
        />
        <span className="w-24 text-right font-mono text-xs tabular-nums text-muted-foreground">
          {formatValue(value, range?.unit ?? '')}
        </span>
      </div>
    </div>
  )
}
