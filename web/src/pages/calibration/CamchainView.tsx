import { useMemo, useState } from 'react'
import yaml from 'js-yaml'
import { ChevronDown } from 'lucide-react'
import type { KalibrCamchain } from '@/api/calibration'
import { GOOD_REPROJ_ERROR_PX } from '@/api/cameras'
import { Badge } from '@/components/ui/badge'
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from '@/components/ui/collapsible'
import { Mat4Table } from '@/components/Mat4Table'
import { isMat4, type Mat4 } from '@/lib/matrices'

// guesswork_meta block KalibrJob writes alongside the standard Kalibr keys.
// Absent for pre-enrichment manual uploads.
interface GuessworkMeta {
  reprojection_error_px?: number
}

interface CamchainSummary {
  model?: string
  distortion_model?: string
  width?: number
  height?: number
  fx?: number
  fy?: number
  cx?: number
  cy?: number
  distortion?: number[]
  reproj_error_px?: number
  // Present in camchain-imucam (extrinsics) documents.
  T_cam_imu?: Mat4
}

export function summariseCamchain(text: string | null): CamchainSummary | null {
  if (!text) return null
  let doc: (KalibrCamchain & { guesswork_meta?: GuessworkMeta }) | undefined
  try {
    doc = yaml.load(text) as KalibrCamchain & { guesswork_meta?: GuessworkMeta }
  } catch {
    return null
  }
  const c = doc?.cam0 as
    | (NonNullable<KalibrCamchain['cam0']> & { T_cam_imu?: unknown })
    | undefined
  if (!c) return null
  const intr = c.intrinsics ?? []
  const res = c.resolution ?? []
  const meta = doc?.guesswork_meta
  return {
    model: c.camera_model,
    distortion_model: c.distortion_model,
    width: typeof res[0] === 'number' ? res[0] : undefined,
    height: typeof res[1] === 'number' ? res[1] : undefined,
    fx: typeof intr[0] === 'number' ? intr[0] : undefined,
    fy: typeof intr[1] === 'number' ? intr[1] : undefined,
    cx: typeof intr[2] === 'number' ? intr[2] : undefined,
    cy: typeof intr[3] === 'number' ? intr[3] : undefined,
    distortion: Array.isArray(c.distortion_coeffs) ? c.distortion_coeffs : undefined,
    reproj_error_px:
      typeof meta?.reprojection_error_px === 'number' ? meta.reprojection_error_px : undefined,
    T_cam_imu: isMat4(c.T_cam_imu) ? c.T_cam_imu : undefined,
  }
}

function Row({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-baseline justify-between gap-4 text-sm">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right font-mono text-xs tabular-nums">{value}</span>
    </div>
  )
}

export function CamchainView({ yamlText }: { yamlText: string }) {
  const summary = useMemo(() => summariseCamchain(yamlText), [yamlText])
  const [rawOpen, setRawOpen] = useState(false)

  if (!summary) {
    return (
      <p className="text-sm text-muted-foreground">
        Could not parse this camchain — view the raw YAML below.
      </p>
    )
  }

  const reproj = summary.reproj_error_px
  return (
    <div className="space-y-2">
      {reproj != null && (
        <div className="flex items-center gap-2">
          <Badge
            variant="outline"
            className={
              reproj <= GOOD_REPROJ_ERROR_PX
                ? 'border-success/50 text-success'
                : 'border-destructive/50 text-destructive'
            }
          >
            {reproj <= GOOD_REPROJ_ERROR_PX ? 'Good' : 'Poor'} — {reproj.toFixed(3)} px reprojection σ
          </Badge>
        </div>
      )}
      <Row label="Model" value={`${summary.model ?? '?'} / ${summary.distortion_model ?? '?'}`} />
      {summary.width != null && summary.height != null && (
        <Row label="Resolution" value={`${summary.width} × ${summary.height}`} />
      )}
      {summary.fx != null && (
        <Row
          label="Intrinsics (fx fy cx cy)"
          value={[summary.fx, summary.fy, summary.cx, summary.cy]
            .map((v) => (v != null ? v.toFixed(2) : '?'))
            .join('  ')}
        />
      )}
      {summary.distortion && (
        <Row label="Distortion" value={summary.distortion.map((d) => d.toFixed(5)).join('  ')} />
      )}
      {summary.T_cam_imu && (
        <div className="pt-1">
          <p className="mb-1 text-sm text-muted-foreground">T_cam_imu</p>
          <Mat4Table matrix={summary.T_cam_imu} />
        </div>
      )}
      <Collapsible open={rawOpen} onOpenChange={setRawOpen}>
        <CollapsibleTrigger className="flex items-center gap-1 text-xs text-muted-foreground hover:text-foreground">
          <ChevronDown className={`size-3 transition-transform ${rawOpen ? 'rotate-180' : ''}`} />
          Raw YAML
        </CollapsibleTrigger>
        <CollapsibleContent>
          <pre className="mt-2 max-h-64 overflow-auto rounded-md border bg-background/60 p-3 font-mono text-xs text-muted-foreground">
            {yamlText}
          </pre>
        </CollapsibleContent>
      </Collapsible>
    </div>
  )
}
