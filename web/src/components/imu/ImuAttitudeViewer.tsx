import { useEffect, useRef } from 'react'
import { toast } from 'sonner'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'
import { formatHz } from '@/lib/format'
import { useImuAttitude, useZeroYaw } from '@/queries/imu'

// Live 3D preview of the BMI088 attitude: a PCB-like slab plus body-axis
// arrows, rotated by the filter quaternion (body→world, world Z-up) and
// drawn with a hand-rolled orthographic projection — no 3D library.
//
// Screen mapping (right-handed): world +Z up on screen, +X to the right and
// slightly toward the viewer, +Y into the scene. The world is first yawed by
// AZIM about Z for a 3/4 view, then tilted by TILT (~-60°) about X; the
// remaining depth axis drives painter's-algorithm face sorting (the slab is
// convex, so sorting alone is sufficient).

type Quat = { w: number; x: number; y: number; z: number }
type Vec3 = [number, number, number]

const Q_IDENTITY: Quat = { w: 1, x: 0, y: 0, z: 0 }

function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q.w, q.x, q.y, q.z)
  if (!Number.isFinite(n) || n < 1e-9) return { ...Q_IDENTITY }
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n }
}

// Shortest-path slerp. Targets arrive hemisphere-canonicalized (w >= 0) but
// the dot-sign flip keeps us safe against any interpolation crossing.
function quatSlerp(a: Quat, b: Quat, t: number): Quat {
  let dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
  let bw = b.w
  let bx = b.x
  let by = b.y
  let bz = b.z
  if (dot < 0) {
    dot = -dot
    bw = -bw
    bx = -bx
    by = -by
    bz = -bz
  }
  if (dot > 0.9995) {
    // Nearly parallel: nlerp avoids the sin(θ)→0 blow-up.
    return quatNormalize({
      w: a.w + (bw - a.w) * t,
      x: a.x + (bx - a.x) * t,
      y: a.y + (by - a.y) * t,
      z: a.z + (bz - a.z) * t,
    })
  }
  const theta = Math.acos(Math.min(dot, 1))
  const s = Math.sin(theta)
  const wa = Math.sin((1 - t) * theta) / s
  const wb = Math.sin(t * theta) / s
  return {
    w: a.w * wa + bw * wb,
    x: a.x * wa + bx * wb,
    y: a.y * wa + by * wb,
    z: a.z * wa + bz * wb,
  }
}

// Rotate a body-frame vector into the world frame: v' = v + 2q̄×(q̄×v + w·v).
function quatRotate(q: Quat, v: Vec3): Vec3 {
  const [vx, vy, vz] = v
  const { w, x, y, z } = q
  const cx = y * vz - z * vy + w * vx
  const cy = z * vx - x * vz + w * vy
  const cz = x * vy - y * vx + w * vz
  return [vx + 2 * (y * cz - z * cy), vy + 2 * (z * cx - x * cz), vz + 2 * (x * cy - y * cx)]
}

// Fixed pleasant view: yaw about Z, then tilt about X, then drop the depth
// axis (orthographic). Larger depth = closer to the viewer.
const AZIM = (-30 * Math.PI) / 180
const TILT = (-60 * Math.PI) / 180
const COS_A = Math.cos(AZIM)
const SIN_A = Math.sin(AZIM)
const COS_T = Math.cos(TILT)
const SIN_T = Math.sin(TILT)

function project(v: Vec3): { x: number; y: number; depth: number } {
  const [wx, wy, wz] = v
  const x1 = wx * COS_A - wy * SIN_A
  const y1 = wx * SIN_A + wy * COS_A
  // canvas y grows downward, so screen-up is -y2
  return { x: x1, y: -(y1 * COS_T - wz * SIN_T), depth: y1 * SIN_T + wz * COS_T }
}

// IMU board: a 1.0 × 0.7 × 0.12 slab centered on the body origin.
const HX = 0.5
const HY = 0.35
const HZ = 0.06
const CORNERS: Vec3[] = [
  [-HX, -HY, -HZ],
  [HX, -HY, -HZ],
  [HX, HY, -HZ],
  [-HX, HY, -HZ],
  [-HX, -HY, HZ],
  [HX, -HY, HZ],
  [HX, HY, HZ],
  [-HX, HY, HZ],
]
const FACES: Array<{ idx: [number, number, number, number]; kind: 'top' | 'bottom' | 'side' }> = [
  { idx: [4, 5, 6, 7], kind: 'top' },
  { idx: [0, 3, 2, 1], kind: 'bottom' },
  { idx: [0, 1, 5, 4], kind: 'side' }, // -Y
  { idx: [2, 3, 7, 6], kind: 'side' }, // +Y
  { idx: [1, 2, 6, 5], kind: 'side' }, // +X
  { idx: [3, 0, 4, 7], kind: 'side' }, // -X
]
// A small "chip" decal offset toward +X on the top face so flips and yaw are
// unambiguous at a glance.
const CHIP: Vec3[] = [
  [0.08, -0.14, HZ + 0.005],
  [0.34, -0.14, HZ + 0.005],
  [0.34, 0.14, HZ + 0.005],
  [0.08, 0.14, HZ + 0.005],
]
const AXES: Array<{ dir: Vec3; len: number; label: string }> = [
  { dir: [1, 0, 0], len: 0.8, label: '+X' },
  { dir: [0, 1, 0], len: 0.65, label: '+Y' },
  { dir: [0, 0, 1], len: 0.5, label: '+Z' },
]
const GRID_Z = -0.55 // world-frame ground reference below the board
const GRID_EXTENT = 0.9
const GRID_STEP = 0.45

interface ViewerColors {
  grid: string
  x: string
  y: string
  z: string
}

// Face/edge shades are fixed white-alpha overlays — the app theme is
// dark-only (see index.css), so these read correctly on the card background.
const FACE_TOP = 'rgba(255, 255, 255, 0.14)'
const FACE_SIDE = 'rgba(255, 255, 255, 0.06)'
const FACE_BOTTOM = 'rgba(255, 255, 255, 0.03)'
const EDGE = 'rgba(255, 255, 255, 0.30)'
const CHIP_FILL = 'rgba(0, 0, 0, 0.5)'
const CHIP_EDGE = 'rgba(255, 255, 255, 0.35)'

function drawAxis(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  s: number,
  q: Quat,
  axis: (typeof AXES)[number],
  color: string,
) {
  const tip3 = quatRotate(q, [axis.dir[0] * axis.len, axis.dir[1] * axis.len, axis.dir[2] * axis.len])
  const tip = project(tip3)
  const tx = cx + tip.x * s
  const ty = cy + tip.y * s
  const dx = tx - cx
  const dy = ty - cy
  const d = Math.hypot(dx, dy)
  if (d < 1e-3) return // axis pointing straight at the viewer — nothing to draw

  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(cx, cy)
  ctx.lineTo(tx, ty)
  ctx.stroke()

  // arrowhead
  const ux = dx / d
  const uy = dy / d
  const ah = 7
  ctx.beginPath()
  ctx.moveTo(tx, ty)
  ctx.lineTo(tx - ah * ux + ah * 0.5 * uy, ty - ah * uy - ah * 0.5 * ux)
  ctx.lineTo(tx - ah * ux - ah * 0.5 * uy, ty - ah * uy + ah * 0.5 * ux)
  ctx.closePath()
  ctx.fill()

  ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillText(axis.label, tx + 12 * ux, ty + 12 * uy)
}

function draw(canvas: HTMLCanvasElement, ctx: CanvasRenderingContext2D, q: Quat, colors: ViewerColors) {
  const dpr = window.devicePixelRatio || 1
  const cw = canvas.clientWidth
  const ch = canvas.clientHeight
  if (cw === 0 || ch === 0) return
  const pw = Math.round(cw * dpr)
  const ph = Math.round(ch * dpr)
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw
    canvas.height = ph
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, cw, ch)

  const cx = cw / 2
  const cy = ch / 2
  const s = Math.min(cw, ch) * 0.45 // world units → px

  // World-frame ground grid (unrotated) so tilt reads instantly.
  ctx.strokeStyle = colors.grid
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let g = -GRID_EXTENT; g <= GRID_EXTENT + 1e-6; g += GRID_STEP) {
    const a = project([g, -GRID_EXTENT, GRID_Z])
    const b = project([g, GRID_EXTENT, GRID_Z])
    ctx.moveTo(cx + a.x * s, cy + a.y * s)
    ctx.lineTo(cx + b.x * s, cy + b.y * s)
    const c = project([-GRID_EXTENT, g, GRID_Z])
    const d = project([GRID_EXTENT, g, GRID_Z])
    ctx.moveTo(cx + c.x * s, cy + c.y * s)
    ctx.lineTo(cx + d.x * s, cy + d.y * s)
  }
  ctx.stroke()

  // Body geometry, rotated then projected once.
  const proj = CORNERS.map((v) => project(quatRotate(q, v)))

  // Axes pointing away from the viewer go behind the slab.
  const axisFront: Array<{ axis: (typeof AXES)[number]; color: string }> = []
  const axisColors = [colors.x, colors.y, colors.z]
  AXES.forEach((axis, i) => {
    const tipDepth = project(quatRotate(q, axis.dir)).depth
    if (tipDepth < 0) drawAxis(ctx, cx, cy, s, q, axis, axisColors[i])
    else axisFront.push({ axis, color: axisColors[i] })
  })

  // Painter's algorithm: far faces (small depth) first.
  const order = FACES.map((face) => ({
    face,
    depth: face.idx.reduce((acc, i) => acc + proj[i].depth, 0) / 4,
  })).sort((a, b) => a.depth - b.depth)

  for (const { face } of order) {
    ctx.beginPath()
    face.idx.forEach((i, k) => {
      const p = proj[i]
      if (k === 0) ctx.moveTo(cx + p.x * s, cy + p.y * s)
      else ctx.lineTo(cx + p.x * s, cy + p.y * s)
    })
    ctx.closePath()
    ctx.fillStyle = face.kind === 'top' ? FACE_TOP : face.kind === 'bottom' ? FACE_BOTTOM : FACE_SIDE
    ctx.fill()
    ctx.strokeStyle = EDGE
    ctx.lineWidth = 1
    ctx.stroke()

    if (face.kind === 'top') {
      // Chip decal rides the top face in paint order.
      ctx.beginPath()
      CHIP.forEach((v, k) => {
        const p = project(quatRotate(q, v))
        if (k === 0) ctx.moveTo(cx + p.x * s, cy + p.y * s)
        else ctx.lineTo(cx + p.x * s, cy + p.y * s)
      })
      ctx.closePath()
      ctx.fillStyle = CHIP_FILL
      ctx.fill()
      ctx.strokeStyle = CHIP_EDGE
      ctx.lineWidth = 1
      ctx.stroke()
    }
  }

  for (const { axis, color } of axisFront) drawAxis(ctx, cx, cy, s, q, axis, color)
}

function fmtDegValue(v: number | undefined): string {
  return v == null || !Number.isFinite(v) ? '—' : `${v.toFixed(1)}°`
}

function fmtMagnitude(v: { x: number; y: number; z: number } | undefined, unit: string, digits: number): string {
  return v == null ? '—' : `${Math.hypot(v.x, v.y, v.z).toFixed(digits)} ${unit}`
}

function Readout({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between gap-2 text-xs">
      <span className="text-muted-foreground">{label}</span>
      <span className="font-mono tabular-nums">{value}</span>
    </div>
  )
}

export function ImuAttitudeViewer() {
  const attitude = useImuAttitude()
  const zeroYaw = useZeroYaw()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const targetRef = useRef<Quat>({ ...Q_IDENTITY })

  const a = attitude.data
  const waiting = !a?.initialized
  const stale = !!a?.initialized && (a.last_age_ms == null || a.last_age_ms > 1000)

  useEffect(() => {
    if (a?.initialized) targetRef.current = quatNormalize(a.q)
  }, [a])

  // rAF loop: slerp the displayed quaternion toward the newest polled target
  // so 150 ms samples render as smooth motion instead of 6–7 fps steps.
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Theme is dark-only and static — resolve the tokens once.
    const style = getComputedStyle(canvas)
    const token = (name: string, fallback: string) => style.getPropertyValue(name).trim() || fallback
    const colors: ViewerColors = {
      grid: token('--border', '#27272b'),
      x: token('--destructive', '#f0506e'),
      y: token('--success', '#2dd36f'),
      z: token('--chart-2', '#38bdf8'),
    }

    let raf = 0
    let last = performance.now()
    let display: Quat = { ...targetRef.current }
    const tick = (now: number) => {
      const dt = Math.min((now - last) / 1000, 0.1)
      last = now
      // Exponential approach (~90 ms time constant) toward the latest target.
      display = quatSlerp(display, targetRef.current, 1 - Math.exp(-dt / 0.09))
      draw(canvas, ctx, display, colors)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <div className="space-y-3">
      <div className="relative">
        <canvas
          ref={canvasRef}
          data-testid="imu-attitude-canvas"
          className={cn('h-60 w-full', waiting && 'opacity-30')}
        />
        {waiting && (
          <div className="absolute inset-0 flex items-center justify-center">
            <span className="text-xs text-muted-foreground">
              waiting for IMU (needs ~1 g gravity reference)
            </span>
          </div>
        )}
        {!waiting && stale && (
          <div className="absolute inset-0 flex items-center justify-center rounded-md bg-background/60">
            <span className="text-xs text-warning">IMU stream stale</span>
          </div>
        )}
      </div>

      <div className="grid grid-cols-2 gap-x-6 gap-y-1 sm:grid-cols-3">
        <Readout label="roll" value={fmtDegValue(a?.euler.roll_deg)} />
        <Readout label="pitch" value={fmtDegValue(a?.euler.pitch_deg)} />
        <Readout label="yaw" value={fmtDegValue(a?.euler.yaw_deg)} />
        <Readout label="|accel|" value={fmtMagnitude(a?.accel_mps2, 'm/s²', 2)} />
        <Readout label="|gyro|" value={fmtMagnitude(a?.gyro_radps, 'rad/s', 3)} />
        <Readout label="rate" value={formatHz(a?.rate_hz, 0)} />
      </div>

      <div className="flex items-center gap-3">
        <Button
          size="sm"
          variant="secondary"
          disabled={waiting || zeroYaw.isPending}
          onClick={() => zeroYaw.mutate(undefined, { onSuccess: () => toast.success('Yaw zeroed') })}
        >
          Zero yaw
        </Button>
        <p className="text-xs text-muted-foreground">
          Yaw is gyro-only and drifts — zero it to re-reference.
        </p>
      </div>
    </div>
  )
}
