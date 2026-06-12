import { cn } from '@/lib/utils'
import type { FieldModel } from './fieldGeometry'

const TAG_SIZE = 0.165 // tag36h11 edge, meters
const ROBOT_LEN = 0.45
const ROBOT_WID = 0.38

export interface FieldPose {
  x: number
  y: number
  thetaRad: number
}

// Top-down SVG field renderer. Geometry is authored directly in field meters:
// viewBox spans the field, a translate+scale flip puts the origin bottom-left
// with +Y up (FRC/NWU convention); text gets a nested un-flip so it reads
// normally.
export function FieldView({
  model,
  pose,
  poseStale = false,
  trail,
  highlightTagIds,
  showIds = true,
  className,
}: {
  model: FieldModel
  pose: FieldPose | null
  // Render the robot dimmed when the pose is stale (>100 ms p95 staleness).
  poseStale?: boolean
  trail?: Array<{ x: number; y: number }>
  // Tags currently observed by any camera — drawn in accent green.
  highlightTagIds?: Set<number>
  showIds?: boolean
  className?: string
}) {
  const { lengthM: L, widthM: W, tags } = model
  const stroke = 0.02

  const gridLines: React.ReactNode[] = []
  for (let x = 1; x < L; x++) {
    gridLines.push(
      <line key={`vx${x}`} x1={x} y1={0} x2={x} y2={W} stroke="var(--border)" strokeWidth={0.008} />,
    )
  }
  for (let y = 1; y < W; y++) {
    gridLines.push(
      <line key={`hy${y}`} x1={0} y1={y} x2={L} y2={y} stroke="var(--border)" strokeWidth={0.008} />,
    )
  }

  return (
    <svg
      viewBox={`${-stroke} ${-stroke} ${L + 2 * stroke} ${W + 2 * stroke}`}
      className={cn('w-full rounded-lg border bg-card', className)}
      data-testid="field-view"
    >
      {/* Flip to field frame: origin bottom-left, +Y up. */}
      <g transform={`translate(0 ${W}) scale(1 -1)`}>
        <rect x={0} y={0} width={L} height={W} fill="var(--background)" stroke="var(--border)" strokeWidth={stroke} />
        {gridLines}
        {/* center line */}
        <line x1={L / 2} y1={0} x2={L / 2} y2={W} stroke="var(--border)" strokeWidth={0.015} strokeDasharray="0.1 0.1" />

        {tags.map((t) => {
          const hot = highlightTagIds?.has(t.id) ?? false
          const color = hot ? 'var(--primary)' : 'var(--muted-foreground)'
          return (
            <g key={t.id} data-tag-id={t.id}>
              <g transform={`translate(${t.x} ${t.y}) rotate(${(t.yawRad * 180) / Math.PI})`}>
                {/* Tag face spans the Y axis in tag frame (normal = +X). */}
                <line
                  x1={0}
                  y1={-TAG_SIZE / 2}
                  x2={0}
                  y2={TAG_SIZE / 2}
                  stroke={color}
                  strokeWidth={0.05}
                />
                {/* facing tick along the outward normal */}
                <line x1={0} y1={0} x2={0.12} y2={0} stroke={color} strokeWidth={0.02} />
              </g>
              {showIds && (
                <g transform={`translate(${t.x} ${t.y}) scale(1 -1)`}>
                  <text
                    x={0}
                    y={-0.16}
                    textAnchor="middle"
                    fontSize={0.22}
                    fill={hot ? 'var(--primary)' : 'var(--muted-foreground)'}
                    fontFamily="var(--font-mono)"
                  >
                    {t.id}
                  </text>
                </g>
              )}
            </g>
          )
        })}

        {trail && trail.length > 1 && (
          <polyline
            points={trail.map((p) => `${p.x},${p.y}`).join(' ')}
            fill="none"
            stroke="var(--chart-2)"
            strokeWidth={0.025}
            strokeOpacity={0.5}
            data-testid="robot-trail"
          />
        )}

        {pose && (
          <g
            transform={`translate(${pose.x} ${pose.y}) rotate(${(pose.thetaRad * 180) / Math.PI})`}
            opacity={poseStale ? 0.4 : 1}
            data-testid="robot-pose"
          >
            {/* heading ray */}
            <line x1={0} y1={0} x2={ROBOT_LEN} y2={0} stroke="var(--primary)" strokeWidth={0.02} strokeOpacity={0.6} />
            {/* isoceles triangle pointing +X */}
            <polygon
              points={`${ROBOT_LEN / 2},0 ${-ROBOT_LEN / 2},${ROBOT_WID / 2} ${-ROBOT_LEN / 2},${-ROBOT_WID / 2}`}
              fill="var(--primary)"
              fillOpacity={0.85}
              stroke="var(--primary)"
              strokeWidth={0.02}
            />
          </g>
        )}
      </g>

      {!pose && (
        <text
          x={L / 2}
          y={W / 2}
          textAnchor="middle"
          fontSize={0.35}
          fill="var(--muted-foreground)"
          data-testid="no-pose"
        >
          no pose
        </text>
      )}
    </svg>
  )
}
