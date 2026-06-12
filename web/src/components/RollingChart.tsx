import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import type { TimePoint } from '@/hooks/use-time-series'

export interface SeriesDef {
  key: string
  label: string
  color?: string
}

const PALETTE = [
  'var(--chart-1)',
  'var(--chart-2)',
  'var(--chart-3)',
  'var(--chart-4)',
  'var(--chart-5)',
]

function clock(t: number): string {
  const d = new Date(t)
  return `${d.getMinutes().toString().padStart(2, '0')}:${d.getSeconds().toString().padStart(2, '0')}`
}

// Rolling time-series line chart over a useTimeSeries buffer. Animation is
// off on purpose — points arrive every poll tick and animating each append
// makes the trace swim.
export function RollingChart({
  points,
  series,
  height = 160,
  unit,
  yDomain,
}: {
  points: TimePoint[]
  series: SeriesDef[]
  height?: number
  unit?: string
  yDomain?: [number | 'auto', number | 'auto']
}) {
  return (
    <ResponsiveContainer width="100%" height={height}>
      <LineChart data={points} margin={{ top: 4, right: 8, bottom: 0, left: -12 }}>
        <CartesianGrid stroke="var(--border)" strokeDasharray="3 3" vertical={false} />
        <XAxis
          dataKey="t"
          type="number"
          domain={['dataMin', 'dataMax']}
          tickFormatter={clock}
          tick={{ fill: 'var(--muted-foreground)', fontSize: 10 }}
          tickLine={false}
          axisLine={{ stroke: 'var(--border)' }}
          minTickGap={40}
        />
        <YAxis
          domain={yDomain ?? [0, 'auto']}
          tick={{ fill: 'var(--muted-foreground)', fontSize: 10 }}
          tickLine={false}
          axisLine={false}
          width={48}
          unit={unit}
        />
        <Tooltip
          isAnimationActive={false}
          labelFormatter={(t) => clock(Number(t))}
          formatter={(value, name) => [
            `${typeof value === 'number' ? value.toFixed(2) : String(value ?? '—')}${unit ?? ''}`,
            String(name),
          ]}
          contentStyle={{
            background: 'var(--popover)',
            border: '1px solid var(--border)',
            borderRadius: 'var(--radius)',
            fontSize: 12,
          }}
          labelStyle={{ color: 'var(--muted-foreground)' }}
        />
        {series.map((s, i) => (
          <Line
            key={s.key}
            type="monotone"
            dataKey={s.key}
            name={s.label}
            stroke={s.color ?? PALETTE[i % PALETTE.length]}
            strokeWidth={1.5}
            dot={false}
            isAnimationActive={false}
            connectNulls
          />
        ))}
      </LineChart>
    </ResponsiveContainer>
  )
}
