import type { LucideIcon } from 'lucide-react'
import { Card, CardContent } from '@/components/ui/card'
import { cn } from '@/lib/utils'

export type StatTone = 'default' | 'good' | 'warn' | 'bad'

const valueTone: Record<StatTone, string> = {
  default: 'text-foreground',
  good: 'text-success',
  warn: 'text-warning',
  bad: 'text-destructive',
}

export function StatCard({
  label,
  value,
  sub,
  icon: Icon,
  tone = 'default',
  className,
  testId,
}: {
  label: string
  value: React.ReactNode
  sub?: React.ReactNode
  icon?: LucideIcon
  tone?: StatTone
  className?: string
  testId?: string
}) {
  return (
    <Card className={cn('py-4', className)} data-testid={testId}>
      <CardContent className="px-4">
        <div className="flex items-center justify-between gap-2">
          <span className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
            {label}
          </span>
          {Icon && <Icon className="size-4 text-muted-foreground" />}
        </div>
        <div
          className={cn('mt-1 font-mono text-2xl font-semibold tabular-nums', valueTone[tone])}
          data-testid={testId ? `${testId}-value` : undefined}
        >
          {value}
        </div>
        {sub != null && <div className="mt-1 text-xs text-muted-foreground">{sub}</div>}
      </CardContent>
    </Card>
  )
}
