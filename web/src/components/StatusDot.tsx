import { cn } from '@/lib/utils'

export type DotTone = 'good' | 'warn' | 'bad' | 'idle'

const toneClass: Record<DotTone, string> = {
  good: 'bg-success',
  warn: 'bg-warning',
  bad: 'bg-destructive',
  idle: 'bg-muted-foreground/50',
}

export function StatusDot({
  tone,
  pulse = false,
  className,
}: {
  tone: DotTone
  pulse?: boolean
  className?: string
}) {
  return (
    <span className={cn('relative inline-flex size-2 shrink-0', className)}>
      {pulse && (
        <span
          className={cn(
            'absolute inline-flex h-full w-full animate-ping rounded-full opacity-60',
            toneClass[tone],
          )}
        />
      )}
      <span className={cn('relative inline-flex size-2 rounded-full', toneClass[tone])} />
    </span>
  )
}
