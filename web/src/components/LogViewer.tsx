import { useEffect, useRef } from 'react'
import { cn } from '@/lib/utils'

// Autoscrolling log pane for SSE job logs. Sticks to the bottom unless the
// user has scrolled up (within a 100px slack band, matching the old UI).
export function LogViewer({
  text,
  className,
  maxHeight = 320,
}: {
  text: string
  className?: string
  maxHeight?: number
}) {
  const ref = useRef<HTMLDivElement>(null)
  const stickToBottom = useRef(true)

  useEffect(() => {
    const el = ref.current
    if (el && stickToBottom.current) el.scrollTop = el.scrollHeight
  }, [text])

  return (
    <div
      ref={ref}
      onScroll={(e) => {
        const el = e.currentTarget
        stickToBottom.current = el.scrollHeight - el.scrollTop - el.clientHeight < 100
      }}
      className={cn('overflow-auto rounded-md border bg-background/60 p-3', className)}
      style={{ maxHeight }}
      data-testid="log-viewer"
    >
      <pre className="font-mono text-xs leading-relaxed whitespace-pre-wrap text-muted-foreground">
        {text || '— no output yet —'}
      </pre>
    </div>
  )
}
