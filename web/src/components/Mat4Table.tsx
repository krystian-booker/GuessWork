import type { Mat4 } from '@/lib/matrices'
import { cn } from '@/lib/utils'

// Compact monospace 4x4 homogeneous-transform display.
export function Mat4Table({
  matrix,
  digits = 4,
  className,
}: {
  matrix: Mat4
  digits?: number
  className?: string
}) {
  return (
    <table className={cn('font-mono text-xs tabular-nums', className)}>
      <tbody>
        {matrix.map((row, i) => (
          <tr key={i}>
            {row.map((v, j) => (
              <td key={j} className="px-2 py-0.5 text-right text-muted-foreground first:pl-0">
                {v.toFixed(digits)}
              </td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  )
}
