import { toast } from 'sonner'
import { CheckCircle2, Trash2 } from 'lucide-react'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ConfirmButton } from '@/components/ConfirmButton'
import {
  useActivateFieldLayout,
  useDeleteFieldLayout,
  useFieldLayouts,
} from '@/queries/fieldLayouts'
import { UploadLayoutDialog } from './UploadLayoutDialog'

export function LayoutManager() {
  const layouts = useFieldLayouts()
  const activate = useActivateFieldLayout()
  const del = useDeleteFieldLayout()

  return (
    <Card className="py-4 gap-3">
      <CardHeader className="flex flex-row items-center justify-between px-4">
        <CardTitle className="text-sm">Field layouts</CardTitle>
        <UploadLayoutDialog />
      </CardHeader>
      <CardContent className="space-y-2 px-4">
        {(layouts.data ?? []).map((l) => (
          <div
            key={l.id}
            className="flex items-center justify-between gap-2 rounded-md border px-3 py-2"
            data-testid={`layout-${l.id}`}
          >
            <div className="min-w-0">
              <p className="flex items-center gap-2 truncate text-sm font-medium">
                {l.name}
                {l.active && (
                  <Badge className="text-[11px]" data-testid="active-layout-badge">
                    active
                  </Badge>
                )}
              </p>
              <p className="text-xs text-muted-foreground">
                {l.tag_count ?? '?'} tags
                {l.field_length_m != null && l.field_width_m != null
                  ? ` · ${l.field_length_m.toFixed(1)} × ${l.field_width_m.toFixed(1)} m`
                  : ''}
              </p>
            </div>
            <span className="flex shrink-0 gap-1">
              {!l.active && (
                <Button
                  size="sm"
                  variant="ghost"
                  onClick={() =>
                    activate.mutate(l.id, {
                      onSuccess: () => toast.success(`"${l.name}" activated`),
                    })
                  }
                >
                  <CheckCircle2 /> Activate
                </Button>
              )}
              <ConfirmButton
                size="icon"
                variant="ghost"
                aria-label={`Delete ${l.name}`}
                title={`Delete layout "${l.name}"?`}
                description={
                  l.active
                    ? 'This layout is active — the server will refuse to delete it. Activate another layout first.'
                    : undefined
                }
                confirmLabel="Delete"
                onConfirm={() => del.mutate(l.id)}
              >
                <Trash2 className="text-destructive" />
              </ConfirmButton>
            </span>
          </div>
        ))}
        {layouts.data?.length === 0 && (
          <p className="text-sm text-muted-foreground">No layouts stored.</p>
        )}
      </CardContent>
    </Card>
  )
}
