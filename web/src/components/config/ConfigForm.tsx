import { useEffect, useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { Separator } from '@/components/ui/separator'
import { Switch } from '@/components/ui/switch'
import { cn } from '@/lib/utils'

export interface NumberFieldDef {
  kind: 'number'
  key: string
  label: string
  unit?: string
  help?: string
  int?: boolean
  min?: number
}

export interface SwitchFieldDef {
  kind: 'switch'
  key: string
  label: string
  help?: string
}

// Free-form string field (e.g. an IP address). Any value — including the
// empty string — is valid; the server does the semantic validation.
export interface TextFieldDef {
  kind: 'text'
  key: string
  label: string
  help?: string
  placeholder?: string
}

export type FieldDef = NumberFieldDef | SwitchFieldDef | TextFieldDef

export interface FieldGroup {
  title?: string
  fields: FieldDef[]
}

type ConfigValue = Record<string, number | boolean | string>

// Explicit-save tuning form: edits accumulate in a draft, Save sends only the
// changed keys, Reset discards. The draft resets when `version` changes
// (pass the config's updated_at) so a successful PUT snaps to server truth
// without clobbering in-progress edits on unrelated refetches.
export function ConfigForm({
  value,
  version,
  groups,
  onSave,
  saving,
}: {
  value: ConfigValue | undefined
  version: number | string | undefined
  groups: FieldGroup[]
  onSave: (patch: ConfigValue) => void
  saving?: boolean
}) {
  const [draft, setDraft] = useState<Record<string, string | boolean>>({})
  useEffect(() => setDraft({}), [version])

  const fields = useMemo(() => groups.flatMap((g) => g.fields), [groups])

  const { patch, invalid } = useMemo(() => {
    const patch: ConfigValue = {}
    const invalid = new Set<string>()
    if (!value) return { patch, invalid }
    for (const f of fields) {
      const d = draft[f.key]
      if (d === undefined) continue
      if (f.kind === 'switch') {
        if (d !== value[f.key]) patch[f.key] = d as boolean
        continue
      }
      if (f.kind === 'text') {
        if (typeof d === 'string' && d !== value[f.key]) patch[f.key] = d
        continue
      }
      const n = Number(d)
      if (typeof d !== 'string' || d.trim() === '' || !Number.isFinite(n)) {
        invalid.add(f.key)
        continue
      }
      if (f.int && !Number.isInteger(n)) {
        invalid.add(f.key)
        continue
      }
      if (f.min != null && n < f.min) {
        invalid.add(f.key)
        continue
      }
      if (n !== value[f.key]) patch[f.key] = n
    }
    return { patch, invalid }
  }, [draft, fields, value])

  const dirty = Object.keys(patch).length > 0 || invalid.size > 0

  if (!value) return <p className="text-sm text-muted-foreground">Loading…</p>

  return (
    <div className="space-y-4">
      {groups.map((g, gi) => (
        <div key={g.title ?? gi} className="space-y-3">
          {g.title && (
            <>
              {gi > 0 && <Separator />}
              <p className="text-xs font-medium tracking-wide text-muted-foreground uppercase">
                {g.title}
              </p>
            </>
          )}
          {g.fields.map((f) =>
            f.kind === 'switch' ? (
              <div key={f.key} className="flex items-center justify-between gap-4">
                <div>
                  <Label className="text-sm">{f.label}</Label>
                  {f.help && <p className="text-xs text-muted-foreground">{f.help}</p>}
                </div>
                <Switch
                  checked={(draft[f.key] ?? value[f.key]) === true}
                  onCheckedChange={(v) => setDraft((d) => ({ ...d, [f.key]: v }))}
                  aria-label={f.label}
                />
              </div>
            ) : (
              <div key={f.key} className="flex items-center justify-between gap-4">
                <div className="min-w-0">
                  <Label className="text-sm" htmlFor={`cf-${f.key}`}>
                    {f.label}
                  </Label>
                  {f.help && <p className="text-xs text-muted-foreground">{f.help}</p>}
                </div>
                <div className="flex shrink-0 items-center gap-1.5">
                  <Input
                    id={`cf-${f.key}`}
                    className={cn(
                      'w-32 font-mono text-xs tabular-nums',
                      f.kind === 'number' && 'text-right',
                      invalid.has(f.key) && 'border-destructive',
                    )}
                    placeholder={f.kind === 'text' ? f.placeholder : undefined}
                    value={(draft[f.key] as string | undefined) ?? String(value[f.key])}
                    onChange={(e) => setDraft((d) => ({ ...d, [f.key]: e.target.value }))}
                  />
                  {f.kind === 'number' && f.unit && (
                    <span className="w-10 text-xs text-muted-foreground">{f.unit}</span>
                  )}
                </div>
              </div>
            ),
          )}
        </div>
      ))}

      <div className="flex items-center gap-2 pt-1">
        <Button
          size="sm"
          disabled={!dirty || invalid.size > 0 || saving}
          onClick={() => onSave(patch)}
          data-testid="config-save"
        >
          {saving ? 'Saving…' : 'Save'}
        </Button>
        {dirty && (
          <Button size="sm" variant="ghost" onClick={() => setDraft({})}>
            Reset
          </Button>
        )}
        {invalid.size > 0 && <span className="text-xs text-destructive">invalid values</span>}
      </div>
    </div>
  )
}
