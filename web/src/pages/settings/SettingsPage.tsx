import { useState } from 'react'
import { Link } from 'react-router-dom'
import { toast } from 'sonner'
import { Download, Upload } from 'lucide-react'
import { useQueryClient } from '@tanstack/react-query'
import { exportConfig, importConfig, type ImportResult } from '@/api/config'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { ConfirmButton } from '@/components/ConfirmButton'
import { PageHeader } from '@/components/PageHeader'
import { formatUptime } from '@/lib/format'
import { useStatus } from '@/queries/status'

export default function SettingsPage() {
  const qc = useQueryClient()
  const status = useStatus()

  const [fileName, setFileName] = useState<string | null>(null)
  const [snapshot, setSnapshot] = useState<unknown>(null)
  const [snapshotError, setSnapshotError] = useState<string | null>(null)
  const [importing, setImporting] = useState(false)
  const [result, setResult] = useState<ImportResult | null>(null)

  const download = async () => {
    try {
      const { blob, filename } = await exportConfig()
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = filename
      a.click()
      URL.revokeObjectURL(url)
    } catch (e) {
      toast.error(e instanceof Error ? e.message : String(e))
    }
  }

  const onFile = (file: File) => {
    setFileName(file.name)
    setResult(null)
    void file.text().then((text) => {
      try {
        setSnapshot(JSON.parse(text))
        setSnapshotError(null)
      } catch {
        setSnapshot(null)
        setSnapshotError('Not valid JSON')
      }
    })
  }

  // Summarize top-level sections of the snapshot for the pre-import preview.
  const sections =
    snapshot && typeof snapshot === 'object'
      ? Object.entries(snapshot as Record<string, unknown>).map(([k, v]) => ({
          name: k,
          count: Array.isArray(v) ? v.length : null,
        }))
      : []

  const runImport = async () => {
    setImporting(true)
    try {
      const r = await importConfig(snapshot)
      setResult(r)
      const errors = Object.values(r.report).flatMap((s) => s.errors)
      if (errors.length === 0) toast.success('Configuration imported')
      else toast.warning(`Imported with ${errors.length} error(s) — see report`)
      // The merge touches everything: cameras, layouts, configs, supervisors.
      void qc.invalidateQueries()
    } catch (e) {
      toast.error(e instanceof Error ? e.message : String(e))
    } finally {
      setImporting(false)
    }
  }

  return (
    <div>
      <PageHeader
        title="Settings"
        description="Robot identity snapshot — export and restore the full configuration."
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-2">
        <div className="space-y-4">
          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Export configuration</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <p className="text-xs text-muted-foreground">
                Everything that defines this robot: cameras (including calibration blobs), field
                layouts, trigger groups, and the IMU / VIO / robot-link / fusion configs.
              </p>
              <Button onClick={download} data-testid="export-config">
                <Download /> Download snapshot
              </Button>
            </CardContent>
          </Card>

          <Card className="py-4 gap-3">
            <CardHeader className="px-4">
              <CardTitle className="text-sm">Import configuration</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 px-4">
              <p className="text-xs text-muted-foreground">
                Non-destructive merge: cameras match by serial, layouts and trigger groups by
                name. Trigger groups are stored but not armed.
              </p>
              <Input
                type="file"
                accept=".json,application/json"
                onChange={(e) => {
                  const f = e.target.files?.[0]
                  if (f) onFile(f)
                }}
              />
              {snapshotError && <p className="text-xs text-destructive">{snapshotError}</p>}
              {sections.length > 0 && (
                <div className="space-y-1 rounded-md border p-3">
                  <p className="text-xs font-medium">{fileName}</p>
                  <div className="flex flex-wrap gap-1.5">
                    {sections.map((s) => (
                      <Badge key={s.name} variant="secondary" className="font-mono text-[11px]">
                        {s.name}
                        {s.count != null ? ` (${s.count})` : ''}
                      </Badge>
                    ))}
                  </div>
                </div>
              )}
              <ConfirmButton
                variant="default"
                disabled={!snapshot || importing}
                title="Import this snapshot?"
                description="Merges into the current configuration and reloads AprilTag, VIO, fusion and the robot link. Existing items with matching identity are updated in place."
                confirmLabel={importing ? 'Importing…' : 'Import'}
                onConfirm={runImport}
              >
                <Upload /> Import snapshot
              </ConfirmButton>

              {result && (
                <div className="space-y-1.5 rounded-md border p-3" data-testid="import-report">
                  {Object.entries(result.report).map(([section, r]) => (
                    <div key={section} className="text-xs">
                      <span className="font-mono">{section}</span>:{' '}
                      <span className="text-muted-foreground">
                        {r.created} created, {r.updated} updated
                      </span>
                      {r.errors.map((e) => (
                        <p key={e} className="text-destructive">
                          ✗ {e}
                        </p>
                      ))}
                    </div>
                  ))}
                  {result.note && <p className="text-xs text-warning">{result.note}</p>}
                </div>
              )}
            </CardContent>
          </Card>
        </div>

        <Card className="py-4 gap-3 self-start">
          <CardHeader className="px-4">
            <CardTitle className="text-sm">About</CardTitle>
          </CardHeader>
          <CardContent className="space-y-1.5 px-4 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Backend uptime</span>
              <span className="font-mono text-xs tabular-nums">
                {formatUptime(status.data?.uptime_s)}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Hardware sync</span>
              <Link to="/hardware-sync" className="text-xs text-primary hover:underline">
                trigger groups →
              </Link>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Field layouts</span>
              <Link to="/field" className="text-xs text-primary hover:underline">
                manage →
              </Link>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
