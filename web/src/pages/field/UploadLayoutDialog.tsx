import { useRef, useState } from 'react'
import { toast } from 'sonner'
import { Upload } from 'lucide-react'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { Textarea } from '@/components/ui/textarea'
import { validateLayoutShape } from '@/components/field/fieldGeometry'
import { useCreateFieldLayout } from '@/queries/fieldLayouts'

// Upload a WPILib AprilTagFieldLayout JSON (file pick or paste). Client-side
// shape validation gives fast feedback; the server parser is authoritative.
export function UploadLayoutDialog() {
  const [open, setOpen] = useState(false)
  const [name, setName] = useState('')
  const [text, setText] = useState('')
  const fileRef = useRef<HTMLInputElement>(null)
  const create = useCreateFieldLayout()

  let parsed: unknown = null
  let parseError: string | null = null
  if (text.trim()) {
    try {
      parsed = JSON.parse(text)
      parseError = validateLayoutShape(parsed)
    } catch {
      parseError = 'invalid JSON'
    }
  }

  const tagCount =
    parsed && !parseError ? (parsed as { tags: unknown[] }).tags.length : null

  const onFile = (file: File) => {
    void file.text().then((t) => {
      setText(t)
      if (!name) setName(file.name.replace(/\.json$/i, ''))
    })
  }

  const submit = () => {
    if (!parsed || parseError || !name.trim()) return
    create.mutate(
      { name: name.trim(), layout: parsed },
      {
        onSuccess: (layout) => {
          toast.success(`Layout "${layout.name}" uploaded`)
          setOpen(false)
          setName('')
          setText('')
        },
      },
    )
  }

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button size="sm" variant="secondary" data-testid="upload-layout">
          <Upload /> Upload layout
        </Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-lg">
        <DialogHeader>
          <DialogTitle>Upload field layout</DialogTitle>
          <DialogDescription>
            WPILib AprilTagFieldLayout JSON (the season file from allwpilib, or a custom layout).
          </DialogDescription>
        </DialogHeader>
        <div className="space-y-4">
          <div className="space-y-1.5">
            <Label htmlFor="layout-name">Name</Label>
            <Input
              id="layout-name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="e.g. 2026-rebuilt"
            />
          </div>
          <div className="space-y-1.5">
            <Label>Layout JSON</Label>
            <Input
              ref={fileRef}
              type="file"
              accept=".json,application/json"
              onChange={(e) => {
                const f = e.target.files?.[0]
                if (f) onFile(f)
              }}
            />
            <Textarea
              value={text}
              onChange={(e) => setText(e.target.value)}
              placeholder='…or paste: {"tags": […], "field": {"length": …, "width": …}}'
              className="min-h-28 font-mono text-xs"
            />
            {text.trim() &&
              (parseError ? (
                <p className="text-xs text-destructive">{parseError}</p>
              ) : (
                <p className="text-xs text-success">{tagCount} tags parsed ✓</p>
              ))}
          </div>
        </div>
        <DialogFooter>
          <Button variant="ghost" onClick={() => setOpen(false)}>
            Cancel
          </Button>
          <Button
            onClick={submit}
            disabled={!name.trim() || !text.trim() || !!parseError || create.isPending}
          >
            Upload
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
