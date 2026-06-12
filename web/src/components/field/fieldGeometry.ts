import type { WpiFieldLayout } from '@/api/fieldLayouts'
import { yawFromQuaternion } from '@/lib/matrices'

// Render model for the top-down field view, in field meters.
// Frame (see src/apriltag/field_layout.hpp): FRC field frame — NWU, origin at
// the blue-alliance right corner, +X toward red, +Y left, +Z up. Tag zero
// rotation faces +X; the tag's outward normal is tag-frame +X.
export interface FieldTag {
  id: number
  x: number
  y: number
  yawRad: number
}

export interface FieldModel {
  lengthM: number
  widthM: number
  tags: FieldTag[]
}

export function parseLayout(json: unknown): FieldModel | null {
  const doc = json as Partial<WpiFieldLayout> | null
  if (!doc || typeof doc !== 'object') return null
  const field = doc.field
  if (!field || typeof field.length !== 'number' || typeof field.width !== 'number') return null
  const tags: FieldTag[] = []
  for (const t of doc.tags ?? []) {
    const tr = t?.pose?.translation
    const q = t?.pose?.rotation?.quaternion
    if (
      typeof t?.ID !== 'number' ||
      typeof tr?.x !== 'number' ||
      typeof tr?.y !== 'number' ||
      typeof q?.W !== 'number'
    ) {
      continue
    }
    tags.push({
      id: t.ID,
      x: tr.x,
      y: tr.y,
      yawRad: yawFromQuaternion(q.W, q.X, q.Y, q.Z),
    })
  }
  return { lengthM: field.length, widthM: field.width, tags }
}

export function parseLayoutText(text: string): FieldModel | null {
  try {
    return parseLayout(JSON.parse(text))
  } catch {
    return null
  }
}

// Fast client-side shape check for the upload dialog (the server's parser is
// authoritative).
export function validateLayoutShape(json: unknown): string | null {
  const doc = json as Partial<WpiFieldLayout> | null
  if (!doc || typeof doc !== 'object') return 'not a JSON object'
  if (!Array.isArray(doc.tags) || doc.tags.length === 0) return 'missing non-empty "tags" array'
  if (typeof doc.field?.length !== 'number' || typeof doc.field?.width !== 'number')
    return 'missing "field": {"length", "width"}'
  for (const t of doc.tags) {
    if (typeof t?.ID !== 'number') return 'a tag is missing its "ID"'
    if (typeof t?.pose?.translation?.x !== 'number') return `tag ${t?.ID}: missing pose.translation`
    if (typeof t?.pose?.rotation?.quaternion?.W !== 'number')
      return `tag ${t.ID}: missing pose.rotation.quaternion`
  }
  return null
}
