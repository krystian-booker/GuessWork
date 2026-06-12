import { getJson, send, sendJson } from './http'

export interface FieldLayoutSummary {
  id: number
  name: string
  active: boolean
  created_at: number
  tag_count: number | null
  field_length_m: number | null
  field_width_m: number | null
}

export interface FieldLayoutDetail extends FieldLayoutSummary {
  // Verbatim WPILib AprilTagFieldLayout JSON text.
  json: string
}

// WPILib AprilTagFieldLayout shape (subset the UI reads — see
// src/apriltag/field_layout.hpp for the normative description).
export interface WpiFieldLayout {
  tags: Array<{
    ID: number
    pose: {
      translation: { x: number; y: number; z: number }
      rotation: { quaternion: { W: number; X: number; Y: number; Z: number } }
    }
  }>
  field: { length: number; width: number }
}

export async function listFieldLayouts(): Promise<FieldLayoutSummary[]> {
  return getJson<FieldLayoutSummary[]>('/api/field-layouts')
}

export async function getFieldLayout(id: number): Promise<FieldLayoutDetail> {
  return getJson<FieldLayoutDetail>(`/api/field-layouts/${id}`)
}

export async function createFieldLayout(input: {
  name: string
  layout: unknown
}): Promise<FieldLayoutSummary> {
  return sendJson<FieldLayoutSummary>('/api/field-layouts', 'POST', input)
}

export async function activateFieldLayout(id: number): Promise<FieldLayoutSummary> {
  return sendJson<FieldLayoutSummary>(`/api/field-layouts/${id}/activate`, 'POST')
}

export async function deleteFieldLayout(id: number): Promise<void> {
  return send(`/api/field-layouts/${id}`, 'DELETE')
}
