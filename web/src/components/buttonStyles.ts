import type { CSSProperties } from 'react'

export const buttonStyle = (color: string, fill: string, text: string): CSSProperties => ({
  padding: '6px 12px',
  fontSize: 14,
  borderWidth: 1,
  borderStyle: 'solid',
  borderColor: color,
  borderRadius: 6,
  background: fill,
  color: text,
  cursor: 'pointer',
})

export const neutralButtonStyle = buttonStyle('#ccc', '#fafafa', '#222')
export const primaryButtonStyle = buttonStyle('#2563eb', '#2563eb', '#fff')
export const dangerButtonStyle = buttonStyle('#dc2626', '#fff', '#dc2626')
