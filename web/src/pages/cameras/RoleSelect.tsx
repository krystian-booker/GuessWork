import { CAMERA_ROLES, type CameraRole } from '@/api/cameras'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'

const ROLE_LABELS: Record<CameraRole, string> = {
  apriltag: 'AprilTag detection',
  vio_left: 'VIO left',
  vio_right: 'VIO right',
}

export function roleLabel(role: CameraRole | null): string {
  return role ? ROLE_LABELS[role] : 'None (stream only)'
}

const NONE = '__none__'

// vio_left / vio_right are unique system-wide — the server replies 409 on
// conflict and the global mutation error toast surfaces it.
export function RoleSelect({
  value,
  disabled,
  onChange,
}: {
  value: CameraRole | null
  disabled?: boolean
  onChange: (role: CameraRole | null) => void
}) {
  return (
    <Select
      value={value ?? NONE}
      disabled={disabled}
      onValueChange={(v) => onChange(v === NONE ? null : (v as CameraRole))}
    >
      <SelectTrigger className="w-full" aria-label="Camera role">
        <SelectValue />
      </SelectTrigger>
      <SelectContent>
        <SelectItem value={NONE}>{roleLabel(null)}</SelectItem>
        {CAMERA_ROLES.map((r) => (
          <SelectItem key={r} value={r}>
            {ROLE_LABELS[r]}
          </SelectItem>
        ))}
      </SelectContent>
    </Select>
  )
}
