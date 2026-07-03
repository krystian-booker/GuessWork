import { NavLink } from 'react-router-dom'
import {
  Camera,
  Crosshair,
  GitMerge,
  LayoutDashboard,
  Map,
  Network,
  QrCode,
  Route,
  Settings,
  Zap,
  type LucideIcon,
} from 'lucide-react'
import { cn } from '@/lib/utils'

export interface NavItem {
  to: string
  label: string
  icon: LucideIcon
}

export interface NavGroup {
  label: string
  items: NavItem[]
}

export const NAV_GROUPS: NavGroup[] = [
  {
    label: 'Overview',
    items: [
      { to: '/', label: 'Dashboard', icon: LayoutDashboard },
      { to: '/field', label: 'Field', icon: Map },
    ],
  },
  {
    label: 'Vision',
    items: [
      { to: '/cameras', label: 'Cameras', icon: Camera },
      { to: '/calibration', label: 'Calibration', icon: Crosshair },
    ],
  },
  {
    label: 'Localization',
    items: [
      { to: '/apriltag', label: 'AprilTag', icon: QrCode },
      { to: '/vio', label: 'VIO', icon: Route },
      { to: '/fusion', label: 'Fusion', icon: GitMerge },
    ],
  },
  {
    label: 'Hardware',
    items: [
      { to: '/robot', label: 'Robot', icon: Network },
      { to: '/hardware-sync', label: 'Hardware Sync', icon: Zap },
    ],
  },
  {
    label: 'System',
    items: [{ to: '/settings', label: 'Settings', icon: Settings }],
  },
]

// Page title for the header, matched longest-prefix-first so /cameras/3 maps
// to "Cameras" and /calibration/intrinsics/3 to "Calibration".
export function pageTitleFor(pathname: string): string {
  const all = NAV_GROUPS.flatMap((g) => g.items)
  const match = all
    .filter((i) => (i.to === '/' ? pathname === '/' : pathname.startsWith(i.to)))
    .sort((a, b) => b.to.length - a.to.length)[0]
  return match?.label ?? 'GuessWork'
}

export function Sidebar() {
  return (
    <aside className="flex w-14 shrink-0 flex-col border-r bg-sidebar lg:w-56">
      <div className="flex h-14 items-center gap-2 border-b px-3 lg:px-4">
        <div className="flex size-8 shrink-0 items-center justify-center rounded-md bg-primary font-mono text-sm font-bold text-primary-foreground">
          G
        </div>
        <span className="hidden text-sm font-semibold tracking-wide lg:inline">GuessWork</span>
      </div>
      <nav className="flex-1 overflow-y-auto px-2 py-3">
        {NAV_GROUPS.map((group) => (
          <div key={group.label} className="mb-4">
            <div className="mb-1 hidden px-2 text-[10px] font-semibold tracking-widest text-muted-foreground/70 uppercase lg:block">
              {group.label}
            </div>
            {group.items.map((item) => (
              <NavLink
                key={item.to}
                to={item.to}
                end={item.to === '/'}
                title={item.label}
                className={({ isActive }) =>
                  cn(
                    'mb-0.5 flex items-center justify-center gap-2.5 rounded-md px-2 py-2 text-sm text-sidebar-foreground/80 transition-colors lg:justify-start',
                    'hover:bg-sidebar-accent hover:text-sidebar-foreground',
                    isActive &&
                      'bg-sidebar-accent font-medium text-primary hover:text-primary',
                  )
                }
              >
                <item.icon className="size-4 shrink-0" />
                <span className="hidden lg:inline">{item.label}</span>
              </NavLink>
            ))}
          </div>
        ))}
      </nav>
    </aside>
  )
}
