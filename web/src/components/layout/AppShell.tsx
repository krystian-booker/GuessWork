import { Outlet, useLocation } from 'react-router-dom'
import { HeaderHealth } from './HeaderHealth'
import { Sidebar, pageTitleFor } from './Sidebar'

export function AppShell() {
  const { pathname } = useLocation()
  return (
    <div className="flex h-screen overflow-hidden">
      <Sidebar />
      <div className="flex min-w-0 flex-1 flex-col">
        <header className="flex h-14 shrink-0 items-center justify-between gap-4 border-b bg-card/40 px-4 lg:px-6">
          <span className="text-sm font-medium text-muted-foreground">{pageTitleFor(pathname)}</span>
          <HeaderHealth />
        </header>
        <main className="flex-1 overflow-y-auto p-4 lg:p-6">
          <Outlet />
        </main>
      </div>
    </div>
  )
}
