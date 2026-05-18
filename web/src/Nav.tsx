import { NavLink } from 'react-router-dom'

const linkStyle = (isActive: boolean): React.CSSProperties => ({
  padding: '6px 12px',
  borderRadius: 6,
  textDecoration: 'none',
  color: isActive ? '#fff' : '#333',
  background: isActive ? '#2563eb' : 'transparent',
  fontWeight: isActive ? 600 : 400,
})

export default function Nav() {
  return (
    <nav style={{ display: 'flex', gap: 8, marginBottom: 24 }}>
      <NavLink to="/" end style={({ isActive }) => linkStyle(isActive)}>
        Stream
      </NavLink>
      <NavLink to="/cameras" style={({ isActive }) => linkStyle(isActive)}>
        Cameras
      </NavLink>
      <NavLink to="/hardware-sync" style={({ isActive }) => linkStyle(isActive)}>
        Hardware Sync
      </NavLink>
    </nav>
  )
}
