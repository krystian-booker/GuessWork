import { Route, Routes } from 'react-router-dom'
import Nav from './Nav'
import CamerasPage from './pages/CamerasPage'
import StreamPage from './pages/StreamPage'

export default function App() {
  return (
    <div style={{ fontFamily: 'system-ui, sans-serif', padding: 24, maxWidth: 720 }}>
      <h1 style={{ marginBottom: 8 }}>GuessWork</h1>
      <Nav />
      <Routes>
        <Route path="/" element={<StreamPage />} />
        <Route path="/cameras" element={<CamerasPage />} />
      </Routes>
    </div>
  )
}
