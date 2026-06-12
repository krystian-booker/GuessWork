import { Route, Routes } from 'react-router-dom'
import { AppShell } from '@/components/layout/AppShell'
import { EmptyState } from '@/components/EmptyState'
import DashboardPage from './pages/DashboardPage'
import CamerasPage from './pages/cameras/CamerasPage'
import CameraDetailPage from './pages/cameras/CameraDetailPage'
import CalibrationHubPage from './pages/calibration/CalibrationHubPage'
import IntrinsicsWizardPage from './pages/calibration/IntrinsicsWizardPage'
import ExtrinsicsPage from './pages/calibration/ExtrinsicsPage'
import AllanPage from './pages/calibration/AllanPage'
import FieldPage from './pages/field/FieldPage'
import VioPage from './pages/vio/VioPage'
import FusionPage from './pages/fusion/FusionPage'
import CanPage from './pages/can/CanPage'
import SettingsPage from './pages/settings/SettingsPage'
import HardwareSyncPage from './pages/hardware-sync/HardwareSyncPage'

export default function App() {
  return (
    <Routes>
      <Route element={<AppShell />}>
        <Route path="/" element={<DashboardPage />} />
        <Route path="/cameras" element={<CamerasPage />} />
        <Route path="/cameras/:id" element={<CameraDetailPage />} />
        <Route path="/calibration" element={<CalibrationHubPage />} />
        <Route path="/calibration/intrinsics/:id" element={<IntrinsicsWizardPage />} />
        <Route path="/calibration/extrinsics" element={<ExtrinsicsPage />} />
        <Route path="/calibration/allan" element={<AllanPage />} />
        <Route path="/field" element={<FieldPage />} />
        <Route path="/vio" element={<VioPage />} />
        <Route path="/fusion" element={<FusionPage />} />
        <Route path="/can" element={<CanPage />} />
        <Route path="/hardware-sync" element={<HardwareSyncPage />} />
        <Route path="/settings" element={<SettingsPage />} />
        <Route path="*" element={<EmptyState title="Not found" description="No such page." />} />
      </Route>
    </Routes>
  )
}
