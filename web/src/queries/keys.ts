// Central query-key factory. Status keys are shared between the header
// health cluster and pages so a mounted page adds zero duplicate requests.
export const qk = {
  status: ['status'] as const,

  cameras: ['cameras'] as const,
  camerasAvailable: ['cameras', 'available'] as const,
  availableModes: (serial: string) => ['cameras', 'available', serial, 'modes'] as const,
  camera: (id: number) => ['cameras', id] as const,
  cameraModes: (id: number) => ['cameras', id, 'modes'] as const,
  cameraLimits: (id: number) => ['cameras', id, 'limits'] as const,
  cameraCalibration: (id: number) => ['cameras', id, 'calibration'] as const,
  cameraRecording: (id: number) => ['cameras', id, 'calibration', 'recording'] as const,
  cameraJob: (id: number) => ['cameras', id, 'calibration', 'job'] as const,
  cameraExtrinsics: (id: number) => ['cameras', id, 'extrinsics'] as const,

  extrinsicsRecording: ['extrinsics', 'recording'] as const,
  extrinsicsJob: ['extrinsics', 'job'] as const,

  fieldLayouts: ['field-layouts'] as const,
  fieldLayout: (id: number) => ['field-layouts', id] as const,

  apriltagStatus: ['apriltag', 'status'] as const,
  vioStatus: ['vio', 'status'] as const,
  vioConfig: ['vio', 'config'] as const,
  canStatus: ['can', 'status'] as const,
  canConfig: ['can', 'config'] as const,
  fusionStatus: ['fusion', 'status'] as const,
  fusionConfig: ['fusion', 'config'] as const,
  imuStatus: ['imu', 'status'] as const,
  imuConfig: ['imu', 'config'] as const,
  imuAllan: ['imu', 'allan'] as const,

  hwSyncStatus: ['hardware-sync', 'status'] as const,
  hwSyncGroups: ['hardware-sync', 'groups'] as const,
}
