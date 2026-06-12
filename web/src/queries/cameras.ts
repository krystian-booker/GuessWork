import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  CameraOfflineError,
  createCamera,
  deleteCamera,
  getAvailableCameraModes,
  getCamera,
  getCameraModes,
  getCameraSettingsLimits,
  listAvailableCameras,
  listCameras,
  updateCamera,
} from '@/api/cameras'
import { POLL_SLOW } from '@/lib/query-client'
import { qk } from './keys'

export function useCameras() {
  return useQuery({
    queryKey: qk.cameras,
    queryFn: listCameras,
    refetchInterval: POLL_SLOW,
  })
}

export function useCamera(id: number) {
  return useQuery({
    queryKey: qk.camera(id),
    queryFn: () => getCamera(id),
  })
}

// Poll while the add dialog is open so plugging a camera in shows up live.
export function useAvailableCameras(enabled: boolean) {
  return useQuery({
    queryKey: qk.camerasAvailable,
    queryFn: listAvailableCameras,
    enabled,
    refetchInterval: enabled ? 3000 : false,
  })
}

export function useCameraModes(id: number, online: boolean) {
  return useQuery({
    queryKey: qk.cameraModes(id),
    queryFn: () => getCameraModes(id),
    enabled: online,
    retry: (count, err) => !(err instanceof CameraOfflineError) && count < 1,
  })
}

export function useAvailableCameraModes(serial: string | null) {
  return useQuery({
    queryKey: serial ? qk.availableModes(serial) : ['cameras', 'available', 'none', 'modes'],
    queryFn: () => getAvailableCameraModes(serial!),
    enabled: serial != null,
  })
}

export function useCameraLimits(id: number, online: boolean) {
  return useQuery({
    queryKey: qk.cameraLimits(id),
    queryFn: () => getCameraSettingsLimits(id),
    enabled: online,
    retry: (count, err) => !(err instanceof CameraOfflineError) && count < 1,
  })
}

export function useCreateCamera() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: createCamera,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.cameras })
      void qc.invalidateQueries({ queryKey: qk.camerasAvailable })
    },
  })
}

export function useUpdateCamera(id: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (patch: Parameters<typeof updateCamera>[1]) => updateCamera(id, patch),
    onSuccess: (camera) => {
      qc.setQueryData(qk.camera(id), camera)
      void qc.invalidateQueries({ queryKey: qk.cameras, exact: true })
    },
  })
}

export function useDeleteCamera() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: deleteCamera,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.cameras })
      void qc.invalidateQueries({ queryKey: qk.camerasAvailable })
    },
  })
}
