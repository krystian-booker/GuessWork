import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  cancelJob,
  deleteCalibration,
  getCalibration,
  getJob,
  getRecording,
  startRecording,
  stopRecording,
  uploadCalibration,
} from '@/api/calibration'
import { POLL_MEDIUM, POLL_RECORDING } from '@/lib/query-client'
import { qk } from './keys'

export function useCalibration(cameraId: number) {
  return useQuery({
    queryKey: qk.cameraCalibration(cameraId),
    queryFn: () => getCalibration(cameraId),
  })
}

export function useCalibrationRecording(cameraId: number, opts?: { poll?: boolean }) {
  return useQuery({
    queryKey: qk.cameraRecording(cameraId),
    queryFn: () => getRecording(cameraId),
    refetchInterval: (opts?.poll ?? true) ? POLL_RECORDING : false,
  })
}

// Job state mostly arrives via the SSE done event; polling is a safety net
// (and provides elapsed-time updates).
export function useCalibrationJob(cameraId: number, opts?: { poll?: boolean }) {
  return useQuery({
    queryKey: qk.cameraJob(cameraId),
    queryFn: () => getJob(cameraId),
    refetchInterval: (opts?.poll ?? true) ? POLL_MEDIUM : false,
  })
}

export function useStartRecording(cameraId: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: () => startRecording(cameraId),
    onSuccess: (status) => qc.setQueryData(qk.cameraRecording(cameraId), status),
  })
}

export function useStopRecording(cameraId: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: () => stopRecording(cameraId),
    onSuccess: (resp) => {
      qc.setQueryData(qk.cameraRecording(cameraId), null)
      qc.setQueryData(qk.cameraJob(cameraId), resp.job)
    },
  })
}

export function useCancelJob(cameraId: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: () => cancelJob(cameraId),
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.cameraJob(cameraId) }),
  })
}

export function useUploadCalibration(cameraId: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (yaml: string) => uploadCalibration(cameraId, yaml),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.cameraCalibration(cameraId) })
      void qc.invalidateQueries({ queryKey: qk.cameras })
    },
  })
}

export function useDeleteCalibration(cameraId: number) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: () => deleteCalibration(cameraId),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.cameraCalibration(cameraId) })
      void qc.invalidateQueries({ queryKey: qk.cameras })
    },
  })
}
