import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  cancelExtrinsicsJob,
  deleteCameraExtrinsics,
  getCameraExtrinsics,
  getExtrinsicsJob,
  getExtrinsicsRecording,
  startExtrinsicsRecording,
  stopExtrinsicsRecording,
} from '@/api/extrinsics'
import { POLL_MEDIUM, POLL_RECORDING } from '@/lib/query-client'
import { qk } from './keys'

export function useExtrinsicsRecording(opts?: { poll?: boolean }) {
  return useQuery({
    queryKey: qk.extrinsicsRecording,
    queryFn: getExtrinsicsRecording,
    refetchInterval: (opts?.poll ?? true) ? POLL_RECORDING : false,
  })
}

export function useExtrinsicsJob(opts?: { poll?: boolean }) {
  return useQuery({
    queryKey: qk.extrinsicsJob,
    queryFn: getExtrinsicsJob,
    refetchInterval: (opts?.poll ?? true) ? POLL_MEDIUM : false,
  })
}

export function useCameraExtrinsics(cameraId: number) {
  return useQuery({
    queryKey: qk.cameraExtrinsics(cameraId),
    queryFn: () => getCameraExtrinsics(cameraId),
  })
}

export function useStartExtrinsicsRecording() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: startExtrinsicsRecording,
    onSuccess: (status) => qc.setQueryData(qk.extrinsicsRecording, status),
  })
}

export function useStopExtrinsicsRecording() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: stopExtrinsicsRecording,
    onSuccess: (resp) => {
      qc.setQueryData(qk.extrinsicsRecording, null)
      qc.setQueryData(qk.extrinsicsJob, resp.job)
    },
  })
}

export function useCancelExtrinsicsJob() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: cancelExtrinsicsJob,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.extrinsicsJob }),
  })
}

export function useDeleteCameraExtrinsics() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: deleteCameraExtrinsics,
    onSuccess: (_data, cameraId) => {
      void qc.invalidateQueries({ queryKey: qk.cameraExtrinsics(cameraId) })
      void qc.invalidateQueries({ queryKey: qk.cameras })
    },
  })
}
