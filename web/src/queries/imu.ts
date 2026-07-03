import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  analyzeAllan,
  applyAllan,
  fetchAllanStatus,
  fetchImuAttitude,
  fetchImuConfig,
  fetchImuStatus,
  postZeroYaw,
  startAllanRecording,
  stopAllanRecording,
  updateImuConfig,
} from '@/api/imu'
import { CONFIG_STALE_MS, POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useImuStatus() {
  return useQuery({
    queryKey: qk.imuStatus,
    queryFn: fetchImuStatus,
    refetchInterval: POLL_FAST,
  })
}

// Deliberately faster than the global polling tiers: the 3D attitude viewer
// slerps toward each new sample, and 150 ms targets keep the motion tight.
// Polls only while the viewer is mounted.
export function useImuAttitude() {
  return useQuery({
    queryKey: qk.imuAttitude,
    queryFn: fetchImuAttitude,
    refetchInterval: 150,
  })
}

export function useZeroYaw() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: postZeroYaw,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.imuAttitude }),
  })
}

export function useImuConfig() {
  return useQuery({
    queryKey: qk.imuConfig,
    queryFn: fetchImuConfig,
    staleTime: CONFIG_STALE_MS,
  })
}

export function useUpdateImuConfig() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: updateImuConfig,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.imuConfig })
      // T_robot_imu gates AprilTag publishing and fusion VIO ingestion.
      void qc.invalidateQueries({ queryKey: qk.apriltagStatus })
      void qc.invalidateQueries({ queryKey: qk.fusionStatus })
    },
  })
}

export function useAllanStatus(opts?: { poll?: boolean }) {
  return useQuery({
    queryKey: qk.imuAllan,
    queryFn: fetchAllanStatus,
    refetchInterval: (opts?.poll ?? true) ? POLL_FAST : false,
  })
}

export function useStartAllanRecording() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: startAllanRecording,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.imuAllan }),
  })
}

export function useStopAllanRecording() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: stopAllanRecording,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.imuAllan }),
  })
}

export function useAnalyzeAllan() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (file?: string) => analyzeAllan(file),
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.imuAllan }),
  })
}

export function useApplyAllan() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: applyAllan,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.imuConfig })
      void qc.invalidateQueries({ queryKey: qk.imuAllan })
    },
  })
}
