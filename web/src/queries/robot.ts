import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import { fetchRobotConfig, fetchRobotStatus, postBenchPose, updateRobotConfig } from '@/api/robot'
import { CONFIG_STALE_MS, POLL_MEDIUM } from '@/lib/query-client'
import { qk } from './keys'

export function useRobotStatus() {
  return useQuery({
    queryKey: qk.robotStatus,
    queryFn: fetchRobotStatus,
    refetchInterval: POLL_MEDIUM,
  })
}

export function useRobotConfig() {
  return useQuery({
    queryKey: qk.robotConfig,
    queryFn: fetchRobotConfig,
    staleTime: CONFIG_STALE_MS,
  })
}

export function useUpdateRobotConfig() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: updateRobotConfig,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.robotConfig })
      void qc.invalidateQueries({ queryKey: qk.robotStatus })
    },
  })
}

export function useBenchPose() {
  return useMutation({ mutationFn: postBenchPose })
}
