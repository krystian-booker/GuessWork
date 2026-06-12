import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import { fetchCanConfig, fetchCanStatus, postBenchPose, updateCanConfig } from '@/api/can'
import { CONFIG_STALE_MS, POLL_MEDIUM } from '@/lib/query-client'
import { qk } from './keys'

export function useCanStatus() {
  return useQuery({
    queryKey: qk.canStatus,
    queryFn: fetchCanStatus,
    refetchInterval: POLL_MEDIUM,
  })
}

export function useCanConfig() {
  return useQuery({
    queryKey: qk.canConfig,
    queryFn: fetchCanConfig,
    staleTime: CONFIG_STALE_MS,
  })
}

export function useUpdateCanConfig() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: updateCanConfig,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.canConfig })
      void qc.invalidateQueries({ queryKey: qk.canStatus })
    },
  })
}

export function useBenchPose() {
  return useMutation({ mutationFn: postBenchPose })
}
