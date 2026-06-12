import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  fetchFusionConfig,
  fetchFusionStatus,
  resetFusion,
  updateFusionConfig,
} from '@/api/fusion'
import { CONFIG_STALE_MS, POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useFusionStatus() {
  return useQuery({
    queryKey: qk.fusionStatus,
    queryFn: fetchFusionStatus,
    refetchInterval: POLL_FAST,
  })
}

export function useFusionConfig() {
  return useQuery({
    queryKey: qk.fusionConfig,
    queryFn: fetchFusionConfig,
    staleTime: CONFIG_STALE_MS,
  })
}

export function useUpdateFusionConfig() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: updateFusionConfig,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.fusionConfig })
      void qc.invalidateQueries({ queryKey: qk.fusionStatus })
    },
  })
}

export function useResetFusion() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: resetFusion,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.fusionStatus }),
  })
}
