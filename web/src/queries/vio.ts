import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import { fetchVioConfig, fetchVioStatus, restartVio, updateVioConfig } from '@/api/vio'
import { CONFIG_STALE_MS, POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useVioStatus() {
  return useQuery({
    queryKey: qk.vioStatus,
    queryFn: fetchVioStatus,
    refetchInterval: POLL_FAST,
  })
}

export function useVioConfig() {
  return useQuery({
    queryKey: qk.vioConfig,
    queryFn: fetchVioConfig,
    staleTime: CONFIG_STALE_MS,
  })
}

export function useUpdateVioConfig() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: updateVioConfig,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.vioConfig })
      void qc.invalidateQueries({ queryKey: qk.vioStatus })
    },
  })
}

export function useRestartVio() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: restartVio,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.vioStatus }),
  })
}
