import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  arm,
  createGroup,
  deleteGroup,
  getStatus,
  listGroups,
  stopOutputs,
  updateGroup,
} from '@/api/hardwareSync'
import { POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useHwSyncStatus() {
  return useQuery({
    queryKey: qk.hwSyncStatus,
    queryFn: getStatus,
    refetchInterval: POLL_FAST,
  })
}

export function useTriggerGroups() {
  return useQuery({
    queryKey: qk.hwSyncGroups,
    queryFn: listGroups,
  })
}

export function useCreateTriggerGroup() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: createGroup,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.hwSyncGroups }),
  })
}

export function useUpdateTriggerGroup() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({ id, patch }: { id: number; patch: Parameters<typeof updateGroup>[1] }) =>
      updateGroup(id, patch),
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.hwSyncGroups }),
  })
}

export function useDeleteTriggerGroup() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: deleteGroup,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.hwSyncGroups }),
  })
}

export function useArmHwSync() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: arm,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.hwSyncStatus }),
  })
}

export function useStopHwSync() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: stopOutputs,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.hwSyncStatus }),
  })
}
