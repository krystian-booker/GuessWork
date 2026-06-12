import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import {
  activateFieldLayout,
  createFieldLayout,
  deleteFieldLayout,
  getFieldLayout,
  listFieldLayouts,
} from '@/api/fieldLayouts'
import { qk } from './keys'

export function useFieldLayouts() {
  return useQuery({
    queryKey: qk.fieldLayouts,
    queryFn: listFieldLayouts,
  })
}

export function useFieldLayout(id: number | null) {
  return useQuery({
    queryKey: id != null ? qk.fieldLayout(id) : ['field-layouts', 'none'],
    queryFn: () => getFieldLayout(id!),
    enabled: id != null,
    staleTime: Infinity, // layout JSON is immutable per id
  })
}

export function useCreateFieldLayout() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: createFieldLayout,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.fieldLayouts }),
  })
}

export function useActivateFieldLayout() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: activateFieldLayout,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: qk.fieldLayouts })
      void qc.invalidateQueries({ queryKey: qk.apriltagStatus })
    },
  })
}

export function useDeleteFieldLayout() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: deleteFieldLayout,
    onSuccess: () => void qc.invalidateQueries({ queryKey: qk.fieldLayouts }),
  })
}
