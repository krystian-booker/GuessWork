import { useQuery } from '@tanstack/react-query'
import { fetchStatus } from '@/api/status'
import { POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useStatus() {
  return useQuery({
    queryKey: qk.status,
    queryFn: fetchStatus,
    refetchInterval: POLL_FAST,
  })
}
