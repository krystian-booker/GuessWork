import { useQuery } from '@tanstack/react-query'
import { fetchAprilTagStatus } from '@/api/apriltag'
import { POLL_FAST } from '@/lib/query-client'
import { qk } from './keys'

export function useAprilTagStatus() {
  return useQuery({
    queryKey: qk.apriltagStatus,
    queryFn: fetchAprilTagStatus,
    refetchInterval: POLL_FAST,
  })
}
