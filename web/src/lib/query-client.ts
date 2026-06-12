import { MutationCache, QueryClient } from '@tanstack/react-query'
import { toast } from 'sonner'

// Polling tiers (ms). Status endpoints poll only while a consuming component
// is mounted — TanStack stops the interval when the last observer unmounts.
export const POLL_FAST = 1000 // live status (fusion, vio, apriltag, /api/status)
export const POLL_MEDIUM = 2000 // can/teensy status, job state (SSE safety net)
export const POLL_RECORDING = 750 // active recording / Allan progress
export const POLL_SLOW = 5000 // camera list and other near-static lists
export const CONFIG_STALE_MS = 30_000 // configs refetch via invalidation on PUT

export const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      refetchIntervalInBackground: false,
      retry: 1,
    },
  },
  // Surface 409/503 error envelopes ("role 'vio_left' already assigned",
  // "teensy not connected", ...) uniformly. Mutations that render errors
  // inline can opt out with meta.silent.
  mutationCache: new MutationCache({
    onError: (error, _variables, _context, mutation) => {
      if (mutation.meta?.silent) return
      toast.error(error instanceof Error ? error.message : String(error))
    },
  }),
})

declare module '@tanstack/react-query' {
  interface Register {
    mutationMeta: { silent?: boolean }
  }
}
