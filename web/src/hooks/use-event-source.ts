import { useEffect, useRef } from 'react'

export interface EventSourceHandlers {
  // Default `message` events (the server emits `data: …` lines without an
  // explicit `event:` header for log frames).
  onMessage?: (data: string) => void
  // Named events, e.g. { done: (data) => ... }. The source closes itself
  // after a named handler if `closeOn` includes the event name.
  namedEvents?: Record<string, (data: string) => void>
  // Event names that terminate the stream (EventSource is closed after the
  // handler runs). Defaults to ['done'].
  closeOn?: string[]
  // Network blip / server restart. The stream auto-reconnects unless closed.
  onError?: (e: Event) => void
}

// Generic SSE subscription. Pass url=null to disable. Closes on unmount and
// whenever the url changes. Handlers are kept in a ref so re-renders don't
// tear down the connection.
export function useEventSource(url: string | null, handlers: EventSourceHandlers) {
  const handlersRef = useRef(handlers)
  handlersRef.current = handlers

  useEffect(() => {
    if (!url) return
    const es = new EventSource(url)
    es.onmessage = (e) => handlersRef.current.onMessage?.(e.data)
    es.onerror = (e) => handlersRef.current.onError?.(e)

    const closeOn = handlersRef.current.closeOn ?? ['done']
    for (const name of Object.keys(handlersRef.current.namedEvents ?? {})) {
      es.addEventListener(name, (e) => {
        handlersRef.current.namedEvents?.[name]?.((e as MessageEvent).data)
        if (closeOn.includes(name)) es.close()
      })
    }
    return () => es.close()
  }, [url])
}
