// Error carrying the HTTP status so callers can branch on 404/409/503
// without parsing message strings.
export class ApiError extends Error {
  constructor(
    message: string,
    readonly status: number,
  ) {
    super(message)
    this.name = 'ApiError'
  }
}

// Reads the server's JSON error envelope ({ error: string }), falling back to
// the HTTP status when the body isn't parseable JSON or doesn't carry an
// `error` field.
export async function readError(res: Response): Promise<string> {
  try {
    const body = await res.json()
    if (body && typeof body.error === 'string') return body.error
  } catch {
    // fall through
  }
  return `HTTP ${res.status}`
}

export async function asJson<T>(res: Response): Promise<T> {
  if (!res.ok) throw new ApiError(await readError(res), res.status)
  return (await res.json()) as T
}

// Some endpoints reply with a full result body on a specific non-2xx status
// (e.g. DELETE …/recording returns 409 with the kept recording_result when
// the Kalibr launch was rejected). Treat those statuses as values, not
// errors, so callers can surface `job_error` instead of a bare toast.
export async function asJsonSoft<T>(res: Response, softStatuses: number[]): Promise<T> {
  if (softStatuses.includes(res.status)) {
    try {
      return (await res.json()) as T
    } catch {
      throw new ApiError(`HTTP ${res.status}`, res.status)
    }
  }
  return asJson<T>(res)
}

export async function getJson<T>(url: string): Promise<T> {
  return asJson<T>(await fetch(url))
}

// 404 means "nothing here yet" for recording/job/calibration getters — map it
// to null instead of throwing.
export async function getJsonOrNull<T>(url: string): Promise<T | null> {
  const res = await fetch(url)
  if (res.status === 404) return null
  return asJson<T>(res)
}

export async function sendJson<T>(
  url: string,
  method: 'POST' | 'PUT' | 'DELETE',
  body?: unknown,
): Promise<T> {
  const res = await fetch(url, {
    method,
    ...(body !== undefined
      ? { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }
      : {}),
  })
  return asJson<T>(res)
}

// For routes that return 204 No Content (or where the body is irrelevant).
export async function send(
  url: string,
  method: 'POST' | 'PUT' | 'DELETE',
  body?: unknown,
): Promise<void> {
  const res = await fetch(url, {
    method,
    ...(body !== undefined
      ? { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }
      : {}),
  })
  if (!res.ok) throw new ApiError(await readError(res), res.status)
}
