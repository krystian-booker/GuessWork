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
  if (!res.ok) throw new Error(await readError(res))
  return (await res.json()) as T
}
