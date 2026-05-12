// Exchange an SDP offer for an SDP answer with the server.
export async function postOffer(offerSdp: string): Promise<string> {
  const res = await fetch('/api/stream/offer', {
    method: 'POST',
    headers: { 'Content-Type': 'application/sdp' },
    body: offerSdp,
  })
  if (!res.ok) {
    const detail = await res.text()
    throw new Error(`Signaling failed (${res.status}): ${detail}`)
  }
  return res.text()
}
