// Exchange an SDP offer for an SDP answer with the server, scoped to a camera.
export async function postOffer(cameraId: number, offerSdp: string): Promise<string> {
  const res = await fetch(`/api/cameras/${cameraId}/stream/offer`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/sdp' },
    body: offerSdp,
  })
  if (!res.ok) {
    // Server emits text/plain errors for this route, not the JSON envelope.
    const detail = await res.text()
    throw new Error(`Signaling failed (${res.status}): ${detail}`)
  }
  return res.text()
}
