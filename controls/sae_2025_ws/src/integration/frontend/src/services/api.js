const SSH_AUTH_ERROR_RE = /(permission denied|authentication failed|auth fail|incorrect password|access denied|password was rejected|password authentication is required|no ssh password is set)/i
const SSH_PASSWORD_HINT = 'If this target requires password auth, open Settings and enter the SSH password.'

function isSshAuthError(error) {
  return typeof error === 'string' && SSH_AUTH_ERROR_RE.test(error)
}

function withSshPasswordHint(error) {
  if (typeof error !== 'string') return error
  if (error.includes(SSH_PASSWORD_HINT)) return error
  return `${error} ${SSH_PASSWORD_HINT}`
}

export function withQueryParams(url, params = {}) {
  const parsed = new URL(url, window.location.origin)
  Object.entries(params).forEach(([key, value]) => {
    if (value === undefined || value === null || `${value}` === '') return
    parsed.searchParams.set(key, `${value}`)
  })
  return `${parsed.pathname}${parsed.search}${parsed.hash}`
}

export function withTargetId(url, targetId) {
  return withQueryParams(url, { target_id: targetId })
}

export function withTargetFormData(formData, targetId) {
  if (!formData || !targetId) return formData
  formData.set('target_id', targetId)
  return formData
}

export async function requestJson(url, opts) {
  try {
    const res = await fetch(url, opts)
    const raw = await res.text()

    if (!raw.trim()) {
      return {
        success: false,
        error: `Backend returned an empty response (HTTP ${res.status}). Is the selected target reachable and is the backend up?`,
      }
    }

    let data
    try {
      data = JSON.parse(raw)
    } catch {
      const preview = raw.slice(0, 180).replace(/\s+/g, ' ').trim()
        return {
          success: false,
          error: `Backend returned an invalid response (HTTP ${res.status}). Is the selected target reachable and is the backend up? ${preview ? `Details: ${preview}` : ''}`.trim(),
        }
    }

    if (!res.ok && (typeof data !== 'object' || data === null || data.success === undefined)) {
      if (typeof data?.detail === 'string' && data.detail.trim()) {
        return {
          success: false,
          error: data.detail,
        }
      }
      return {
        success: false,
        error: `HTTP ${res.status}`,
      }
    }

    return data
  } catch (e) {
    return { success: false, error: e.message }
  }
}

export async function api(url, opts) {
  const data = await requestJson(url, opts)

  if (!data.success && isSshAuthError(data.error)) {
    return {
      ...data,
      error: withSshPasswordHint(data.error),
    }
  }

  return data
}
