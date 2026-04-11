const SSH_AUTH_ERROR_RE = /(permission denied|authentication failed|auth fail|incorrect password|access denied|password was rejected|password authentication is required|no ssh password is set)/i
const SSH_PASSWORD_HINT = 'If this target requires password auth, open Settings and enter the SSH password.'

let passwordPromptInFlight = null

function isSshAuthError(error) {
  return typeof error === 'string' && SSH_AUTH_ERROR_RE.test(error)
}

function shouldPromptForAuth(opts) {
  const method = (opts?.method || 'GET').toUpperCase()
  return method !== 'GET'
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

async function promptSshPassword() {
  if (!passwordPromptInFlight) {
    passwordPromptInFlight = Promise.resolve(
      window.prompt('SSH authentication failed. Enter the target SSH password to retry:', '')
    ).finally(() => {
      passwordPromptInFlight = null
    })
  }
  return passwordPromptInFlight
}

function targetIdFromRequest(url, opts) {
  try {
    const parsed = new URL(url, window.location.origin)
    const fromQuery = parsed.searchParams.get('target_id')
    if (fromQuery) return fromQuery
  } catch {
    // Ignore URL parsing failure and fall back to FormData.
  }

  if (opts?.body instanceof FormData) {
    const fromBody = opts.body.get('target_id')
    if (typeof fromBody === 'string' && fromBody.trim()) return fromBody.trim()
  }
  return ''
}

async function updateSshPassword(targetId, password) {
  if (!targetId) {
    return {
      success: false,
      error: 'No target is selected for SSH password update.',
    }
  }
  const fd = new FormData()
  fd.append('target_id', targetId)
  fd.append('ssh_pass', password)
  return requestJson('/api/inventory', { method: 'POST', body: fd })
}

export async function api(url, opts, hasRetriedAuth = false) {
  const data = await requestJson(url, opts)
  const targetId = targetIdFromRequest(url, opts)

  if (
    !hasRetriedAuth &&
    shouldPromptForAuth(opts) &&
    !url.startsWith('/api/inventory') &&
    !data.success &&
    isSshAuthError(data.error)
  ) {
    const password = await promptSshPassword()
    if (password === null) {
      return {
        success: false,
        error: withSshPasswordHint('SSH authentication failed. Password update cancelled.'),
      }
    }

    const updateRes = await updateSshPassword(targetId, password)
    if (!updateRes.success) {
      return {
        success: false,
        error: updateRes.error || 'Failed to update SSH password.',
      }
    }

    return api(url, opts, true)
  }

  if (!data.success && isSshAuthError(data.error)) {
    return {
      ...data,
      error: withSshPasswordHint(data.error),
    }
  }

  return data
}
