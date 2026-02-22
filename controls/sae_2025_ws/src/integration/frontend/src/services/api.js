const SSH_AUTH_ERROR_RE = /(permission denied|authentication failed|auth fail|incorrect password|access denied|password was rejected|password authentication is required|no ssh password is set)/i
const SSH_PASSWORD_HINT = 'If your Pi requires password auth, open Settings and enter the SSH password.'

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

export async function requestJson(url, opts) {
  try {
    const res = await fetch(url, opts)
    const raw = await res.text()

    if (!raw.trim()) {
      return {
        success: false,
        error: `Backend returned an empty response (HTTP ${res.status}). Are you connected to the Pi and is the backend reachable?`,
      }
    }

    let data
    try {
      data = JSON.parse(raw)
    } catch {
      const preview = raw.slice(0, 180).replace(/\s+/g, ' ').trim()
      return {
        success: false,
        error: `Backend returned an invalid response (HTTP ${res.status}). Are you connected to the Pi and is the backend reachable? ${preview ? `Details: ${preview}` : ''}`.trim(),
      }
    }

    if (!res.ok && (typeof data !== 'object' || data === null || data.success === undefined)) {
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
      window.prompt('SSH authentication failed. Enter the Pi SSH password to retry:', '')
    ).finally(() => {
      passwordPromptInFlight = null
    })
  }
  return passwordPromptInFlight
}

async function updateSshPassword(password) {
  const fd = new FormData()
  fd.append('ssh_pass', password)
  return requestJson('/api/config', { method: 'POST', body: fd })
}

export async function api(url, opts, hasRetriedAuth = false) {
  const data = await requestJson(url, opts)

  if (
    !hasRetriedAuth &&
    shouldPromptForAuth(opts) &&
    url !== '/api/config' &&
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

    const updateRes = await updateSshPassword(password)
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
