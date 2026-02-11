import { useState, useEffect, useCallback, useRef, useMemo } from 'react'
import './App.css'

const SSH_AUTH_ERROR_RE = /(permission denied|authentication failed|auth fail|incorrect password|access denied|password was rejected|password authentication is required|no ssh password is set)/i
const SSH_PASSWORD_HINT = 'If your Pi requires password auth, open Settings and enter the SSH password.'
let passwordPromptInFlight = null

const LAUNCH_PARAM_FIELDS = [
  {
    key: 'mission_name',
    label: 'Mission name',
    type: 'string',
    help: 'Mission profile identifier used by launch.',
  },
  {
    key: 'uav_debug',
    label: 'UAV debug',
    type: 'boolean',
    help: 'Enable extra UAV-side debug outputs.',
  },
  {
    key: 'vision_debug',
    label: 'Vision debug',
    type: 'boolean',
    help: 'Enable extra vision pipeline debug outputs.',
  },
  {
    key: 'run_mission',
    label: 'Run mission',
    type: 'boolean',
    help: 'If true, mission flow starts after launch prepares.',
  },
  {
    key: 'airframe',
    label: 'Airframe',
    type: 'string',
    help: 'Preset or valid PX4 airframe ID (e.g., quadcopter, tiltrotor_vtol, fixed_wing, standard_vtol, quadtailsitter, 4014).',
  },
  {
    key: 'use_camera',
    label: 'Use camera',
    type: 'boolean',
    help: 'Enable camera node and camera-driven functionality.',
  },
  {
    key: 'custom_airframe_model',
    label: 'Custom airframe model',
    type: 'string',
    help: 'Leave blank for default PX4 model, or provide custom model file name.',
  },
  {
    key: 'save_vision',
    label: 'Save vision',
    type: 'boolean',
    help: 'Save vision output artifacts for debugging.',
  },
  {
    key: 'servo_only',
    label: 'Servo only',
    type: 'boolean',
    help: 'Run servo-only behavior without full mission flow.',
  },
  {
    key: 'camera_offsets',
    label: 'Camera offsets (x,y,z)',
    type: 'array3',
    help: 'Camera position relative to payload mechanism (meters, NED: x forward, y right, z down).',
  },
  {
    key: 'sim',
    label: 'Simulation mode',
    type: 'boolean',
    help: 'Enable simulation-specific settings.',
  },
]

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

function escapeRegex(value) {
  return value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
}

function splitInlineComment(valuePart) {
  const idx = valuePart.indexOf(' #')
  if (idx >= 0) {
    return {
      value: valuePart.slice(0, idx).trim(),
      comment: valuePart.slice(idx),
    }
  }
  return { value: valuePart.trim(), comment: '' }
}

function unquoteYamlScalar(value) {
  if ((value.startsWith("'") && value.endsWith("'")) || (value.startsWith('"') && value.endsWith('"'))) {
    return value.slice(1, -1)
  }
  return value
}

function parseYamlScalar(raw, type) {
  const value = raw.trim()
  if (type === 'boolean') {
    return value.toLowerCase() === 'true'
  }
  if (type === 'array3') {
    const stripped = value.replace(/^\[/, '').replace(/\]$/, '')
    const parts = stripped.split(',').map(p => p.trim()).filter(Boolean)
    if (parts.length !== 3) return [0, 0, 0]
    return parts.map(p => Number(p) || 0)
  }
  return unquoteYamlScalar(value)
}

function serializeYamlScalar(value, type) {
  if (type === 'boolean') {
    return value ? 'true' : 'false'
  }
  if (type === 'array3') {
    const arr = Array.isArray(value) ? value : [0, 0, 0]
    const normalized = [arr[0], arr[1], arr[2]].map(v => Number(v) || 0)
    return `[${normalized.join(', ')}]`
  }

  const text = `${value ?? ''}`
  if (text === '') return "''"
  if (/[:#\s]/.test(text)) {
    return `'${text.replace(/'/g, "''")}'`
  }
  return text
}

function getYamlFieldValue(content, key, type) {
  const keyRe = new RegExp(`^\\s*${escapeRegex(key)}\\s*:\\s*(.*)$`, 'm')
  const match = content.match(keyRe)
  if (!match) {
    if (type === 'boolean') return false
    if (type === 'array3') return [0, 0, 0]
    return ''
  }

  const { value } = splitInlineComment(match[1])
  return parseYamlScalar(value, type)
}

function setYamlFieldValue(content, key, type, value) {
  const lines = content.split('\n')
  const keyRe = new RegExp(`^(\\s*${escapeRegex(key)}\\s*:\\s*)(.*)$`)
  const serialized = serializeYamlScalar(value, type)
  let updated = false

  const nextLines = lines.map(line => {
    const match = line.match(keyRe)
    if (!match) return line
    updated = true
    const prefix = match[1]
    const rest = match[2]
    const { comment } = splitInlineComment(rest)
    return `${prefix}${serialized}${comment}`
  })

  if (!updated) {
    nextLines.push(`${key}: ${serialized}`)
  }

  return nextLines.join('\n')
}

const ANSI_COLOR_16 = [
  '#111827', '#ef4444', '#22c55e', '#eab308', '#3b82f6', '#a855f7', '#06b6d4', '#d1d5db',
  '#6b7280', '#f87171', '#4ade80', '#facc15', '#60a5fa', '#c084fc', '#22d3ee', '#f9fafb',
]

function toHex(value) {
  return value.toString(16).padStart(2, '0')
}

function ansi256ToColor(code) {
  const n = Math.max(0, Math.min(255, Number(code) || 0))
  if (n < 16) return ANSI_COLOR_16[n]
  if (n >= 232) {
    const gray = 8 + (n - 232) * 10
    const hex = toHex(gray)
    return `#${hex}${hex}${hex}`
  }
  const idx = n - 16
  const r = Math.floor(idx / 36)
  const g = Math.floor((idx % 36) / 6)
  const b = idx % 6
  const map = [0, 95, 135, 175, 215, 255]
  return `#${toHex(map[r])}${toHex(map[g])}${toHex(map[b])}`
}

function defaultAnsiState() {
  return {
    fg: null,
    bg: null,
    bold: false,
    dim: false,
    underline: false,
    inverse: false,
  }
}

function ansiStateToStyle(state) {
  let fg = state.fg
  let bg = state.bg
  if (state.inverse) {
    const defaultFg = '#d6d6e2'
    const defaultBg = '#0c0c12'
    const origFg = fg || defaultFg
    const origBg = bg || defaultBg
    fg = origBg
    bg = origFg
  }

  const style = {}
  if (fg) style.color = fg
  if (bg) style.backgroundColor = bg
  if (state.bold) style.fontWeight = 700
  if (state.dim) style.opacity = 0.75
  if (state.underline) style.textDecoration = 'underline'
  return style
}

function applyAnsiCodes(state, codes) {
  const next = { ...state }
  for (let i = 0; i < codes.length; i += 1) {
    const code = codes[i]
    if (code === 0) {
      Object.assign(next, defaultAnsiState())
    } else if (code === 1) {
      next.bold = true
    } else if (code === 2) {
      next.dim = true
    } else if (code === 21 || code === 22) {
      next.bold = false
      next.dim = false
    } else if (code === 4) {
      next.underline = true
    } else if (code === 24) {
      next.underline = false
    } else if (code === 7) {
      next.inverse = true
    } else if (code === 27) {
      next.inverse = false
    } else if (code === 39) {
      next.fg = null
    } else if (code === 49) {
      next.bg = null
    } else if (code >= 30 && code <= 37) {
      next.fg = ANSI_COLOR_16[code - 30]
    } else if (code >= 90 && code <= 97) {
      next.fg = ANSI_COLOR_16[8 + (code - 90)]
    } else if (code >= 40 && code <= 47) {
      next.bg = ANSI_COLOR_16[code - 40]
    } else if (code >= 100 && code <= 107) {
      next.bg = ANSI_COLOR_16[8 + (code - 100)]
    } else if (code === 38 || code === 48) {
      const target = code === 38 ? 'fg' : 'bg'
      const mode = codes[i + 1]
      if (mode === 5 && i + 2 < codes.length) {
        next[target] = ansi256ToColor(codes[i + 2])
        i += 2
      } else if (mode === 2 && i + 4 < codes.length) {
        const r = Math.max(0, Math.min(255, Number(codes[i + 2]) || 0))
        const g = Math.max(0, Math.min(255, Number(codes[i + 3]) || 0))
        const b = Math.max(0, Math.min(255, Number(codes[i + 4]) || 0))
        next[target] = `#${toHex(r)}${toHex(g)}${toHex(b)}`
        i += 4
      }
    }
  }
  return next
}

function parseAnsiSegments(raw) {
  const text = typeof raw === 'string' ? raw.replace(/\r\n/g, '\n').replace(/\r/g, '\n') : ''
  if (!text) return []

  // Accept standard ESC-prefixed ANSI codes and orphan forms like "[0m".
  const re = /(?:\x1b\[|\[)([0-9;]*)m/g
  let state = defaultAnsiState()
  let last = 0
  const segments = []
  let match

  while ((match = re.exec(text)) !== null) {
    if (match.index > last) {
      segments.push({
        text: text.slice(last, match.index),
        style: ansiStateToStyle(state),
      })
    }
    const codes = (match[1] === '' ? ['0'] : match[1].split(';'))
      .map(v => Number.parseInt(v, 10))
      .filter(v => Number.isFinite(v))
    state = applyAnsiCodes(state, codes.length ? codes : [0])
    last = re.lastIndex
  }

  if (last < text.length) {
    segments.push({
      text: text.slice(last),
      style: ansiStateToStyle(state),
    })
  }

  return segments
}

function AnsiLog({ text }) {
  const segments = useMemo(() => parseAnsiSegments(text), [text])
  if (!segments.length) return 'No launch output yet.'
  return segments.map((seg, i) => (
    <span key={i} style={seg.style}>{seg.text}</span>
  ))
}

function launchStateLabel(status) {
  if (status?.state === 'running') return 'Launch Running'
  if (status?.state === 'stopped') return 'Launch Stopped'
  if (status?.state === 'not_prepared') return 'Not Prepared'
  return 'Unknown'
}

function launchStateClass(status) {
  if (status?.state === 'running') return 'pill-running'
  if (status?.state === 'stopped') return 'pill-stopped'
  if (status?.state === 'not_prepared') return 'pill-not-prepared'
  return 'pill-stopped'
}

async function requestJson(url, opts) {
  try {
    const res = await fetch(url, opts)
    const raw = await res.text()

    if (!raw.trim()) {
      return {
        success: false,
        error: `Backend returned an empty response (HTTP ${res.status}). Are you connected to the Pi and is the deploy server reachable?`,
      }
    }

    let data
    try {
      data = JSON.parse(raw)
    } catch {
      const preview = raw.slice(0, 180).replace(/\s+/g, ' ').trim()
      return {
        success: false,
        error: `Backend returned an invalid response (HTTP ${res.status}). Are you connected to the Pi and is the deploy server reachable? ${preview ? `Details: ${preview}` : ''}`.trim(),
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

async function api(url, opts, hasRetriedAuth = false) {
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

function StatusBar({ connected, wifiStatus, buildInfo }) {
  return (
    <div className="status-bar">
      <div className="status-item">
        <span className={`status-dot ${connected ? 'dot-ok' : 'dot-err'}`} />
        <span>{connected ? 'Pi connected' : 'Pi unreachable'}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${wifiStatus?.current_wifi ? 'dot-ok' : 'dot-warn'}`} />
        <span>{wifiStatus?.is_hotspot ? 'Hotspot' : wifiStatus?.current_wifi || 'Unknown'}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${buildInfo?.installed ? 'dot-ok' : 'dot-warn'}`} />
        <span>{buildInfo?.installed ? 'Build active' : 'No build'}</span>
      </div>
    </div>
  )
}

function ConnectionCard({ sshCommand }) {
  const [copied, setCopied] = useState(false)

  const copy = () => {
    if (navigator.clipboard?.writeText) {
      navigator.clipboard.writeText(sshCommand).catch(() => fallbackCopy(sshCommand))
    } else {
      fallbackCopy(sshCommand)
    }
    setCopied(true)
    setTimeout(() => setCopied(false), 2000)
  }

  const fallbackCopy = (text) => {
    const el = document.createElement('textarea')
    el.value = text
    el.style.position = 'fixed'
    el.style.opacity = '0'
    document.body.appendChild(el)
    el.select()
    document.execCommand('copy')
    document.body.removeChild(el)
  }

  return (
    <div className="card card-full">
      <h2 className="card-title">Pi Connection</h2>
      <div className="ssh-box" onClick={copy}>
        <code>{sshCommand || '...'}</code>
        <span className="copy-tag">{copied ? 'Copied' : 'Copy'}</span>
      </div>
      <p className="subtext">Click to copy SSH command</p>
    </div>
  )
}

function WifiCard({ wifiStatus, onRefresh }) {
  const [networks, setNetworks] = useState([])
  const [scanning, setScanning] = useState(false)
  const [selectedSsid, setSelectedSsid] = useState('')
  const [password, setPassword] = useState('')
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState(null)

  const scan = async () => {
    setScanning(true)
    setResult(null)
    const data = await api('/api/wifi/scan')
    if (data.success) {
      setNetworks(data.networks)
      if (data.networks.length > 0 && !selectedSsid) {
        setSelectedSsid(data.networks[0].ssid)
      }
    } else {
      setResult(data)
    }
    setScanning(false)
  }

  const connect = async () => {
    if (!selectedSsid) return
    setLoading(true)
    setResult(null)
    const fd = new FormData()
    fd.append('ssid', selectedSsid)
    fd.append('password', password)
    const data = await api('/api/wifi/connect', { method: 'POST', body: fd })
    setResult(data)
    setLoading(false)
    if (data.success) {
      const macFd = new FormData()
      macFd.append('ssid', selectedSsid)
      macFd.append('password', password)
      await api('/api/wifi/switch-local', { method: 'POST', body: macFd })
    }
    onRefresh()
  }

  const hotspot = async () => {
    setLoading(true)
    setResult(null)
    const data = await api('/api/wifi/hotspot', { method: 'POST' })
    setResult(data)
    setLoading(false)
    onRefresh()
  }

  return (
    <div className="card">
      <h2 className="card-title">WiFi</h2>
      <div className="card-content">
        <button className="btn btn-secondary" onClick={scan} disabled={scanning}>
          {scanning ? 'Scanning...' : 'Scan networks'}
        </button>

        {networks.length > 0 && (
          <>
            <label>Network</label>
            <select value={selectedSsid} onChange={e => setSelectedSsid(e.target.value)}>
              {networks.map(n => (
                <option key={n.ssid} value={n.ssid}>
                  {n.ssid} — {n.signal}%{n.security ? ` · ${n.security}` : ''}
                </option>
              ))}
            </select>

            <label>Password</label>
            <input
              type="password"
              value={password}
              onChange={e => setPassword(e.target.value)}
              placeholder="Leave empty if open"
            />

            <button className="btn btn-primary" onClick={connect} disabled={loading}>
              {loading ? 'Connecting...' : 'Connect both devices'}
            </button>
          </>
        )}

        {!wifiStatus?.is_hotspot && (
          <button className="btn btn-secondary" onClick={hotspot} disabled={loading}>
            {loading ? 'Switching...' : 'Restore hotspot'}
          </button>
        )}

        <Result data={result} />
      </div>
    </div>
  )
}

function BuildCard({ buildInfo, onRefresh }) {
  const [file, setFile] = useState(null)
  const [uploading, setUploading] = useState(false)
  const [builds, setBuilds] = useState([])
  const [selectedTag, setSelectedTag] = useState('')
  const [loadingBuilds, setLoadingBuilds] = useState(false)
  const [downloading, setDownloading] = useState(false)
  const [result, setResult] = useState(null)

  const upload = async () => {
    if (!file) return
    setUploading(true)
    setResult(null)
    const fd = new FormData()
    fd.append('file', file)
    const data = await api('/api/builds/upload', { method: 'POST', body: fd })
    setResult(data)
    setUploading(false)
    setFile(null)
    onRefresh()
  }

  const listBuilds = async () => {
    setLoadingBuilds(true)
    setResult(null)
    const data = await api('/api/builds/list')
    if (data.success && data.builds) {
      setBuilds(data.builds)
      if (data.builds.length > 0 && !selectedTag) setSelectedTag(data.builds[0].tag)
    } else {
      setResult(data)
    }
    setLoadingBuilds(false)
  }

  const download = async () => {
    if (!selectedTag) return
    setDownloading(true)
    setResult(null)
    const fd = new FormData()
    fd.append('tag', selectedTag)
    const data = await api('/api/builds/download', { method: 'POST', body: fd })
    setResult(data)
    setDownloading(false)
    onRefresh()
  }

  const rollback = async () => {
    setResult(null)
    const data = await api('/api/builds/rollback', { method: 'POST' })
    setResult(data)
    onRefresh()
  }

  return (
    <div className="card">
      <h2 className="card-title">Build Deploy</h2>
      <div className="card-content">
        {buildInfo?.info && (
          <div className="info-box">
            <pre>{buildInfo.info}</pre>
          </div>
        )}

        <label>Upload artifact</label>
        <div className="file-upload">
          <input
            type="file"
            accept=".tar.gz,.tgz,.tar,.gz,application/gzip,application/x-gzip,application/x-tar,application/octet-stream"
            onChange={e => setFile(e.target.files?.[0] || null)}
            id="build-file"
          />
          <label htmlFor="build-file" className="file-label">
            {file ? file.name : 'Choose .tar.gz'}
          </label>
        </div>

        <button className="btn btn-primary" onClick={upload} disabled={uploading || !file}>
          {uploading ? 'Uploading...' : 'Upload & replace install'}
        </button>

        <div className="divider" />

        <label>From GitHub</label>
        <button className="btn btn-secondary" onClick={listBuilds} disabled={loadingBuilds}>
          {loadingBuilds ? 'Loading...' : 'Fetch releases'}
        </button>

        {builds.length > 0 && (
          <>
            <select value={selectedTag} onChange={e => setSelectedTag(e.target.value)}>
              {builds.map(b => (
                <option key={b.tag} value={b.tag}>
                  {b.sha} — {b.date}{b.size_mb ? ` · ${b.size_mb} MB` : ''}
                </option>
              ))}
            </select>

            <button className="btn btn-primary" onClick={download} disabled={downloading}>
              {downloading ? 'Downloading...' : 'Download & deploy'}
            </button>
          </>
        )}

        {buildInfo?.installed && (
          <>
            <div className="divider" />
            <button className="btn btn-secondary" onClick={rollback}>
              Rollback
            </button>
          </>
        )}

        <Result data={result} />
      </div>
    </div>
  )
}

function SettingsPanel({ onRefresh }) {
  const [open, setOpen] = useState(false)
  const [cfg, setCfg] = useState(null)
  const [saving, setSaving] = useState(false)
  const [result, setResult] = useState(null)

  const load = async () => {
    const data = await api('/api/config')
    if (data.success) setCfg(data.config)
  }

  const toggle = () => {
    if (!open && !cfg) load()
    setOpen(o => !o)
  }

  const update = (key, value) => {
    setCfg(prev => ({ ...prev, [key]: value }))
  }

  const save = async () => {
    if (!cfg) return
    setSaving(true)
    setResult(null)
    const fd = new FormData()
    Object.entries(cfg).forEach(([k, v]) => fd.append(k, v))
    const data = await api('/api/config', { method: 'POST', body: fd })
    setResult(data)
    setSaving(false)
    onRefresh()
  }

  const fields = [
    { key: 'pi_host', label: 'Pi host', placeholder: 'penn-desktop.local' },
    { key: 'pi_user', label: 'Pi user', placeholder: 'penn' },
    { key: 'ssh_pass', label: 'SSH password', placeholder: 'Leave blank for SSH key auth', type: 'password' },
    { key: 'ssh_key', label: 'SSH key path', placeholder: '~/.ssh/id_rsa' },
    { key: 'remote_dir', label: 'Remote directory', placeholder: '/home/penn/monorepo/controls/sae_2025_ws' },
    { key: 'github_repo', label: 'GitHub repo', placeholder: 'org/repo' },
    { key: 'hotspot_name', label: 'Hotspot connection name', placeholder: 'penn-desktop' },
  ]

  return (
    <div className="settings-floating">
      <button className="settings-fab" onClick={toggle}>
        {open ? 'Close Settings' : 'Settings'}
      </button>

      {open && cfg && (
        <div className="card settings-popover">
          <h2 className="card-title">Settings</h2>
          <div className="card-content settings-grid">
            {fields.map(f => (
              <div key={f.key} className="settings-field">
                <label>{f.label}</label>
                <input
                  type={f.type || 'text'}
                  value={cfg[f.key] || ''}
                  onChange={e => update(f.key, e.target.value)}
                  placeholder={f.placeholder}
                />
              </div>
            ))}
            <div className="settings-field settings-save">
              <button className="btn btn-primary" onClick={save} disabled={saving}>
                {saving ? 'Saving...' : 'Save'}
              </button>
            </div>
            <Result data={result} />
          </div>
        </div>
      )}
    </div>
  )
}

function MissionControl({ buildInfo, onRefresh }) {
  const [actionLoading, setActionLoading] = useState('')
  const [actionResult, setActionResult] = useState(null)

  const [launchStatus, setLaunchStatus] = useState({ running: false, state: 'unknown' })
  const [logs, setLogs] = useState('')
  const [logsResult, setLogsResult] = useState(null)
  const logsOffsetRef = useRef(0)
  const logsRefreshInFlightRef = useRef(false)

  const [paramsMode, setParamsMode] = useState('form')
  const [paramsText, setParamsText] = useState('')
  const [paramsLoading, setParamsLoading] = useState(false)
  const [paramsResult, setParamsResult] = useState(null)

  const refreshLaunchData = useCallback(async (forceFullLogs = false) => {
    if (logsRefreshInFlightRef.current) return
    logsRefreshInFlightRef.current = true
    try {
      const status = await api('/api/mission/launch/status')
      if (!status.success) {
        setLogsResult(status)
        return
      }

      setLaunchStatus(status)
      const isRunning = status.state === 'running'
      if (!isRunning && !forceFullLogs) {
        setLogsResult(null)
        return
      }

      if (isRunning) {
        const currentOffset = logsOffsetRef.current
        const logData = await api(`/api/mission/launch/logs?offset=${currentOffset}`)
        if (logData.success) {
          const rawChunk = logData.logs || ''
          if (logData.reset) {
            setLogs(rawChunk)
          } else if (rawChunk) {
            setLogs(prev => `${prev}${rawChunk}`)
          }

          if (typeof logData.next_offset === 'number' && Number.isFinite(logData.next_offset) && logData.next_offset >= 0) {
            logsOffsetRef.current = logData.next_offset
          } else {
            const fallbackBytes = typeof TextEncoder !== 'undefined'
              ? new TextEncoder().encode(rawChunk).length
              : rawChunk.length
            logsOffsetRef.current = currentOffset + fallbackBytes
          }
          setLogsResult(null)
        } else {
          setLogsResult(logData)
        }
      } else {
        const logData = await api('/api/mission/launch/logs?lines=0')
        if (logData.success) {
          const full = logData.logs || ''
          setLogs(full)
          logsOffsetRef.current = typeof TextEncoder !== 'undefined'
            ? new TextEncoder().encode(full).length
            : full.length
          setLogsResult(null)
        } else {
          setLogsResult(logData)
        }
      }
    } finally {
      logsRefreshInFlightRef.current = false
    }
  }, [])

  const loadParams = useCallback(async () => {
    setParamsLoading(true)
    setParamsResult(null)
    const data = await api('/api/mission/launch-params')
    if (data.success) {
      setParamsText(data.content || '')
    } else {
      setParamsResult(data)
    }
    setParamsLoading(false)
  }, [])

  useEffect(() => {
    refreshLaunchData(true)
    loadParams()
    const interval = setInterval(() => refreshLaunchData(false), 500)
    return () => clearInterval(interval)
  }, [refreshLaunchData, loadParams])

  const runAction = async (name, url) => {
    setActionLoading(name)
    setActionResult(null)
    const data = await api(url, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    refreshLaunchData()
    onRefresh()
  }

  const saveParams = async () => {
    setParamsLoading(true)
    setParamsResult(null)
    const fd = new FormData()
    fd.append('content', paramsText)
    const data = await api('/api/mission/launch-params', { method: 'POST', body: fd })
    setParamsResult(data)
    setParamsLoading(false)
  }

  const updateField = (field, value) => {
    setParamsText(prev => setYamlFieldValue(prev, field.key, field.type, value))
  }

  return (
    <>
      <div className="grid mission-grid">
        <div className="card">
          <h2 className="card-title">Current Build On Pi</h2>
          <div className="info-box">
            <pre>{buildInfo?.info || 'No build metadata available'}</pre>
          </div>
          <div className="launch-state-row">
            <span className={`launch-pill ${launchStateClass(launchStatus)}`}>
              {launchStateLabel(launchStatus)}
            </span>
            {launchStatus?.pid && <span className="launch-pid">PID {launchStatus.pid}</span>}
          </div>
        </div>

        <div className="card">
          <h2 className="card-title">Mission Actions</h2>
          <div className="card-content">
            <button
              className="btn btn-mission-prepare"
              onClick={() => runAction('prepare', '/api/mission/prepare')}
              disabled={actionLoading !== ''}
            >
              {actionLoading === 'prepare' ? 'PREPARING...' : 'PREPARE MISSION'}
            </button>

            <button
              className="btn btn-mission-start"
              onClick={() => runAction('start', '/api/mission/start')}
              disabled={actionLoading !== ''}
            >
              {actionLoading === 'start' ? 'STARTING...' : 'START MISSION'}
            </button>

            <button
              className="btn btn-failsafe"
              onClick={() => runAction('failsafe', '/api/failsafe')}
              disabled={actionLoading !== ''}
            >
              {actionLoading === 'failsafe' ? 'TRIGGERING...' : 'FAILSAFE'}
            </button>
          </div>
          <Result data={actionResult} />
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Launch Params (uav/launch/launch_params.yaml)</h2>
        <div className="card-content">
          <div className="mini-tabs">
            <button className={`mini-tab ${paramsMode === 'form' ? 'mini-tab-active' : ''}`} onClick={() => setParamsMode('form')}>Form View</button>
            <button className={`mini-tab ${paramsMode === 'raw' ? 'mini-tab-active' : ''}`} onClick={() => setParamsMode('raw')}>Raw YAML</button>
          </div>

          {paramsMode === 'form' ? (
            <div className="params-form-grid">
              {LAUNCH_PARAM_FIELDS.map(field => {
                const value = getYamlFieldValue(paramsText, field.key, field.type)
                if (field.type === 'boolean') {
                  return (
                    <div key={field.key} className="param-field">
                      <label className="toggle-label">
                        <input
                          type="checkbox"
                          checked={Boolean(value)}
                          onChange={e => updateField(field, e.target.checked)}
                        />
                        <span>{field.label}</span>
                      </label>
                      <p className="param-help">{field.help}</p>
                    </div>
                  )
                }

                if (field.type === 'array3') {
                  const textValue = Array.isArray(value) ? value.join(', ') : '0, 0, 0'
                  return (
                    <div key={field.key} className="param-field">
                      <label>{field.label}</label>
                      <input
                        type="text"
                        value={textValue}
                        onChange={e => {
                          const parts = e.target.value.split(',').map(v => Number(v.trim()) || 0)
                          const normalized = [parts[0] ?? 0, parts[1] ?? 0, parts[2] ?? 0]
                          updateField(field, normalized)
                        }}
                        placeholder="0, 0, 0"
                      />
                      <p className="param-help">{field.help}</p>
                    </div>
                  )
                }

                return (
                  <div key={field.key} className="param-field">
                    <label>{field.label}</label>
                    <input
                      type="text"
                      value={value}
                      onChange={e => updateField(field, e.target.value)}
                    />
                    <p className="param-help">{field.help}</p>
                  </div>
                )
              })}
            </div>
          ) : (
            <textarea
              className="yaml-editor"
              value={paramsText}
              onChange={e => setParamsText(e.target.value)}
              spellCheck={false}
            />
          )}

          <div className="row-actions">
            <button className="btn btn-secondary" onClick={loadParams} disabled={paramsLoading}>
              {paramsLoading ? 'Loading...' : 'Reload From Pi'}
            </button>
            <button className="btn btn-primary" onClick={saveParams} disabled={paramsLoading}>
              {paramsLoading ? 'Saving...' : 'Save Params'}
            </button>
          </div>
          <p className="subtext left-note">Reload discards unsaved local edits and re-reads the file from the Pi.</p>
          <Result data={paramsResult} />
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Launch Output (ros2 launch uav main.launch.py)</h2>
        <div className="card-content">
          <button className="btn btn-secondary" onClick={() => refreshLaunchData(true)}>Refresh Logs Now</button>
          <p className="subtext left-note">Logs auto-refresh every 0.5 seconds while launch is running.</p>
          <pre className="terminal-output"><AnsiLog text={logs} /></pre>
          <Result data={logsResult} />
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Video & Debug Streams</h2>
        <div className="placeholder-grid">
          <div className="placeholder-box">Camera feed placeholder</div>
          <div className="placeholder-box">CV debug stream placeholder</div>
        </div>
      </div>
    </>
  )
}

function DeployPage({ sshCommand, wifiStatus, buildInfo, onRefresh }) {
  return (
    <>
      <ConnectionCard sshCommand={sshCommand} />

      <div className="grid">
        <WifiCard wifiStatus={wifiStatus} onRefresh={onRefresh} />
        <BuildCard buildInfo={buildInfo} onRefresh={onRefresh} />
      </div>
    </>
  )
}

function Result({ data }) {
  if (!data) return null
  return (
    <div className={`result ${data.success ? 'result-ok' : 'result-err'}`}>
      {data.success ? (data.output || 'Done') : (data.error || 'Failed')}
    </div>
  )
}

function App() {
  const [page, setPage] = useState('mission')

  const [connected, setConnected] = useState(false)
  const [wifiStatus, setWifiStatus] = useState(null)
  const [buildInfo, setBuildInfo] = useState(null)
  const [sshCommand, setSshCommand] = useState('')
  const [pollError, setPollError] = useState(null)

  const refreshAll = useCallback(async () => {
    const [conn, wifi, build, ssh] = await Promise.all([
      api('/api/connection/status'),
      api('/api/wifi/status'),
      api('/api/builds/current'),
      api('/api/connection/ssh-command'),
    ])

    setConnected(conn.connected ?? false)
    if (wifi.success) setWifiStatus(wifi)
    if (build.success) setBuildInfo(build)
    if (ssh.success) setSshCommand(ssh.command)

    const err = conn?.error || (!wifi.success ? wifi?.error : null) || null
    setPollError(err ? { success: false, error: err } : null)
  }, [])

  useEffect(() => {
    refreshAll()
    const interval = setInterval(refreshAll, 5000)
    return () => clearInterval(interval)
  }, [refreshAll])

  return (
    <div className="app">
      <SettingsPanel onRefresh={refreshAll} />

      <h1 className="title">PennAiR Auton Deploy</h1>
      <StatusBar connected={connected} wifiStatus={wifiStatus} buildInfo={buildInfo} />
      <Result data={pollError} />

      <div className="page-tabs">
        <button
          className={`tab-btn ${page === 'mission' ? 'tab-active' : ''}`}
          onClick={() => setPage('mission')}
        >
          Mission Control
        </button>
        <button
          className={`tab-btn ${page === 'deploy' ? 'tab-active' : ''}`}
          onClick={() => setPage('deploy')}
        >
          Deploy
        </button>
      </div>

      {page === 'mission' ? (
        <MissionControl buildInfo={buildInfo} onRefresh={refreshAll} />
      ) : (
        <DeployPage sshCommand={sshCommand} wifiStatus={wifiStatus} buildInfo={buildInfo} onRefresh={refreshAll} />
      )}
    </div>
  )
}

export default App
