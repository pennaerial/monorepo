import { useState, useEffect, useCallback, useRef } from 'react'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'
import '@xterm/xterm/css/xterm.css'
import './App.css'

const SSH_AUTH_ERROR_RE = /(permission denied|authentication failed|auth fail|incorrect password|access denied|password was rejected|password authentication is required|no ssh password is set)/i
const SSH_PASSWORD_HINT = 'If your Pi requires password auth, open Settings and enter the SSH password.'
const MAX_TERMINAL_BUFFER_CHARS = 1_000_000
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

const MISSION_ACTIONS = [
  {
    key: 'prepare',
    url: '/api/mission/prepare',
    className: 'btn btn-mission-prepare',
    label: 'PREPARE MISSION',
    loadingLabel: 'PREPARING...',
  },
  {
    key: 'start',
    url: '/api/mission/start',
    className: 'btn btn-mission-start',
    label: 'START MISSION',
    loadingLabel: 'STARTING...',
  },
  {
    key: 'failsafe',
    url: '/api/failsafe',
    className: 'btn btn-failsafe',
    label: 'FAILSAFE',
    loadingLabel: 'TRIGGERING...',
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

function trimTerminalBuffer(text) {
  if (text.length <= MAX_TERMINAL_BUFFER_CHARS) return text
  return text.slice(text.length - MAX_TERMINAL_BUFFER_CHARS)
}

async function requestJson(url, opts) {
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
    setOpen(prev => {
      const next = !prev
      setResult(null)
      if (next) load()
      return next
    })
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
  const [logsResult, setLogsResult] = useState(null)
  const [streamConnected, setStreamConnected] = useState(false)
  const terminalHostRef = useRef(null)
  const terminalRef = useRef(null)
  const fitAddonRef = useRef(null)
  const terminalBufferRef = useRef('')
  const statusInFlightRef = useRef(false)
  const streamRef = useRef(null)
  const streamActiveRef = useRef(false)
  const streamReconnectTimerRef = useRef(null)
  const hasLoadedLogsRef = useRef(false)

  const [paramsMode, setParamsMode] = useState('form')
  const [paramsText, setParamsText] = useState('')
  const [paramsLoading, setParamsLoading] = useState(false)
  const [paramsResult, setParamsResult] = useState(null)

  const resetTerminal = useCallback((text = '') => {
    const normalized = trimTerminalBuffer(text)
    terminalBufferRef.current = normalized
    const term = terminalRef.current
    if (!term) return
    term.reset()
    if (normalized) term.write(normalized)
  }, [])

  const appendTerminal = useCallback((text) => {
    if (typeof text !== 'string' || text.length === 0) return
    const next = terminalBufferRef.current + text
    if (next.length <= MAX_TERMINAL_BUFFER_CHARS) {
      terminalBufferRef.current = next
      const term = terminalRef.current
      if (!term) return
      term.write(text)
      return
    }

    const trimmed = trimTerminalBuffer(next)
    terminalBufferRef.current = trimmed
    const term = terminalRef.current
    if (!term) return
    term.reset()
    term.write(trimmed)
  }, [])

  useEffect(() => {
    const host = terminalHostRef.current
    if (!host) return undefined

    const term = new Terminal({
      disableStdin: true,
      convertEol: true,
      cursorBlink: false,
      fontSize: 12,
      fontFamily: 'SF Mono, Fira Code, ui-monospace, monospace',
      scrollback: 200000,
      theme: {
        background: '#0c0c12',
        foreground: '#d6d6e2',
      },
    })
    const fitAddon = new FitAddon()
    term.loadAddon(fitAddon)
    term.open(host)
    fitAddon.fit()
    termRef.current = term
    fitAddonRef.current = fitAddon
    if (terminalBufferRef.current) {
      term.write(terminalBufferRef.current)
    }

    const onResize = () => fitAddonRef.current?.fit()
    window.addEventListener('resize', onResize)

    return () => {
      window.removeEventListener('resize', onResize)
      fitAddonRef.current = null
      termRef.current = null
      term.dispose()
    }
  }, [])

  const loadFullLogs = useCallback(async () => {
    const data = await api('/api/mission/launch/logs?lines=0')
    if (data.success) {
      resetTerminal(data.logs || '')
      hasLoadedLogsRef.current = true
      setLogsResult(null)
      return true
    }
    setLogsResult(data)
    return false
  }, [resetTerminal])

  const closeTerminalStream = useCallback(() => {
    if (streamReconnectTimerRef.current) {
      clearTimeout(streamReconnectTimerRef.current)
      streamReconnectTimerRef.current = null
    }
    const ws = streamRef.current
    if (!ws) return
    ws.onopen = null
    ws.onmessage = null
    ws.onerror = null
    ws.onclose = null
    try {
      ws.close()
    } catch {
      // Ignore close errors during cleanup.
    }
    streamRef.current = null
    setStreamConnected(false)
  }, [])

  const openTerminalStream = useCallback(() => {
    if (streamRef.current) return
    const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
    const ws = new WebSocket(`${proto}://${window.location.host}/ws/mission/terminal`)
    streamRef.current = ws

    ws.onopen = () => {
      setStreamConnected(true)
      setLogsResult(null)
    }

    ws.onmessage = event => {
      if (typeof event.data === 'string' && event.data) {
        appendTerminal(event.data)
      }
    }

    ws.onerror = () => {
      setLogsResult({
        success: false,
        error: 'Live SSH terminal stream encountered an error.',
      })
    }

    ws.onclose = () => {
      if (streamRef.current === ws) {
        streamRef.current = null
      }
      setStreamConnected(false)
      if (streamActiveRef.current) {
        if (streamReconnectTimerRef.current) {
          clearTimeout(streamReconnectTimerRef.current)
        }
        streamReconnectTimerRef.current = window.setTimeout(() => {
          streamReconnectTimerRef.current = null
          if (streamActiveRef.current) {
            openTerminalStream()
          }
        }, 1000)
      }
    }
  }, [appendTerminal])

  const refreshLaunchStatus = useCallback(async (forceFullLogs = false) => {
    if (statusInFlightRef.current) return
    statusInFlightRef.current = true
    try {
      const status = await api('/api/mission/launch/status')
      if (!status.success) {
        streamActiveRef.current = false
        closeTerminalStream()
        setLaunchStatus({ running: false, state: 'error' })
        setStreamConnected(false)
        setLogsResult(status)
        return
      }

      setLaunchStatus(status)
      const isRunning = status.state === 'running'
      if (isRunning) {
        if (forceFullLogs || !hasLoadedLogsRef.current) {
          await loadFullLogs()
        }
        streamActiveRef.current = true
        openTerminalStream()
        setLogsResult(null)
        return
      }

      const wasStreaming = streamActiveRef.current
      streamActiveRef.current = false
      closeTerminalStream()
      if (forceFullLogs || wasStreaming || !hasLoadedLogsRef.current) {
        await loadFullLogs()
      } else {
        setLogsResult(null)
      }
    } finally {
      statusInFlightRef.current = false
    }
  }, [closeTerminalStream, loadFullLogs, openTerminalStream])

  const refreshLaunchData = useCallback(async (forceFullLogs = false) => {
    await refreshLaunchStatus(forceFullLogs)
  }, [refreshLaunchStatus])

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
    loadFullLogs()
    refreshLaunchStatus(false)
    loadParams()
    const interval = setInterval(() => refreshLaunchStatus(false), 500)
    return () => {
      clearInterval(interval)
      streamActiveRef.current = false
      closeTerminalStream()
    }
  }, [closeTerminalStream, loadFullLogs, loadParams, refreshLaunchStatus])

  const runAction = async (name, url) => {
    setActionLoading(name)
    setActionResult(null)
    if (name === 'prepare') {
      resetTerminal('')
      hasLoadedLogsRef.current = false
    }
    const data = await api(url, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshLaunchData(true)
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
            <span className={`launch-pill ${streamConnected ? 'pill-running' : 'pill-not-prepared'}`}>
              {streamConnected ? 'Live Stream Connected' : 'Live Stream Offline'}
            </span>
          </div>
        </div>

        <div className="card">
          <h2 className="card-title">Mission Actions</h2>
          <div className="card-content">
            {MISSION_ACTIONS.map(action => (
              <button
                key={action.key}
                className={action.className}
                onClick={() => runAction(action.key, action.url)}
                disabled={actionLoading !== ''}
              >
                {actionLoading === action.key ? action.loadingLabel : action.label}
              </button>
            ))}
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
          <p className="subtext left-note">Live SSH stream runs while launch is running. Refresh re-syncs full log history.</p>
          <div ref={terminalHostRef} className="terminal-output terminal-host" />
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
