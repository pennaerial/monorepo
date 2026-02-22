import { useState, useEffect, useCallback } from 'react'
import '@xterm/xterm/css/xterm.css'
import './App.css'
import { useMissionControl } from './hooks/useMissionControl'
import { api } from './services/api'

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

const LAUNCH_PARAM_FIELD_MAP = Object.fromEntries(
  LAUNCH_PARAM_FIELDS.map(field => [field.key, field])
)

const LAUNCH_PARAM_CORE_FIELDS = [
  'mission_name',
  'airframe',
  'custom_airframe_model',
  'camera_offsets',
].map(key => LAUNCH_PARAM_FIELD_MAP[key]).filter(Boolean)

const LAUNCH_PARAM_TOGGLE_FIELDS = [
  'uav_debug',
  'vision_debug',
  'run_mission',
  'use_camera',
  'save_vision',
  'servo_only',
  'sim',
].map(key => LAUNCH_PARAM_FIELD_MAP[key]).filter(Boolean)

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
    key: 'stop',
    url: '/api/mission/stop',
    className: 'btn btn-mission-stop',
    label: 'STOP MISSION',
    loadingLabel: 'STOPPING...',
  },
  {
    key: 'failsafe',
    url: '/api/failsafe',
    className: 'btn btn-failsafe',
    label: 'FAILSAFE',
    loadingLabel: 'TRIGGERING...',
  },
]

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
  const launchState = status?.launch_state || status?.state
  if (launchState === 'offline') return 'Pi Offline'
  if (launchState === 'running') return 'Launch Running'
  if (launchState === 'stopped') return 'Launch Stopped'
  if (launchState === 'not_prepared') return 'Not Prepared'
  return 'Launch Unavailable'
}

function launchStateClass(status) {
  const launchState = status?.launch_state || status?.state
  if (launchState === 'offline') return 'pill-offline'
  if (launchState === 'running') return 'pill-running'
  if (launchState === 'stopped') return 'pill-stopped'
  if (launchState === 'not_prepared') return 'pill-not-prepared'
  return 'pill-not-prepared'
}

function StatusBar({ connected, wifiStatus, buildInfo }) {
  const wifiText = connected
    ? (wifiStatus?.is_hotspot ? 'Hotspot' : wifiStatus?.current_wifi || 'No client WiFi')
    : 'Unavailable (Pi offline)'

  const buildText = connected
    ? (buildInfo?.installed ? 'Build active' : 'No build')
    : 'Unavailable (Pi offline)'

  return (
    <div className="status-bar">
      <div className="status-item">
        <span className={`status-dot ${connected ? 'dot-ok' : 'dot-err'}`} />
        <span>{connected ? 'Pi connected' : 'Pi unreachable'}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${connected && wifiStatus?.current_wifi ? 'dot-ok' : 'dot-warn'}`} />
        <span>{wifiText}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${connected && buildInfo?.installed ? 'dot-ok' : 'dot-warn'}`} />
        <span>{buildText}</span>
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

function WifiCard({ connected, wifiStatus, onRefresh }) {
  const [networks, setNetworks] = useState([])
  const [scanning, setScanning] = useState(false)
  const [selectedSsid, setSelectedSsid] = useState('')
  const [password, setPassword] = useState('')
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState(null)

  useEffect(() => {
    if (!connected) {
      setNetworks([])
      setSelectedSsid('')
      setResult(null)
    }
  }, [connected])

  const scan = async () => {
    if (!connected) return
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
    if (!connected || !selectedSsid) return
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
    if (!connected) return
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
        <button className="btn btn-secondary" onClick={scan} disabled={scanning || !connected}>
          {scanning ? 'Scanning...' : 'Scan networks'}
        </button>
        {!connected && (
          <p className="subtext left-note">Connect to the Pi WiFi to scan and manage networks.</p>
        )}

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

            <button className="btn btn-primary" onClick={connect} disabled={loading || !connected}>
              {loading ? 'Connecting...' : 'Connect both devices'}
            </button>
          </>
        )}

        {!wifiStatus?.is_hotspot && (
          <button className="btn btn-secondary" onClick={hotspot} disabled={loading || !connected}>
            {loading ? 'Switching...' : 'Restore hotspot'}
          </button>
        )}

        <Result data={result} />
      </div>
    </div>
  )
}

function BuildCard({ connected, buildInfo, onRefresh }) {
  const [file, setFile] = useState(null)
  const [uploading, setUploading] = useState(false)
  const [builds, setBuilds] = useState([])
  const [selectedTag, setSelectedTag] = useState('')
  const [loadingBuilds, setLoadingBuilds] = useState(false)
  const [downloading, setDownloading] = useState(false)
  const [result, setResult] = useState(null)

  useEffect(() => {
    if (!connected) {
      setResult(null)
      setBuilds([])
      setSelectedTag('')
    }
  }, [connected])

  const upload = async () => {
    if (!connected || !file) return
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
    if (!connected) return
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
    if (!connected || !selectedTag) return
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
    if (!connected) return
    setResult(null)
    const data = await api('/api/builds/rollback', { method: 'POST' })
    setResult(data)
    onRefresh()
  }

  return (
    <div className="card">
      <h2 className="card-title">Build Deploy</h2>
      <div className="card-content">
        {connected && buildInfo?.info && (
          <div className="info-box">
            <pre>{buildInfo.info}</pre>
          </div>
        )}
        {!connected && (
          <p className="subtext left-note">Connect to the Pi WiFi to view deployed build info and deploy updates.</p>
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

        <button className="btn btn-primary" onClick={upload} disabled={uploading || !file || !connected}>
          {uploading ? 'Uploading...' : 'Upload & replace install'}
        </button>

        <div className="divider" />

        <label>From GitHub</label>
        <button className="btn btn-secondary" onClick={listBuilds} disabled={loadingBuilds || !connected}>
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

            <button className="btn btn-primary" onClick={download} disabled={downloading || !connected}>
              {downloading ? 'Downloading...' : 'Download & deploy'}
            </button>
          </>
        )}

        {connected && buildInfo?.installed && (
          <>
            <div className="divider" />
            <button className="btn btn-secondary" onClick={rollback} disabled={!connected}>
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

function MissionControl({ connected, buildInfo, onRefresh }) {
  const [paramsMode, setParamsMode] = useState('form')
  const {
    terminalHostRef,
    missionState,
    streamConnected,
    logsResult,
    actionLoading,
    actionResult,
    paramsText,
    setParamsText,
    paramsLoading,
    paramsResult,
    missionNames,
    missionNamesLoading,
    missionNamesError,
    missionFileText,
    setMissionFileText,
    missionFileLoading,
    missionFileSaving,
    missionFileResult,
    setMissionFileResult,
    loadParams,
    loadMissionFile,
    saveMissionFile,
    saveParams,
    runAction,
    refreshLaunchData,
  } = useMissionControl({ connected, onRefresh })

  const updateField = (field, value) => {
    setParamsText(prev => setYamlFieldValue(prev, field.key, field.type, value))
  }

  const selectedMissionName = `${getYamlFieldValue(paramsText, 'mission_name', 'string') || ''}`.trim()

  useEffect(() => {
    if (!connected || !selectedMissionName) {
      setMissionFileText('')
      setMissionFileResult(null)
      return
    }
    loadMissionFile(selectedMissionName)
  }, [
    connected,
    loadMissionFile,
    selectedMissionName,
    setMissionFileResult,
    setMissionFileText,
  ])

  const renderCoreField = (field) => {
    const value = getYamlFieldValue(paramsText, field.key, field.type)

    if (field.key === 'mission_name') {
      const currentValue = `${value ?? ''}`
      const options = Array.isArray(missionNames) ? [...missionNames] : []
      if (currentValue && !options.includes(currentValue)) {
        options.unshift(currentValue)
      }

      return (
        <div key={field.key} className="param-field">
          <label>{field.label}</label>
          <select
            value={currentValue}
            onChange={e => updateField(field, e.target.value)}
            disabled={!connected || missionNamesLoading}
          >
            {!currentValue && (
              <option value="" disabled>
                {missionNamesLoading ? 'Loading mission files...' : 'Select mission file'}
              </option>
            )}
            {options.length === 0 && (
              <option value="" disabled>
                No mission YAML files found
              </option>
            )}
            {options.map(name => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </select>
          <p className="param-help">{field.help}</p>
          {connected && missionNamesError && (
            <p className="param-help param-help-warn">
              Mission file list unavailable from `src/uav/uav/missions`: {missionNamesError}
            </p>
          )}
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
            disabled={!connected}
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
          disabled={!connected}
        />
        <p className="param-help">{field.help}</p>
      </div>
    )
  }

  const renderToggleField = (field) => {
    const value = getYamlFieldValue(paramsText, field.key, field.type)
    return (
      <label key={field.key} className="toggle-card">
        <input
          type="checkbox"
          checked={Boolean(value)}
          onChange={e => updateField(field, e.target.checked)}
          disabled={!connected}
        />
        <div>
          <span className="toggle-card-title">{field.label}</span>
          <p className="param-help">{field.help}</p>
        </div>
      </label>
    )
  }

  return (
    <>
      <div className="grid mission-grid">
        <div className="card">
          <h2 className="card-title">Current Build On Pi</h2>
          <div className="info-box">
            <pre>
              {connected
                ? (buildInfo?.info || 'No build metadata available')
                : 'Pi is unreachable. Connect to the Pi WiFi to read build metadata.'}
            </pre>
          </div>
          <div className="launch-state-row">
            <span className={`launch-pill ${launchStateClass(missionState)}`}>
              {launchStateLabel(missionState)}
            </span>
            {connected && missionState?.pid && <span className="launch-pid">PID {missionState.pid}</span>}
            {connected && missionState?.launch_state === 'running' && (
              <span className={`launch-pill ${streamConnected ? 'pill-running' : 'pill-not-prepared'}`}>
                {streamConnected ? 'Live Stream Connected' : 'Connecting Live Stream'}
              </span>
            )}
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
                disabled={actionLoading !== '' || !connected}
              >
                {actionLoading === action.key ? action.loadingLabel : action.label}
              </button>
            ))}
            {!connected && (
              <p className="subtext left-note">Connect to the Pi WiFi to enable mission actions.</p>
            )}
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

          {!connected && (
            <p className="subtext left-note">Connect to the Pi WiFi to load and edit `launch_params.yaml`.</p>
          )}

          {paramsMode === 'form' ? (
            <div className="params-layout">
              <div className="params-section">
                <h3 className="params-section-title">Core Configuration</h3>
                <div className="params-stack">
                  {LAUNCH_PARAM_CORE_FIELDS.map(renderCoreField)}
                </div>
              </div>
              <div className="params-section">
                <h3 className="params-section-title">Mission Toggles</h3>
                <div className="toggle-grid">
                  {LAUNCH_PARAM_TOGGLE_FIELDS.map(renderToggleField)}
                </div>
              </div>
            </div>
          ) : (
            <textarea
              className="yaml-editor"
              value={paramsText}
              onChange={e => setParamsText(e.target.value)}
              spellCheck={false}
              disabled={!connected}
            />
          )}

          <div className="row-actions">
            <button className="btn btn-secondary" onClick={loadParams} disabled={paramsLoading || !connected}>
              {paramsLoading ? 'Loading...' : 'Reload From Pi'}
            </button>
            <button className="btn btn-primary" onClick={saveParams} disabled={paramsLoading || !connected}>
              {paramsLoading ? 'Saving...' : 'Save Params'}
            </button>
          </div>
          <p className="subtext left-note">Reload discards unsaved local edits and re-reads the file from the Pi.</p>
          {connected && <Result data={paramsResult} />}
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Launch Output (ros2 launch uav main.launch.py)</h2>
        <div className="card-content">
          <button className="btn btn-secondary" onClick={() => refreshLaunchData(true)} disabled={!connected}>Refresh Logs Now</button>
          <p className="subtext left-note">
            {connected
              ? 'Live SSH stream runs while launch is running. Refresh re-syncs full log history.'
              : 'Connect to the Pi WiFi to stream launch output.'}
          </p>
          <div ref={terminalHostRef} className="terminal-output terminal-host" />
          {connected && <Result data={logsResult} />}
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">
          Mission View (
          {selectedMissionName
            ? `uav/uav/missions/${selectedMissionName}.yaml`
            : 'uav/uav/missions/<mission>.yaml'}
          )
        </h2>
        <div className="card-content">
          {!connected && (
            <p className="subtext left-note">Connect to the Pi WiFi to view and edit mission YAML.</p>
          )}
          {connected && !selectedMissionName && (
            <p className="subtext left-note">Set `mission_name` in launch params to load a mission YAML file.</p>
          )}
          {connected && selectedMissionName && (
            <>
              <textarea
                className="yaml-editor mission-editor"
                value={missionFileText}
                onChange={e => setMissionFileText(e.target.value)}
                spellCheck={false}
                disabled={missionFileLoading || missionFileSaving}
              />
              <div className="row-actions">
                <button
                  className="btn btn-secondary"
                  onClick={() => loadMissionFile(selectedMissionName)}
                  disabled={missionFileLoading || missionFileSaving}
                >
                  {missionFileLoading ? 'Loading...' : 'Reload Mission YAML'}
                </button>
                <button
                  className="btn btn-primary"
                  onClick={() => saveMissionFile(selectedMissionName)}
                  disabled={missionFileLoading || missionFileSaving}
                >
                  {missionFileSaving ? 'Saving...' : 'Save Mission YAML'}
                </button>
              </div>
              <p className="subtext left-note">
                Edits are saved on the Pi inside `src/uav/uav/missions` for the selected mission name.
              </p>
              <Result data={missionFileResult} />
            </>
          )}
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

function DeployPage({ connected, sshCommand, wifiStatus, buildInfo, onRefresh }) {
  return (
    <>
      <ConnectionCard sshCommand={sshCommand} />

      <div className="grid">
        <WifiCard connected={connected} wifiStatus={wifiStatus} onRefresh={onRefresh} />
        <BuildCard connected={connected} buildInfo={buildInfo} onRefresh={onRefresh} />
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

const THEME_STORAGE_KEY = 'integration-theme'
const THEME_DARK = 'dark'
const THEME_LIGHT = 'light'

function readStoredTheme() {
  if (typeof window === 'undefined') return THEME_DARK
  let theme = THEME_DARK
  try {
    const stored = window.localStorage.getItem(THEME_STORAGE_KEY)
    if (stored === THEME_DARK || stored === THEME_LIGHT) {
      theme = stored
    }
  } catch {
    // Ignore storage access failures and keep default theme.
  }
  document.documentElement.setAttribute('data-theme', theme)
  return theme
}

function App() {
  const [page, setPage] = useState('mission')
  const [theme, setTheme] = useState(readStoredTheme)

  const [connected, setConnected] = useState(false)
  const [wifiStatus, setWifiStatus] = useState(null)
  const [buildInfo, setBuildInfo] = useState(null)
  const [sshCommand, setSshCommand] = useState('')
  const [pollError, setPollError] = useState(null)

  const refreshAll = useCallback(async () => {
    const conn = await api('/api/connection/status')
    const sshPromise = api('/api/connection/ssh-command')

    const isConnected = Boolean(conn?.connected)
    setConnected(isConnected)

    if (!isConnected) {
      const ssh = await sshPromise
      if (ssh.success) setSshCommand(ssh.command)
      setWifiStatus(null)
      setBuildInfo(null)
      setPollError(conn?.error ? { success: false, error: conn.error } : null)
      return
    }

    const [wifi, build, ssh] = await Promise.all([
      api('/api/wifi/status'),
      api('/api/builds/current'),
      sshPromise,
    ])

    setWifiStatus(wifi.success ? wifi : null)
    setBuildInfo(build.success ? build : null)
    if (ssh.success) setSshCommand(ssh.command)

    const err = (!wifi.success ? wifi?.error : null) || (!build.success ? build?.error : null) || null
    setPollError(err ? { success: false, error: err } : null)
  }, [])

  useEffect(() => {
    refreshAll()
    const interval = setInterval(refreshAll, 5000)
    return () => clearInterval(interval)
  }, [refreshAll])

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme)
    try {
      window.localStorage.setItem(THEME_STORAGE_KEY, theme)
    } catch {
      // Ignore storage access failures when persistence is unavailable.
    }
  }, [theme])

  const toggleTheme = () => {
    setTheme(prev => (prev === THEME_DARK ? THEME_LIGHT : THEME_DARK))
  }

  return (
    <div className="app">
      <SettingsPanel onRefresh={refreshAll} />

      <h1 className="title">PennAiR Auton Deploy</h1>
      <StatusBar connected={connected} wifiStatus={wifiStatus} buildInfo={buildInfo} />
      <Result data={pollError} />

      <div className="top-controls">
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
        <button className="theme-toggle-btn" type="button" onClick={toggleTheme}>
          {theme === THEME_DARK ? 'Light Mode' : 'Dark Mode'}
        </button>
      </div>

      {page === 'mission' ? (
        <MissionControl connected={connected} buildInfo={buildInfo} onRefresh={refreshAll} />
      ) : (
        <DeployPage connected={connected} sshCommand={sshCommand} wifiStatus={wifiStatus} buildInfo={buildInfo} onRefresh={refreshAll} />
      )}
    </div>
  )
}

export default App
