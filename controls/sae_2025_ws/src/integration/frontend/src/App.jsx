import { useState, useEffect, useCallback } from 'react'
import './App.css'

async function api(url, opts) {
  try {
    const res = await fetch(url, opts)
    return await res.json()
  } catch (e) {
    return { success: false, error: e.message }
  }
}

// ── Status Bar ─────────────────────────────────────────────

function StatusBar({ connected, wifiStatus, buildInfo }) {
  return (
    <div className="status-bar">
      <div className="status-item">
        <span className={`status-dot ${connected ? 'dot-ok' : 'dot-err'}`} />
        <span>{connected ? 'Pi connected' : 'Pi unreachable'}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${wifiStatus?.current_wifi ? 'dot-ok' : 'dot-warn'}`} />
        <span>
          {wifiStatus?.is_hotspot
            ? 'Hotspot'
            : wifiStatus?.current_wifi || 'Unknown'}
        </span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${buildInfo?.installed ? 'dot-ok' : 'dot-warn'}`} />
        <span>{buildInfo?.installed ? 'Build active' : 'No build'}</span>
      </div>
    </div>
  )
}

// ── Connect + Failsafe ─────────────────────────────────────

function ConnectCard({ sshCommand }) {
  const [copied, setCopied] = useState(false)
  const [failsafeLoading, setFailsafeLoading] = useState(false)
  const [failsafeResult, setFailsafeResult] = useState(null)

  const copy = () => {
    navigator.clipboard.writeText(sshCommand)
    setCopied(true)
    setTimeout(() => setCopied(false), 2000)
  }

  const failsafe = async () => {
    setFailsafeLoading(true)
    setFailsafeResult(null)
    const data = await api('/api/failsafe', { method: 'POST' })
    setFailsafeResult(data)
    setFailsafeLoading(false)
  }

  return (
    <div className="card card-full">
      <div className="card-inner">
        <div className="card-left">
          <h2 className="card-title">SSH</h2>
          <div className="ssh-box" onClick={copy}>
            <code>{sshCommand || '...'}</code>
            <span className="copy-tag">{copied ? 'Copied' : 'Copy'}</span>
          </div>
        </div>
        <div className="card-right">
          <button className="btn btn-failsafe" onClick={failsafe} disabled={failsafeLoading}>
            {failsafeLoading ? 'Sending...' : 'FAILSAFE'}
          </button>
          <p className="subtext">
            Publishes <code>/failsafe_trigger</code>
          </p>
        </div>
      </div>
      <Result data={failsafeResult} />
    </div>
  )
}

// ── WiFi ───────────────────────────────────────────────────

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

// ── Build ──────────────────────────────────────────────────

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
      <h2 className="card-title">Build</h2>
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
            accept=".tar.gz,.tgz"
            onChange={e => setFile(e.target.files?.[0] || null)}
            id="build-file"
          />
          <label htmlFor="build-file" className="file-label">
            {file ? file.name : 'Choose .tar.gz'}
          </label>
        </div>

        <button className="btn btn-primary" onClick={upload} disabled={uploading || !file}>
          {uploading ? 'Uploading...' : 'Upload & deploy'}
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

// ── Settings ───────────────────────────────────────────────

function SettingsCard({ onRefresh }) {
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
    { key: 'ssh_pass', label: 'SSH password', placeholder: '123', type: 'password' },
    { key: 'ssh_key', label: 'SSH key path', placeholder: '~/.ssh/id_rsa' },
    { key: 'remote_dir', label: 'Remote directory', placeholder: '/home/penn/monorepo/controls/sae_2025_ws' },
    { key: 'github_repo', label: 'GitHub repo', placeholder: 'org/repo' },
    { key: 'hotspot_name', label: 'Hotspot connection name', placeholder: 'penn-desktop' },
  ]

  return (
    <div className="card card-full">
      <button className="btn btn-ghost" onClick={toggle}>
        {open ? 'Hide settings' : 'Settings'}
      </button>

      {open && cfg && (
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
      )}
    </div>
  )
}

// ── Shared ─────────────────────────────────────────────────

function Result({ data }) {
  if (!data) return null
  return (
    <div className={`result ${data.success ? 'result-ok' : 'result-err'}`}>
      {data.success ? (data.output || 'Done') : (data.error || 'Failed')}
    </div>
  )
}

// ── App ────────────────────────────────────────────────────

function App() {
  const [connected, setConnected] = useState(false)
  const [wifiStatus, setWifiStatus] = useState(null)
  const [buildInfo, setBuildInfo] = useState(null)
  const [sshCommand, setSshCommand] = useState('')

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
  }, [])

  useEffect(() => {
    refreshAll()
    const interval = setInterval(refreshAll, 5000)
    return () => clearInterval(interval)
  }, [refreshAll])

  return (
    <div className="app">
      <h1 className="title">SAE Deploy</h1>
      <StatusBar connected={connected} wifiStatus={wifiStatus} buildInfo={buildInfo} />

      <ConnectCard sshCommand={sshCommand} />

      <div className="grid">
        <WifiCard wifiStatus={wifiStatus} onRefresh={refreshAll} />
        <BuildCard buildInfo={buildInfo} onRefresh={refreshAll} />
      </div>

      <SettingsCard onRefresh={refreshAll} />
    </div>
  )
}

export default App
