import { useState, useEffect, useCallback, useDeferredValue, useMemo, useRef } from 'react'
import '@xterm/xterm/css/xterm.css'
import './App.css'
import { useMissionControl } from './hooks/useMissionControl'
import {
  api,
  normalizeFleetActionResult,
  withDeviceFormData,
  withDeviceScope,
  withQueryParams,
} from './services/api'

const LAUNCH_PARAM_FIELDS = [
  {
    key: 'mission',
    label: 'Mission name',
    type: 'string',
    help: 'Mission profile identifier used by the target runtime.',
  },
  {
    key: 'mission_path',
    label: 'Mission path',
    type: 'string',
    help: 'Optional explicit mission YAML path on the target. Leave blank to use Mission name.',
  },
  {
    key: 'debug',
    label: 'Debug',
    type: 'boolean',
    help: 'Enable extra runtime debug output.',
  },
  {
    key: 'vision_debug',
    label: 'Vision debug',
    type: 'boolean',
    help: 'Enable extra vision pipeline debug outputs.',
  },
  {
    key: 'auto_launch',
    label: 'Auto start mission',
    type: 'boolean',
    help: 'If true, mission flow starts automatically after the runtime is ready.',
  },
  {
    key: 'px4_airframe_id',
    label: 'PX4 airframe ID',
    type: 'integer',
    help: 'Required for UAV hardware targets. Example: 4004 for standard_vtol.',
  },
  {
    key: 'payload_controller',
    label: 'Payload controller',
    type: 'string',
    help: 'Optional payload controller override. Leave blank to use the default controller.',
  },
  {
    key: 'save_vision_milliseconds',
    label: 'Save vision interval (ms)',
    type: 'integer',
    help: 'Milliseconds between saved vision frames. Set to 0 to disable image capture.',
  },
  {
    key: 'camera_mount_offsets',
    label: 'Camera offsets (x,y,z)',
    type: 'array3',
    help: 'Camera position offsets in meters (NED: x forward, y right, z down).',
  },
  {
    key: 'servo_only',
    label: 'Servo only',
    type: 'boolean',
    help: 'Run servo-only behavior without the rest of the mission flow.',
  },
  {
    key: 'camera_input_transport',
    label: 'Camera input transport',
    type: 'string',
    help: 'Preferred camera transport, usually raw or compressed.',
  },
  {
    key: 'camera_rotate_degrees',
    label: 'Camera rotate degrees',
    type: 'integer',
    help: 'Rotate incoming camera frames before vision processing.',
  },
  {
    key: 'camera_preprocess_hook',
    label: 'Camera preprocess hook',
    type: 'string',
    help: 'Optional Python hook for image preprocessing.',
  },
]

const LAUNCH_PARAM_FIELD_MAP = Object.fromEntries(
  LAUNCH_PARAM_FIELDS.map(field => [field.key, field])
)

const LAUNCH_PARAM_CORE_FIELDS = [
  'mission',
  'mission_path',
  'px4_airframe_id',
  'payload_controller',
  'save_vision_milliseconds',
  'camera_mount_offsets',
  'camera_input_transport',
  'camera_rotate_degrees',
  'camera_preprocess_hook',
].map(key => LAUNCH_PARAM_FIELD_MAP[key]).filter(Boolean)

const LAUNCH_PARAM_TOGGLE_FIELDS = [
  'debug',
  'vision_debug',
  'auto_launch',
  'servo_only',
].map(key => LAUNCH_PARAM_FIELD_MAP[key]).filter(Boolean)

const LAUNCH_PARAM_STRUCTURE_FIELDS = LAUNCH_PARAM_CORE_FIELDS.filter(
  field => field.key !== 'mission' && field.key !== 'mission_path'
)

const MISSION_ACTIONS = [
  {
    key: 'prepare',
    url: '/api/fleet/prepare',
    className: 'btn btn-mission-prepare',
    label: 'PREPARE MISSION',
    loadingLabel: 'PREPARING...',
  },
  {
    key: 'start',
    url: '/api/fleet/start',
    className: 'btn btn-mission-start',
    label: 'START MISSION',
    loadingLabel: 'STARTING...',
  },
  {
    key: 'stop',
    url: '/api/fleet/stop',
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

function getTargetId(target) {
  if (!target) return ''
  if (typeof target === 'string') return target.trim()
  return `${target.target_id ?? target.id ?? target.name ?? target.hostname ?? target.pi_host ?? ''}`.trim()
}

function getTargetLabel(target) {
  if (!target) return 'No target'
  if (typeof target === 'string') return target.trim() || 'Unnamed target'
  const id = getTargetId(target)
  const label = `${target.label ?? target.name ?? target.hostname ?? target.pi_host ?? id}`.trim() || 'Unnamed target'
  return id && label !== id ? `${label} (${id})` : label
}

function normalizeTargets(targets) {
  if (!Array.isArray(targets)) return []
  return targets
    .map(target => ({
      ...target,
      target_id: getTargetId(target),
    }))
    .filter(target => Boolean(target.target_id))
}

function uniqueTargets(targets) {
  const seen = new Set()
  return normalizeTargets(targets).filter(target => {
    if (seen.has(target.target_id)) return false
    seen.add(target.target_id)
    return true
  })
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
  if (type === 'integer') {
    const parsed = Number.parseInt(value, 10)
    return Number.isFinite(parsed) ? parsed : 0
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
  if (type === 'integer') {
    const parsed = Number.parseInt(`${value ?? 0}`, 10)
    return `${Number.isFinite(parsed) ? parsed : 0}`
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
  if (launchState === 'offline') return 'Target Offline'
  if (launchState === 'running') return 'Runtime Running'
  if (launchState === 'stopped') return 'Runtime Stopped'
  if (launchState === 'not_prepared') return 'Not Prepared'
  return 'Runtime Unavailable'
}

function launchStateClass(status) {
  const launchState = status?.launch_state || status?.state
  if (launchState === 'offline') return 'pill-offline'
  if (launchState === 'running') return 'pill-running'
  if (launchState === 'stopped') return 'pill-stopped'
  if (launchState === 'not_prepared') return 'pill-not-prepared'
  return 'pill-not-prepared'
}

function displayWorkspacePath(path, workspaceRoot) {
  if (!path) return ''
  if (workspaceRoot && path.startsWith(`${workspaceRoot}/`)) {
    return path.slice(workspaceRoot.length + 1)
  }
  return path
}

function indentBlock(text, spaces) {
  const pad = ' '.repeat(spaces)
  return text
    .split('\n')
    .map(line => (line.trim() ? `${pad}${line}` : line))
    .join('\n')
}

function dedentBlock(text, spaces) {
  const pattern = new RegExp(`^ {0,${spaces}}`)
  return text
    .split('\n')
    .map(line => line.replace(pattern, ''))
    .join('\n')
    .trim()
}

function parseInlineYamlValue(line) {
  const idx = line.indexOf(':')
  if (idx < 0) return ''
  return line.slice(idx + 1).trim()
}

function stripYamlComment(value) {
  const idx = value.indexOf(' #')
  return idx >= 0 ? value.slice(0, idx).trim() : value.trim()
}

function parseTransitionEntries(rawText) {
  return rawText
    .split('\n')
    .map(line => line.trim())
    .filter(line => line && !line.startsWith('#') && line.includes(':'))
    .map(line => {
      const idx = line.indexOf(':')
      const key = line.slice(0, idx).trim()
      const value = stripYamlComment(line.slice(idx + 1))
      return { key, value }
    })
    .filter(entry => entry.key)
}

function inferMissionTarget(classPath) {
  if (typeof classPath !== 'string') return ''
  if (classPath.includes('.payload.')) return 'payload'
  if (classPath.includes('.uav.')) return 'uav'
  return ''
}

function parseMissionDocument(text) {
  const rawText = `${text || ''}`
  const lines = rawText.split('\n')
  const modesLineIndex = lines.findIndex(line => /^\s*modes:\s*$/.test(line))

  if (modesLineIndex < 0) {
    return {
      rawText,
      modes: [],
      selectedTarget: '',
      warnings: ['Mission YAML does not contain a top-level modes mapping.'],
    }
  }

  const modeStarts = []
  for (let index = modesLineIndex + 1; index < lines.length; index += 1) {
    const line = lines[index]
    const match = line.match(/^  ([^:#\s][^:]*)\s*:\s*$/)
    if (match) {
      modeStarts.push({ name: match[1], line: index })
    }
  }

  if (modeStarts.length === 0) {
    return {
      rawText,
      modes: [],
      selectedTarget: '',
      warnings: ['Mission YAML has a modes block but no mode entries.'],
    }
  }

  const modes = modeStarts.map((modeStart, index) => {
    const nextLine = modeStarts[index + 1]?.line ?? lines.length
    const blockLines = lines.slice(modeStart.line, nextLine)

    let classPath = ''
    let paramsRaw = ''
    let transitionsRaw = ''
    let classLine = -1
    let paramsLine = -1
    let transitionsLine = -1

    for (let i = 0; i < blockLines.length; i += 1) {
      const line = blockLines[i]
      if (classLine < 0 && /^\s{4}class:\s*/.test(line)) {
        classLine = i
        classPath = stripYamlComment(parseInlineYamlValue(line))
        continue
      }
      if (paramsLine < 0 && /^\s{4}params:\s*/.test(line)) {
        paramsLine = i
        const inline = stripYamlComment(parseInlineYamlValue(line))
        const section = []
        if (inline) section.push(inline)
        for (let j = i + 1; j < blockLines.length; j += 1) {
          const nested = blockLines[j]
          if (/^\s{4}[A-Za-z0-9_.-]+\s*:\s*/.test(nested)) break
          section.push(nested)
        }
        paramsRaw = dedentBlock(section.join('\n'), 6)
        continue
      }
      if (transitionsLine < 0 && /^\s{4}transitions:\s*/.test(line)) {
        transitionsLine = i
        const inline = stripYamlComment(parseInlineYamlValue(line))
        const section = []
        if (inline) section.push(inline)
        for (let j = i + 1; j < blockLines.length; j += 1) {
          const nested = blockLines[j]
          if (/^\s{4}[A-Za-z0-9_.-]+\s*:\s*/.test(nested)) break
          section.push(nested)
        }
        transitionsRaw = dedentBlock(section.join('\n'), 6)
      }
    }

    const transitions = parseTransitionEntries(transitionsRaw)
    const target = inferMissionTarget(classPath)

    return {
      name: modeStart.name,
      classPath,
      paramsRaw,
      transitionsRaw,
      transitions,
      target,
      hasParams: paramsLine >= 0,
      hasTransitions: transitionsLine >= 0,
    }
  })

  const inferredTargets = [...new Set(modes.map(mode => mode.target).filter(Boolean))]

  return {
    rawText,
    modes,
    selectedTarget: inferredTargets.length === 1 ? inferredTargets[0] : '',
    warnings: inferredTargets.length > 1
      ? [`Mission modes mix targets: ${inferredTargets.join(', ')}`]
      : [],
  }
}

function renderMissionDocument(doc) {
  if (!doc?.modes?.length) return doc?.rawText || ''

  const lines = ['modes:']
  doc.modes.forEach((mode, index) => {
    lines.push(`  ${mode.name}:`)
    lines.push(`    class: ${mode.classPath || ''}`)

    if (mode.paramsRaw && `${mode.paramsRaw}`.trim()) {
      lines.push('    params:')
      lines.push(...indentBlock(mode.paramsRaw.trimEnd(), 6).split('\n'))
    } else {
      lines.push('    params: {}')
    }

    if (mode.transitions?.length) {
      lines.push('    transitions:')
      mode.transitions.forEach(entry => {
        lines.push(`      ${entry.key}: ${entry.value}`)
      })
    } else if (mode.hasTransitions) {
      lines.push('    transitions: {}')
    }

    if (index < doc.modes.length - 1) {
      lines.push('')
    }
  })

  return lines.join('\n')
}

function updateMissionDocumentMode(doc, modeName, updater) {
  const next = {
    ...doc,
    modes: doc.modes.map(mode => (mode.name === modeName ? updater({ ...mode }) : mode)),
  }
  next.rawText = renderMissionDocument(next)
  return next
}

function normalizeYamlFragment(content) {
  const text = `${content || ''}`
  const trimmed = text.trim()
  if (!trimmed || trimmed === '{}' || trimmed === 'null') return ''
  return text
}

function splitTopLevelYamlEntries(content) {
  const text = normalizeYamlFragment(content)
  if (!text) return []
  const lines = text.split('\n')
  const entries = []

  for (let index = 0; index < lines.length; index += 1) {
    const line = lines[index]
    const match = line.match(/^([A-Za-z0-9_.-]+)\s*:\s*(.*)$/)
    if (!match) continue
    const start = index
    let end = lines.length
    for (let next = index + 1; next < lines.length; next += 1) {
      if (/^[A-Za-z0-9_.-]+\s*:\s*(.*)$/.test(lines[next])) {
        end = next
        break
      }
    }
    entries.push({
      key: match[1],
      value: match[2],
      start,
      end,
      lines: lines.slice(start, end),
    })
    index = end - 1
  }

  return entries
}

function getYamlBlockValue(content, key) {
  const entry = splitTopLevelYamlEntries(content).find(item => item.key === key)
  if (!entry) return ''

  const inline = stripYamlComment(entry.value || '')
  if (inline) return inline
  if (entry.lines.length <= 1) return ''
  return dedentBlock(entry.lines.slice(1).join('\n'), 2).trimEnd()
}

function setYamlBlockValue(content, key, rawValue) {
  const lines = normalizeYamlFragment(content).split('\n').filter(Boolean)
  const entries = splitTopLevelYamlEntries(content)
  const existing = entries.find(item => item.key === key)
  const nextLines = [...lines]

  if (existing) {
    nextLines.splice(existing.start, existing.end - existing.start)
  }

  const trimmed = `${rawValue || ''}`.trim()
  if (trimmed) {
    const replacement = trimmed.includes('\n')
      ? [`${key}:`, ...indentBlock(trimmed, 2).split('\n')]
      : [`${key}: ${trimmed}`]
    const insertAt = existing ? existing.start : nextLines.length
    nextLines.splice(insertAt, 0, ...replacement)
  }

  return nextLines.join('\n')
}

function schemaFieldInputKind(field) {
  if (!field) return 'text'
  if (field.choices?.length) return 'select'
  if (field.schema_type === 'boolean' || field.schema_type === 'bool') return 'boolean'
  if (field.schema_type === 'integer' || field.schema_type === 'number' || field.schema_type === 'int' || field.schema_type === 'float') return 'number'
  if (field.schema_type === 'tuple') return 'tuple'
  if (field.schema_type === 'array' || field.schema_type === 'object' || field.schema_type === 'union' || field.schema_type === 'list' || field.schema_type === 'dict') {
    return 'block'
  }
  return 'text'
}

function effectiveSchemaFieldValue(paramsRaw, field) {
  const blockValue = getYamlBlockValue(paramsRaw, field.name)
  if (!blockValue) {
    return field.default ?? ''
  }

  const kind = schemaFieldInputKind(field)
  if (kind === 'boolean') {
    return `${blockValue}`.trim().toLowerCase() === 'true'
  }
  if (kind === 'number') {
    const parsed = Number(`${blockValue}`.trim())
    return Number.isFinite(parsed) ? parsed : (field.default ?? 0)
  }
  if (kind === 'tuple') {
    const stripped = `${blockValue}`.replace(/^\[/, '').replace(/\]$/, '')
    return stripped.split(',').map(part => Number(part.trim()) || 0)
  }
  return blockValue
}

function writeSchemaFieldValue(paramsRaw, field, nextValue) {
  const kind = schemaFieldInputKind(field)
  if (kind === 'boolean') {
    return setYamlBlockValue(paramsRaw, field.name, nextValue ? 'true' : 'false')
  }
  if (kind === 'number') {
    return setYamlBlockValue(paramsRaw, field.name, `${nextValue}`)
  }
  if (kind === 'tuple') {
    const arr = Array.isArray(nextValue) ? nextValue : [0, 0, 0]
    const normalized = [arr[0] ?? 0, arr[1] ?? 0, arr[2] ?? 0].map(v => Number(v) || 0)
    return setYamlBlockValue(paramsRaw, field.name, `[${normalized.join(', ')}]`)
  }
  return setYamlBlockValue(paramsRaw, field.name, `${nextValue || ''}`)
}

function normalizeModeRegistry(modeRegistry) {
  const flat = {}
  Object.values(modeRegistry || {}).forEach(entries => {
    ;(entries || []).forEach(entry => {
      flat[entry.class_path] = entry
      const parts = `${entry.class_path}`.split('.')
      if (parts.length > 1 && parts.at(-1) === parts.at(-2)) {
        flat[parts.slice(0, -1).join('.')] = entry
      }
    })
  })
  return flat
}

function defaultParamsRawForMode(metadata) {
  if (!metadata?.params?.length) return ''
  const lines = []
  metadata.params.forEach(field => {
    if (field.default_kind === 'missing') return
    if (field.default === null || field.default === undefined) return
    if (field.schema_type === 'tuple' && Array.isArray(field.default)) {
      lines.push(`${field.name}: [${field.default.join(', ')}]`)
      return
    }
    if (field.schema_type === 'boolean') {
      lines.push(`${field.name}: ${field.default ? 'true' : 'false'}`)
      return
    }
    if (field.schema_type === 'array' || field.schema_type === 'object') {
      return
    }
    lines.push(`${field.name}: ${field.default}`)
  })
  return lines.join('\n')
}

function uniqueModeName(doc, baseName = 'mode') {
  const names = new Set(doc.modes.map(mode => mode.name))
  if (!names.has(baseName)) return baseName
  let index = 1
  while (names.has(`${baseName}_${index}`)) {
    index += 1
  }
  return `${baseName}_${index}`
}

function WifiCard({ connected, wifiStatus, onRefresh, targetId, hostname = '', vehicleName = '' }) {
  const [networks, setNetworks] = useState([])
  const [scanning, setScanning] = useState(false)
  const [selectedSsid, setSelectedSsid] = useState('')
  const [password, setPassword] = useState('')
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState(null)

  useEffect(() => {
    setNetworks([])
    setSelectedSsid('')
    setResult(null)
  }, [targetId, hostname, vehicleName])

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
    const data = await api(withDeviceScope('/api/wifi/scan', { targetId, hostname, vehicleName }))
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
    const data = await api(withDeviceScope('/api/wifi/connect', { targetId, hostname, vehicleName }), {
      method: 'POST',
      body: withDeviceFormData(fd, { targetId, hostname, vehicleName }),
    })
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
    const data = await api(withDeviceScope('/api/wifi/hotspot', { targetId, hostname, vehicleName }), {
      method: 'POST',
      body: withDeviceFormData(new FormData(), { targetId, hostname, vehicleName }),
    })
    setResult(data)
    setLoading(false)
    onRefresh()
  }

  return (
    <div className="card">
      <h2 className="card-title">Target WiFi</h2>
      <div className="card-content">
        <button className="btn btn-secondary" onClick={scan} disabled={scanning || !connected}>
          {scanning ? 'Scanning...' : 'Scan networks'}
        </button>
        {!connected && (
          <p className="subtext left-note">Connect to the target WiFi to scan and manage networks.</p>
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

function MissionControl({
  connected,
  buildInfo,
  onRefresh,
  workspacePaths,
  buildSource,
  targetId,
  hostname = '',
  vehicleName = '',
  selectedTarget,
}) {
  const [missionViewMode, setMissionViewMode] = useState('graph')
  const [showAdvancedRaw, setShowAdvancedRaw] = useState(false)
  const [integerDrafts, setIntegerDrafts] = useState({})
  const selectedVehicle = useMemo(
    () => fleetVehicleByName(buildSource, selectedTarget?.vehicle_name || vehicleName || ''),
    [buildSource, selectedTarget?.vehicle_name, vehicleName]
  )
  const selectedMissionName = useMemo(
    () => missionNameFromVehicle(selectedVehicle),
    [selectedVehicle]
  )
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
    modeRegistry,
    schemaLoading,
    schemaError,
    missionDocumentSchema,
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
  } = useMissionControl({ connected, onRefresh, targetId, hostname, vehicleName })
  const parsedMission = useMemo(() => parseMissionDocument(missionFileText), [missionFileText])

  useEffect(() => {
    setIntegerDrafts({})
  }, [paramsText])

  useEffect(() => {
    if (!connected || !selectedMissionName) return
    setParamsText(prev => {
      const withMission = setYamlFieldValue(prev, 'mission', 'string', selectedMissionName)
      return setYamlFieldValue(withMission, 'mission_path', 'string', selectedVehicle?.missionPath || '')
    })
  }, [connected, selectedMissionName, selectedVehicle?.missionPath, setParamsText])

  const updateField = (field, value) => {
    setParamsText(prev => setYamlFieldValue(prev, field.key, field.type, value))
  }

  const commitIntegerField = (field, rawValue) => {
    const parsed = Number.parseInt(`${rawValue ?? ''}`.trim(), 10)
    updateField(field, Number.isFinite(parsed) ? parsed : 0)
    setIntegerDrafts(prev => {
      const next = { ...prev }
      delete next[field.key]
      return next
    })
  }

  const workspaceRoot = workspacePaths?.deploy_root || ''
  const launchParamsDisplayPath = displayWorkspacePath(workspacePaths?.overlay_file, workspaceRoot) || 'config/overlay.yaml'
  const missionsDirDisplayPath =
    displayWorkspacePath(workspacePaths?.missions_dir, workspaceRoot) ||
    'config/missions'
  const fleetDisplayPath =
    displayWorkspacePath(buildSource?.fleetFile, workspaceRoot) ||
    'config/fleet.yaml'

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

  const renderCommonOverrideField = (field) => {
    const value = getYamlFieldValue(paramsText, field.key, field.type)

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

    if (field.type === 'integer') {
      const draftValue = integerDrafts[field.key]
      const displayValue = draftValue ?? `${Number.isFinite(value) ? value : 0}`
      return (
        <div key={field.key} className="param-field">
          <label>{field.label}</label>
          <input
            type="text"
            inputMode="numeric"
            pattern="[0-9]*"
            value={displayValue}
            onChange={e => {
              const nextValue = e.target.value
              if (!/^\d*$/.test(nextValue)) return
              setIntegerDrafts(prev => ({ ...prev, [field.key]: nextValue }))
            }}
            onBlur={e => commitIntegerField(field, e.target.value)}
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
          <h2 className="card-title">Current Runtime</h2>
          <div className="info-box">
            <pre>
              {connected
                ? (buildInfo?.info || 'No build metadata available')
                : 'Target is unreachable. Connect to the target WiFi to read build metadata.'}
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
              <p className="subtext left-note">Connect to the target WiFi to enable mission actions.</p>
            )}
          </div>
          <Result data={actionResult} />
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Runtime Overrides ({launchParamsDisplayPath})</h2>
        <div className="card-content">
          {!connected && (
            <p className="subtext left-note">Connect to the target WiFi to load and edit runtime overrides.</p>
          )}
          {connected && (!selectedVehicle || !selectedMissionName) && (
            <p className="subtext left-note">Assign a controllable in Launch Prep before editing runtime overrides.</p>
          )}

          <div className="mission-context-summary">
            <div className="overview-stack">
              <div className="overview-row">
                <span>Fleet file</span>
                <strong>{fleetDisplayPath || 'No fleet file selected'}</strong>
              </div>
              <div className="overview-row">
                <span>Controllable</span>
                <strong>{selectedVehicle?.name || selectedTarget?.vehicle_name || 'Not assigned'}</strong>
              </div>
              <div className="overview-row">
                <span>Derived mission</span>
                <strong>{selectedMissionName || 'Not available'}</strong>
              </div>
              <div className="overview-row">
                <span>Kind</span>
                <strong>{selectedVehicle?.kind || parsedMission.selectedTarget || 'unknown'}</strong>
              </div>
            </div>
            <p className="subtext left-note">
              The fleet entry defines the base mission. These fields only edit Pi-specific runtime overrides on top of that derived context.
            </p>
          </div>

          <div className="params-layout">
            <div className="params-section">
              <h3 className="params-section-title">Common Overrides</h3>
              <div className="params-stack">
                {LAUNCH_PARAM_STRUCTURE_FIELDS.map(renderCommonOverrideField)}
              </div>
            </div>
            <div className="params-section">
              <h3 className="params-section-title">Mission Toggles</h3>
              <div className="toggle-grid">
                {LAUNCH_PARAM_TOGGLE_FIELDS.map(renderToggleField)}
              </div>
            </div>
          </div>

          <details className="advanced-raw-editor" open={showAdvancedRaw}>
            <summary onClick={e => {
              e.preventDefault()
              setShowAdvancedRaw(value => !value)
            }}>
              {showAdvancedRaw ? 'Hide raw YAML overrides' : 'Show raw YAML overrides'}
            </summary>
            <textarea
              className="yaml-editor"
              value={paramsText}
              onChange={e => setParamsText(e.target.value)}
              spellCheck={false}
              disabled={!connected}
            />
            <p className="subtext left-note">
              Use raw YAML only for extra overrides that are not exposed in the structured fields above.
            </p>
          </details>

          <div className="row-actions">
            <button className="btn btn-secondary" onClick={loadParams} disabled={paramsLoading || !connected}>
              {paramsLoading ? 'Loading...' : 'Reload From Target'}
            </button>
            <button className="btn btn-primary" onClick={saveParams} disabled={paramsLoading || !connected}>
              {paramsLoading ? 'Saving...' : 'Save Overrides'}
            </button>
          </div>
          <p className="subtext left-note">Reload discards unsaved local edits and re-reads the selected target file from the Pi.</p>
          {connected && <Result data={paramsResult} />}
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Runtime Output</h2>
        <div className="card-content">
          <button className="btn btn-secondary" onClick={() => refreshLaunchData(true)} disabled={!connected}>Refresh Logs Now</button>
          <p className="subtext left-note">
            {connected
              ? 'Live SSH stream runs while runtime is running. Refresh re-syncs full log history.'
              : 'Connect to the target WiFi to stream runtime output.'}
          </p>
          <div ref={terminalHostRef} className="terminal-output terminal-host" />
          {connected && <Result data={logsResult} />}
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">
          Mission YAML (
          {selectedMissionName
            ? `${missionsDirDisplayPath}/${selectedMissionName}.yaml`
            : `${missionsDirDisplayPath}/<mission>.yaml`}
          )
        </h2>
        <div className="card-content">
          {!connected && (
            <p className="subtext left-note">Connect to the target WiFi to view and edit mission YAML.</p>
          )}
          {connected && !selectedMissionName && (
            <p className="subtext left-note">Assign a controllable in Launch Prep to load its mission YAML file.</p>
          )}
          {connected && selectedMissionName && (
            <>
              <div className="mini-tabs">
                <button className={`mini-tab ${missionViewMode === 'graph' ? 'mini-tab-active' : ''}`} onClick={() => setMissionViewMode('graph')}>
                  Graph View
                </button>
                <button className={`mini-tab ${missionViewMode === 'raw' ? 'mini-tab-active' : ''}`} onClick={() => setMissionViewMode('raw')}>
                  Raw YAML
                </button>
              </div>

              {missionViewMode === 'graph' ? (
                <MissionGraphEditor
                  connected={connected}
                  setMissionFileText={setMissionFileText}
                  parsedMission={parsedMission}
                  selectedTarget={selectedTarget}
                  buildSource={buildSource}
                  missionsDirDisplayPath={missionsDirDisplayPath}
                  selectedMissionName={selectedMissionName}
                  busy={missionFileLoading || missionFileSaving}
                  modeRegistry={modeRegistry}
                schemaLoading={schemaLoading}
                schemaError={schemaError}
                missionDocumentSchema={missionDocumentSchema}
              />
              ) : (
                <textarea
                  className="yaml-editor mission-editor"
                  value={missionFileText}
                  onChange={e => setMissionFileText(e.target.value)}
                  spellCheck={false}
                  disabled={missionFileLoading || missionFileSaving}
                />
              )}

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
                Mission graph edits stay in sync with the raw YAML text. Fleet source: `{fleetDisplayPath}`.
              </p>
              <Result data={missionFileResult} />
            </>
          )}
        </div>
      </div>

    </>
  )
}

function MissionGraphEditor({
  connected,
  setMissionFileText,
  parsedMission,
  selectedTarget,
  buildSource,
  missionsDirDisplayPath,
  selectedMissionName,
  busy,
  modeRegistry,
  schemaLoading,
  schemaError,
  missionDocumentSchema,
}) {
  const [selectedModeName, setSelectedModeName] = useState('')
  const registryByClass = useMemo(() => normalizeModeRegistry(modeRegistry), [modeRegistry])
  const selectedVehicle = useMemo(
    () => fleetVehicleByName(buildSource, selectedTarget?.vehicle_name || ''),
    [buildSource, selectedTarget?.vehicle_name]
  )

  useEffect(() => {
    if (!parsedMission.modes.length) {
      setSelectedModeName('')
      return
    }
    if (!parsedMission.modes.some(mode => mode.name === selectedModeName)) {
      setSelectedModeName(parsedMission.modes[0].name)
    }
  }, [parsedMission.modes, selectedModeName])

  const selectedMode = parsedMission.modes.find(mode => mode.name === selectedModeName) || null
  const selectedModeMetadata = selectedMode ? registryByClass[selectedMode.classPath] || null : null
  const availableModes = useMemo(() => {
    const target = selectedMode?.target || parsedMission.selectedTarget
    if (target && Array.isArray(modeRegistry?.[target])) {
      return modeRegistry[target]
    }
    return Object.values(modeRegistry || {}).flat()
  }, [modeRegistry, parsedMission.selectedTarget, selectedMode?.target])
  const fleetDisplayPath = buildSource?.fleetFile || ''

  const applyMissionUpdate = useCallback((updater) => {
    setMissionFileText(prev => {
      const current = parseMissionDocument(prev)
      const next = updater(current)
      return renderMissionDocument(next)
    })
  }, [setMissionFileText])

  const updateSelectedMode = useCallback((updater) => {
    if (!selectedModeName) return
    applyMissionUpdate(doc => updateMissionDocumentMode(doc, selectedModeName, updater))
  }, [applyMissionUpdate, selectedModeName])

  const addMode = () => {
    applyMissionUpdate(doc => {
      const fallbackMode = availableModes[0] || null
      const nextName = uniqueModeName(doc)
      const nextMode = {
        name: nextName,
        classPath: fallbackMode?.class_path || '',
        paramsRaw: defaultParamsRawForMode(fallbackMode),
        transitionsRaw: '',
        transitions: [],
        target: fallbackMode?.mission_target || parsedMission.selectedTarget || '',
        hasParams: true,
        hasTransitions: true,
      }
      doc.modes = [...doc.modes, nextMode]
      return doc
    })
    setSelectedModeName('')
  }

  const removeSelectedMode = () => {
    if (!selectedMode) return
    applyMissionUpdate(doc => {
      doc.modes = doc.modes.filter(mode => mode.name !== selectedMode.name)
      doc.modes = doc.modes.map(mode => ({
        ...mode,
        transitions: mode.transitions.filter(entry => entry.value !== selectedMode.name),
      }))
      return doc
    })
  }

  const updateTransitionRow = (index, keyOrValue, nextValue) => {
    updateSelectedMode(mode => {
      const nextTransitions = [...mode.transitions]
      const current = nextTransitions[index] || { key: '', value: '' }
      nextTransitions[index] = { ...current, [keyOrValue]: nextValue }
      mode.transitions = nextTransitions.filter(entry => entry.key || entry.value)
      return mode
    })
  }

  const addTransitionRow = () => {
    updateSelectedMode(mode => {
      mode.transitions = [...mode.transitions, { key: '', value: '' }]
      return mode
    })
  }

  const removeTransitionRow = (index) => {
    updateSelectedMode(mode => {
      mode.transitions = mode.transitions.filter((_, rowIndex) => rowIndex !== index)
      return mode
    })
  }

  const renderParamEditor = (field) => {
    const value = effectiveSchemaFieldValue(selectedMode?.paramsRaw, field)
    const kind = schemaFieldInputKind(field)
    const onChange = (nextValue) => {
      updateSelectedMode(mode => {
        mode.paramsRaw = writeSchemaFieldValue(mode.paramsRaw, field, nextValue)
        mode.hasParams = true
        return mode
      })
    }

    if (kind === 'select') {
      return (
        <div key={field.name} className="param-field">
          <label>{field.name}</label>
          <select
            value={`${value ?? ''}`}
            onChange={e => onChange(e.target.value)}
            disabled={!connected || busy}
          >
            <option value="">Use constructor default</option>
            {(field.choices || []).map(choice => (
              <option key={`${field.name}-${choice}`} value={`${choice}`}>
                {`${choice}`}
              </option>
            ))}
          </select>
          <p className="param-help">{field.annotation}</p>
        </div>
      )
    }

    if (kind === 'boolean') {
      return (
        <label key={field.name} className="toggle-card">
          <input
            type="checkbox"
            checked={Boolean(value)}
            onChange={e => onChange(e.target.checked)}
            disabled={!connected || busy}
          />
          <div>
            <span className="toggle-card-title">{field.name}</span>
            <p className="param-help">{field.annotation}</p>
          </div>
        </label>
      )
    }

    if (kind === 'number') {
      return (
        <div key={field.name} className="param-field">
          <label>{field.name}</label>
          <input
            type="text"
            inputMode="decimal"
            value={`${value ?? ''}`}
            onChange={e => onChange(e.target.value)}
            disabled={!connected || busy}
          />
          <p className="param-help">{field.annotation}</p>
        </div>
      )
    }

    if (kind === 'tuple') {
      const display = Array.isArray(value) ? value.join(', ') : `${value || ''}`
      return (
        <div key={field.name} className="param-field">
          <label>{field.name}</label>
          <input
            type="text"
            value={display}
            onChange={e => {
              const parts = e.target.value.split(',').map(part => Number(part.trim()) || 0)
              onChange(parts)
            }}
            disabled={!connected || busy}
          />
          <p className="param-help">{field.annotation}</p>
        </div>
      )
    }

    if (kind === 'block') {
      return (
        <div key={field.name} className="param-field">
          <label>{field.name}</label>
          <textarea
            className="yaml-editor mission-mode-editor"
            value={`${value || ''}`}
            onChange={e => onChange(e.target.value)}
            spellCheck={false}
            disabled={!connected || busy}
          />
          <p className="param-help">{field.annotation}</p>
        </div>
      )
    }

    return (
      <div key={field.name} className="param-field">
        <label>{field.name}</label>
        <input
          type="text"
          value={`${value ?? ''}`}
          onChange={e => onChange(e.target.value)}
          disabled={!connected || busy}
        />
        <p className="param-help">{field.annotation}</p>
      </div>
    )
  }

  return (
    <div className="mission-schema-editor">
      <div className="mission-schema-summary">
        <div>
          <div className="mission-schema-kicker">Backend metadata</div>
          <div className="mission-schema-line">
            Mission source: `{selectedMissionName ? `${missionsDirDisplayPath}/${selectedMissionName}.yaml` : `${missionsDirDisplayPath}/<mission>.yaml`}`
          </div>
          <div className="mission-schema-line">
            Fleet source: `{fleetDisplayPath || 'No fleet file selected'}`
          </div>
          <div className="mission-schema-line">
            Controllable: `{selectedVehicle?.name || selectedTarget?.vehicle_name || 'Not assigned'}`
          </div>
        </div>
        <div className="mission-schema-badges">
          <span className={`launch-pill ${parsedMission.selectedTarget === 'payload' ? 'pill-running' : 'pill-not-prepared'}`}>
            {parsedMission.selectedTarget || 'mixed/unknown target'}
          </span>
          <span className="launch-pill">Modes: {parsedMission.modes.length}</span>
          <span className="launch-pill">Edges: {parsedMission.modes.reduce((acc, mode) => acc + mode.transitions.length, 0)}</span>
          <span className="launch-pill">{missionDocumentSchema ? 'Schema loaded' : 'Schema unavailable'}</span>
        </div>
      </div>

      {schemaLoading && (
        <div className="result result-ok">
          Loading mission and mode schema metadata...
        </div>
      )}

      {schemaError && (
        <div className="result result-err">
          {schemaError}
        </div>
      )}

      {parsedMission.warnings.length > 0 && (
        <div className="result result-err">
          {parsedMission.warnings.join('\n')}
        </div>
      )}

      <div className="mission-graph-grid">
        <div className="mission-node-list">
          <div className="mission-edge-header">
            <label>Mode Graph</label>
            <button className="btn btn-secondary btn-inline" type="button" onClick={addMode} disabled={!connected || busy}>
              Add mode
            </button>
          </div>
          {parsedMission.modes.map(mode => (
            <button
              key={mode.name}
              type="button"
              className={`mission-node ${selectedModeName === mode.name ? 'mission-node-active' : ''}`}
              onClick={() => setSelectedModeName(mode.name)}
            >
              <span className="mission-node-name">{mode.name}</span>
              <span className="mission-node-class">{mode.classPath}</span>
              <span className="mission-node-meta">
                {mode.target || 'unknown'} · {mode.transitions.length} edge{mode.transitions.length === 1 ? '' : 's'}
              </span>
            </button>
          ))}
        </div>

        <div className="mission-node-editor">
          {!selectedMode ? (
            <p className="subtext left-note">Select a mode to edit its class, params, and outgoing transitions.</p>
          ) : (
            <>
              <div className="mission-node-editor-head">
                <div>
                  <div className="mission-schema-kicker">Selected mode</div>
                  <h3>{selectedMode.name}</h3>
                </div>
                <span className="launch-pill">{selectedMode.target || 'unknown target'}</span>
              </div>

              <label>Mode name</label>
              <input
                type="text"
                value={selectedMode.name}
                onChange={e => {
                  const nextName = e.target.value
                  if (!nextName) return
                  applyMissionUpdate(doc => {
                    doc.modes = doc.modes.map(mode => {
                      if (mode.name === selectedMode.name) {
                        return { ...mode, name: nextName }
                      }
                      return {
                        ...mode,
                        transitions: mode.transitions.map(entry => (
                          entry.value === selectedMode.name ? { ...entry, value: nextName } : entry
                        )),
                      }
                    })
                    return doc
                  })
                  setSelectedModeName(nextName)
                }}
                disabled={!connected || busy}
              />

              <label>Class path</label>
              <select
                value={selectedMode.classPath}
                onChange={e => updateSelectedMode(mode => {
                  const metadata = registryByClass[e.target.value] || null
                  mode.classPath = e.target.value
                  mode.target = metadata?.mission_target || inferMissionTarget(e.target.value)
                  mode.paramsRaw = defaultParamsRawForMode(metadata)
                  mode.transitions = []
                  mode.hasParams = true
                  mode.hasTransitions = true
                  return mode
                })}
                disabled={!connected || busy || availableModes.length === 0}
              >
                {availableModes.map(mode => (
                  <option key={mode.class_path} value={mode.class_path}>
                    {mode.class_path}
                  </option>
                ))}
                {!availableModes.length && (
                  <option value={selectedMode.classPath}>{selectedMode.classPath || 'No registered modes'}</option>
                )}
              </select>

              {selectedModeMetadata?.params?.length ? (
                <div className="params-layout">
                  <div className="params-section">
                    <h3 className="params-section-title">Mode Parameters</h3>
                    <div className="params-stack">
                      {selectedModeMetadata.params.map(renderParamEditor)}
                    </div>
                  </div>
                </div>
              ) : (
                <>
                  <label>Params YAML</label>
                  <textarea
                    className="yaml-editor mission-mode-editor"
                    value={selectedMode.paramsRaw}
                    onChange={e => updateSelectedMode(mode => {
                      mode.paramsRaw = e.target.value
                      return mode
                    })}
                    spellCheck={false}
                    disabled={!connected || busy}
                  />
                </>
              )}

              <div className="mission-edge-header">
                <label>Transitions</label>
                <button className="btn btn-secondary btn-inline" type="button" onClick={addTransitionRow} disabled={!connected || busy}>
                  Add edge
                </button>
                <button className="btn btn-secondary btn-inline" type="button" onClick={removeSelectedMode} disabled={!connected || busy || parsedMission.modes.length <= 1}>
                  Remove mode
                </button>
              </div>

              <div className="mission-edge-list">
                {(selectedMode.transitions.length ? selectedMode.transitions : [{ key: '', value: '' }]).map((entry, index) => (
                  <div key={`${selectedMode.name}-${index}`} className="mission-edge-row">
                    {selectedModeMetadata?.transition_labels?.length ? (
                      <select
                        value={entry.key}
                        onChange={e => updateTransitionRow(index, 'key', e.target.value)}
                        disabled={!connected || busy}
                      >
                        <option value="">Select transition</option>
                        {selectedModeMetadata.transition_labels.map(label => (
                          <option key={`${selectedMode.name}-${label}`} value={label}>
                            {label}
                          </option>
                        ))}
                      </select>
                    ) : (
                      <input
                        type="text"
                        value={entry.key}
                        onChange={e => updateTransitionRow(index, 'key', e.target.value)}
                        placeholder="state"
                        disabled={!connected || busy}
                      />
                    )}
                    <span className="mission-edge-arrow">→</span>
                    <select
                      value={entry.value}
                      onChange={e => updateTransitionRow(index, 'value', e.target.value)}
                      disabled={!connected || busy}
                    >
                      <option value="">Select next mode</option>
                      {parsedMission.modes.map(mode => (
                        <option key={mode.name} value={mode.name}>
                          {mode.name}
                        </option>
                      ))}
                    </select>
                    <button
                      className="btn btn-secondary btn-inline"
                      type="button"
                      onClick={() => removeTransitionRow(index)}
                      disabled={!connected || busy}
                    >
                      Remove
                    </button>
                  </div>
                ))}
              </div>
              <p className="param-help">
                Edit the mode graph directly here. Schema-backed fields come from the UAV package. For unsupported structures or exact text control, switch to the raw editor.
              </p>
            </>
          )}
        </div>
      </div>
    </div>
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

function normalizeHostname(hostname) {
  return `${hostname || ''}`.trim().replace(/\.$/, '').replace(/\.local$/i, '').toLowerCase()
}

const DISCOVERY_STALE_WINDOW_MS = 5 * 60 * 1000

function timestampToMs(value) {
  const raw = `${value || ''}`.trim()
  if (!raw) return 0
  const parsed = Date.parse(raw)
  return Number.isFinite(parsed) ? parsed : 0
}

function formatRelativeTimestamp(value) {
  const parsed = timestampToMs(value)
  if (!parsed) return ''
  const deltaSeconds = Math.max(0, Math.round((Date.now() - parsed) / 1000))
  if (deltaSeconds < 15) return 'seen just now'
  if (deltaSeconds < 60) return `seen ${deltaSeconds}s ago`
  const deltaMinutes = Math.round(deltaSeconds / 60)
  if (deltaMinutes < 60) return `seen ${deltaMinutes}m ago`
  const deltaHours = Math.round(deltaMinutes / 60)
  if (deltaHours < 24) return `seen ${deltaHours}h ago`
  return `seen ${Math.round(deltaHours / 24)}d ago`
}

function discoveryDeviceKey(device) {
  if (!device || typeof device !== 'object') return ''
  return normalizeHostname(device.hostname) || `${device.hardware_id || device.id || ''}`.trim()
}

function mergeDiscoveryDevices(previousDevices, nextDevices) {
  const nextMap = new Map()
  ;(nextDevices || []).forEach(device => {
    const key = discoveryDeviceKey(device)
    if (!key) return
    nextMap.set(key, device)
  })

  ;(previousDevices || []).forEach(device => {
    const key = discoveryDeviceKey(device)
    if (!key || nextMap.has(key)) return
    const lastSeen = timestampToMs(device.last_seen_at)
    if (!lastSeen || Date.now() - lastSeen > DISCOVERY_STALE_WINDOW_MS) return
    nextMap.set(key, {
      ...device,
      discovery_stale: true,
    })
  })

  return Array.from(nextMap.values()).sort((left, right) => {
    const leftSeen = timestampToMs(left.last_seen_at)
    const rightSeen = timestampToMs(right.last_seen_at)
    if (leftSeen !== rightSeen) return rightSeen - leftSeen
    return discoveryDeviceKey(left).localeCompare(discoveryDeviceKey(right))
  })
}

function suggestedTargetId(hostname) {
  const normalized = normalizeHostname(hostname).replace(/[^a-z0-9._-]+/g, '-')
  return normalized || 'target'
}

function defaultOverlayYaml() {
  return 'mission: hover\nauto_launch: false\npx4_airframe_id: 4004\n'
}

function normalizeBuildRecord(build) {
  if (!build || typeof build !== 'object') return null
  const timestamp = build.timestamp || build.published_at || build.updated_at || build.created_at || build.run_started_at || build.date || ''
  return {
    source: build.source || build.source_type || 'release',
    tag: build.tag || build.ref || '',
    sha: build.sha || build.commit || '',
    commitSha: build.commit_sha || build.commitSha || build.head_sha || '',
    commitSubject: build.commit_subject || build.commitSubject || '',
    name: build.name || build.artifact_name || build.bundle_name || build.tag || 'Build',
    date: build.date || timestamp || '',
    timestamp,
    download_url: build.download_url || '',
    size_mb: build.size_mb ?? null,
    run_id: build.run_id || '',
    artifact_id: build.artifact_id || '',
    artifact_name: build.artifact_name || '',
    artifactName: build.artifact_name || build.artifactName || '',
    branch: build.branch || '',
    workflowName: build.workflow_name || build.workflowName || build.workflow || '',
    jobName: build.job_name || build.jobName || '',
    event: build.workflow_event || build.event || '',
    conclusion: build.workflow_conclusion || build.conclusion || '',
    runNumber: build.run_number || build.runNumber || '',
  }
}

function buildSelectionKey(build) {
  if (!build) return ''
  return [
    build.source || 'release',
    build.tag || '',
    build.artifact_id || '',
    build.run_id || '',
  ].join('::')
}

function mergeBuildLists(previous, incoming) {
  const seen = new Set()
  const next = []
  ;[...(previous || []), ...(incoming || [])].forEach(build => {
    const key = buildSelectionKey(build)
    if (!key || seen.has(key)) return
    seen.add(key)
    next.push(build)
  })
  return next
}

function formatBuildType(build) {
  if (!build) return 'Build'
  return build.source === 'actions' ? 'Artifact' : 'Release'
}

function buildListPaginationFromResponse(data, current = {}) {
  const pageInfo = data?.artifacts_page_info || data?.artifact_page_info || data?.artifacts_pagination || data?.artifact_pagination || data?.pagination?.artifacts || data?.pagination || null
  const currentPage = Number.isFinite(Number(pageInfo?.page ?? data?.artifact_page ?? data?.artifacts_page ?? current.page))
    ? Number(pageInfo?.page ?? data?.artifact_page ?? data?.artifacts_page ?? current.page)
    : current.page || 0
  const currentLimit = Number.isFinite(Number(pageInfo?.limit ?? pageInfo?.page_size ?? data?.artifact_page_size ?? data?.artifacts_page_size ?? data?.page_size ?? current.limit))
    ? Number(pageInfo?.limit ?? pageInfo?.page_size ?? data?.artifact_page_size ?? data?.artifacts_page_size ?? data?.page_size ?? current.limit)
    : current.limit || 0
  const hasMore = Boolean(
    pageInfo?.has_more ??
    pageInfo?.hasMore ??
    data?.artifacts_has_more ??
    data?.artifact_has_more ??
    data?.artifacts_more ??
    data?.artifact_more ??
    data?.next_artifact_cursor ??
    data?.artifact_next_cursor ??
    data?.artifacts_next_cursor ??
    data?.next_cursor ??
    data?.next_offset ??
    data?.next_page
  )
  const normalized = {
    hasMore,
    cursor: `${pageInfo?.next_cursor ?? data?.artifact_next_cursor ?? data?.artifacts_next_cursor ?? data?.next_cursor ?? current.cursor ?? ''}`.trim(),
    offset: Number.isFinite(Number(pageInfo?.offset ?? pageInfo?.next_offset ?? data?.artifact_next_offset ?? data?.artifacts_next_offset ?? data?.next_offset ?? current.offset))
      ? Number(pageInfo?.offset ?? pageInfo?.next_offset ?? data?.artifact_next_offset ?? data?.artifacts_next_offset ?? data?.next_offset ?? current.offset)
      : current.offset || 0,
    page: Number.isFinite(Number(pageInfo?.next_page ?? data?.artifact_next_page ?? data?.artifacts_next_page ?? data?.next_page))
      ? Number(pageInfo?.next_page ?? data?.artifact_next_page ?? data?.artifacts_next_page ?? data?.next_page)
      : (hasMore ? currentPage + 1 : currentPage),
    limit: currentLimit,
  }

  if (!normalized.hasMore) {
    normalized.hasMore = Boolean(normalized.cursor || normalized.offset)
  }
  return normalized
}

function buildListQueryParams(pagination = {}) {
  const params = {}
  if (pagination.cursor) {
    params.cursor = pagination.cursor
    params.artifact_cursor = pagination.cursor
    params.artifacts_cursor = pagination.cursor
  }
  if (Number.isFinite(pagination.offset)) {
    params.offset = pagination.offset
    params.artifact_offset = pagination.offset
    params.artifacts_offset = pagination.offset
  }
  if (Number.isFinite(pagination.page)) {
    params.page = pagination.page
    params.artifact_page = pagination.page
    params.artifacts_page = pagination.page
  }
  if (Number.isFinite(pagination.limit) && pagination.limit > 0) {
    params.limit = pagination.limit
    params.artifact_page_size = pagination.limit
    params.artifacts_page_size = pagination.limit
  }
  return params
}

function selectedFleetVehicle(buildSource, selectedTarget) {
  return fleetVehicleByName(buildSource, selectedTarget?.vehicle_name || '')
}

function deviceReadinessSummary(device, buildSource, buildSourceLoaded = true, assignedVehicleName = '') {
  const sourceReady = describeBuildSource(buildSource, buildSourceLoaded).ready
  const assigned = Boolean(assignedVehicleName || device?.vehicle_name)
  const backendReadiness = device?.readiness && typeof device.readiness === 'object'
    ? {
      connected: Boolean(device.readiness.connected),
      build_installed: Boolean(device.readiness.build_installed),
      runtime_ready: Boolean(device.readiness.runtime_ready),
      vehicle_assigned: Boolean(device.readiness.vehicle_assigned) || assigned,
      ready: Boolean(device.readiness.ready),
      notes: Array.isArray(device.readiness.notes) ? device.readiness.notes.filter(Boolean) : [],
    }
    : null

  if (!buildSourceLoaded) {
    return {
      label: 'Loading setup',
      detail: 'Waiting for the global build and fleet selection to load.',
      ready: false,
    }
  }

  if (device?.discovery_stale) {
    return {
      label: 'Offline',
      detail: formatRelativeTimestamp(device.last_seen_at) || 'Recently seen on the network, but not present in the latest discovery pass.',
      ready: false,
    }
  }

  if (!sourceReady) {
    return {
      label: 'Needs setup',
      detail: 'Select a build and fleet before launching devices.',
      ready: false,
    }
  }

  if (backendReadiness) {
    const mergedReady = backendReadiness.connected
      && backendReadiness.build_installed
      && backendReadiness.runtime_ready
      && backendReadiness.vehicle_assigned

    if (mergedReady) {
      return {
        label: 'Ready for launch',
        detail: backendReadiness.notes[0] || 'Build, fleet, and device assignment are aligned.',
        ready: true,
      }
    }

    if (!backendReadiness.vehicle_assigned) {
      return {
        label: 'Needs assignment',
        detail: backendReadiness.notes[0] || 'Open the device drilldown and choose a controllable.',
        ready: false,
      }
    }

    if (!backendReadiness.connected) {
      return {
        label: 'Offline',
        detail: backendReadiness.notes[0] || 'SSH is unavailable for this device.',
        ready: false,
      }
    }

    if (!backendReadiness.build_installed) {
      return {
        label: 'Needs deploy',
        detail: backendReadiness.notes[0] || 'Deploy the selected build to this device.',
        ready: false,
      }
    }

    return {
      label: 'Runtime blocked',
      detail: backendReadiness.notes[0] || 'Runtime is not ready on this device.',
      ready: false,
    }
  }

  if (!assigned) {
    return {
      label: 'Needs assignment',
      detail: 'Open the device drilldown and choose a controllable.',
      ready: false,
    }
  }

  return {
    label: 'Ready for launch',
    detail: 'Build, fleet, and device assignment are aligned.',
    ready: true,
  }
}

function missionNameFromVehicle(vehicle) {
  if (!vehicle) return ''
  const missionPath = `${vehicle.missionPath || ''}`.trim()
  if (missionPath) {
    const normalized = missionPath.replace(/\\/g, '/')
    const basename = normalized.split('/').pop() || ''
    return basename.replace(/\.ya?ml$/i, '')
  }
  return `${vehicle.mission || ''}`.trim()
}

function normalizeBackendBuildSource(raw) {
  const sourceString = typeof raw?.source === 'string' ? raw.source.trim().toLowerCase() : ''
  const payload =
    raw?.build_source
    || raw?.buildSource
    || raw?.selected
    || raw?.selection
    || raw?.current
    || raw?.data
    || (raw?.source && typeof raw.source === 'object' ? raw.source : null)
    || raw
  if (!payload || typeof payload !== 'object') {
    return { kind: 'none' }
  }

  const kindHint = `${payload.kind || payload.source_kind || payload.source_type || payload.type || sourceString || ''}`.trim().toLowerCase()
  const normalizedKind = kindHint.replace(/_/g, '-')
  const fleetVehicles = Array.isArray(payload.fleet_vehicles)
    ? payload.fleet_vehicles.map(vehicle => ({
      name: `${vehicle?.name || ''}`.trim(),
      kind: vehicle?.kind ? `${vehicle.kind}`.trim() : '',
      mission: vehicle?.mission ? `${vehicle.mission}`.trim() : '',
      missionPath: vehicle?.mission_path ? `${vehicle.mission_path}`.trim() : '',
    })).filter(vehicle => vehicle.name)
    : []
  const fleetOptions = normalizeFleetOptions(
    payload.available_fleets || payload.availableFleets || payload.fleet_options || payload.fleetOptions || payload.fleet_files || payload.fleetFiles || payload.fleet_catalog || payload.fleetCatalog
  )
  const contextFields = {
    fleetFile: fleetSelectionId(payload.fleet_file || ''),
    fleetExists: payload.fleet_exists !== false,
    fleetError: payload.fleet_error || '',
    fleetCatalogError: payload.fleet_catalog_error || '',
    fleetCatalogStale: Boolean(payload.fleet_catalog_stale),
    fleetVehicles,
    fleetOptions,
  }
  if (!kindHint || kindHint === 'none' || kindHint === 'null' || kindHint === 'empty') {
    return { kind: 'none', ...contextFields }
  }

  if (normalizedKind === 'github' || normalizedKind === 'release' || normalizedKind === 'actions') {
    const build = normalizeBuildRecord({
      ...(payload.build || payload.selected_build || payload.build_info || payload),
      source: payload.github_source || payload.source || payload.source_type || sourceString || 'release',
    })
    return {
      kind: 'github',
      build,
      ...contextFields,
    }
  }

  if (
    normalizedKind === 'local-artifact'
    || normalizedKind === 'artifact'
    || normalizedKind === 'upload'
    || normalizedKind === 'artifact-file'
    || normalizedKind.startsWith('artifact')
  ) {
    return {
      kind: 'local-artifact',
      fileName: payload.file_name || payload.filename || payload.artifact_name || payload.source_label || payload.name || '',
      sourceLabel: payload.source_label || payload.artifact_name || payload.file_name || payload.filename || 'Local artifact bundle',
      cached: payload.cached !== false,
      ...contextFields,
    }
  }

  if (
    normalizedKind === 'local-codebase'
    || normalizedKind === 'codebase'
    || normalizedKind === 'source-build'
    || normalizedKind === 'source'
  ) {
    return {
      kind: 'local-codebase',
      sourceLabel: payload.source_label || payload.name || 'Current local codebase',
      packages: Array.isArray(payload.packages) ? payload.packages : [],
      ...contextFields,
    }
  }

  return { kind: 'none', ...contextFields }
}

function describeBuildSource(buildSource, loaded = true) {
  if (!loaded) {
    return {
      title: 'Loading build and fleet...',
      detail: 'Fetching the current build and fleet selection.',
      ready: false,
    }
  }

  if (!buildSource || buildSource.kind === 'none') {
    return {
      title: 'No build and fleet selected',
      detail: 'Select a build and fleet before opening device controls.',
      fleetDetail: buildSource?.fleetFile
        ? `Fleet: ${buildSource.fleetFile}`
        : 'Fleet file not selected.',
      ready: false,
    }
  }
  if (!buildSource.fleetFile || buildSource.fleetExists === false) {
    const fleetDetail = buildSource.fleetCatalogError && (!buildSource.fleetOptions || buildSource.fleetOptions.length === 0)
      ? buildSource.fleetCatalogError
      : buildSource.fleetError
    return {
      title: buildSource.fleetFile ? 'Fleet selection needs attention' : 'Fleet file not selected',
      detail: fleetDetail || 'Choose a valid fleet file before assigning and deploying devices.',
      fleetDetail: buildSource.fleetFile || 'No fleet file selected.',
      ready: false,
    }
  }
  if (buildSource.kind === 'github') {
    const build = buildSource.build
    return {
      title: buildDisplayTitle(build),
      detail: buildDisplayDetail(build),
      fleetDetail: buildSource.fleetFile,
      ready: true,
    }
  }
  if (buildSource.kind === 'local-artifact') {
    return {
      title: buildSource.sourceLabel || buildSource.fileName || 'Local artifact bundle',
      detail: buildSource.cached
        ? 'Cached by the backend and ready for deployment.'
        : 'Uploaded from this laptop and cached by the backend.',
      fleetDetail: buildSource.fleetFile,
      ready: true,
    }
  }
  if (buildSource.kind === 'local-codebase') {
    return {
      title: buildSource.sourceLabel || 'Current local codebase',
      detail: buildSource.packages?.length
        ? `Packages the local workspace into a source bundle (${buildSource.packages.length} package(s)).`
        : 'Packages the local workspace into a source bundle and builds it on the selected Pi.',
      fleetDetail: buildSource.fleetFile,
      ready: true,
    }
  }
  return {
    title: 'Unknown build and fleet context',
    detail: 'Select a supported build source.',
    fleetDetail: buildSource?.fleetFile || 'No fleet file selected.',
    ready: false,
  }
}

function buildSourceBadge(buildSource, loaded = true) {
  if (!loaded) return 'Loading...'
  if (!buildSource || buildSource.kind === 'none') return 'Not Selected'
  if (buildSource.kind === 'github') {
    return buildSource.build?.source === 'actions' ? 'Actions Artifact' : 'GitHub Release'
  }
  if (buildSource.kind === 'local-artifact') return 'Local Artifact'
  if (buildSource.kind === 'local-codebase') return 'Local Codebase'
  return 'Custom Source'
}

function normalizeFleetOptions(rawOptions) {
  if (!Array.isArray(rawOptions)) return []
  return rawOptions
    .map(option => {
      if (!option) return null
      if (typeof option === 'string') {
        const rawValue = option.trim()
        const value = fleetSelectionId(rawValue)
        if (!value) return null
        return {
          value,
          path: rawValue,
          label: value,
          description: '',
          disabled: false,
        }
      }
      if (typeof option !== 'object') return null
      const rawValue = `${option.fleet_file || option.path || option.value || option.file || option.name || ''}`.trim()
      const value = fleetSelectionId(rawValue)
      if (!value) return null
      return {
        value,
        path: rawValue,
        label: `${option.label || option.name || value}`.trim(),
        description: `${option.description || option.detail || ''}`.trim(),
        disabled: option.available === false,
      }
    })
    .filter(Boolean)
}

function fleetSelectionId(rawValue) {
  const value = `${rawValue || ''}`.trim().replace(/\\/g, '/')
  if (!value) return ''
  return value.split('/').filter(Boolean).pop() || value
}

function shortBuildSha(build) {
  const sha = `${build?.commitSha || build?.sha || ''}`.trim()
  return sha ? sha.slice(0, 8) : ''
}

function buildTimestampValue(build) {
  return build?.timestamp
    || build?.publishedAt
    || build?.updatedAt
    || build?.createdAt
    || build?.date
    || ''
}

function formatBuildTimestamp(build) {
  const value = `${buildTimestampValue(build) || ''}`.trim()
  if (!value) return ''
  if (/^\d{4}-\d{2}-\d{2}$/.test(value)) return value
  const parsed = Date.parse(value)
  if (!Number.isFinite(parsed)) return value
  return new Intl.DateTimeFormat(undefined, {
    year: 'numeric',
    month: 'short',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  }).format(new Date(parsed))
}

function buildDisplayTitle(build) {
  const sha = shortBuildSha(build)
  const subject = `${build?.commitSubject || build?.name || build?.artifactName || build?.artifact_name || build?.tag || 'Build'}`.trim()
  return sha ? `${sha} · ${subject}` : subject
}

function buildDisplayDetail(build) {
  const parts = []
  if (build?.branch) parts.push(build.branch)
  const timestamp = formatBuildTimestamp(build)
  if (timestamp) parts.push(timestamp)
  if (build?.workflowName) parts.push(build.workflowName)
  if (build?.jobName) parts.push(build.jobName)
  return parts.join(' · ') || 'No build metadata available'
}

function buildSearchText(build) {
  return [
    build?.sha,
    build?.commitSha,
    build?.commitSubject,
    build?.branch,
    build?.name,
    build?.artifactName,
    build?.artifact_name,
    build?.workflowName,
    build?.jobName,
    build?.tag,
    build?.runId,
    build?.runNumber,
    build?.source,
    build?.event,
    build?.conclusion,
    formatBuildTimestamp(build),
  ]
    .filter(Boolean)
    .join(' ')
    .toLowerCase()
}

function buildWorkflowMetadata(build) {
  if (build?.workflowName) return build.workflowName
  if (build?.jobName) return build.jobName
  if (build?.source === 'actions') {
    if (build?.run_id) return `GitHub Actions run ${build.run_id}`
    return 'GitHub Actions artifact'
  }
  return 'Workflow metadata unavailable'
}

function normalizeFleetBoardDevices(rawDevices) {
  if (!Array.isArray(rawDevices)) return []
  return rawDevices
    .map(device => {
      if (!device || typeof device !== 'object') return null
      const hardwareId = `${device.hardware_id || device.id || device.hostname || device.host || device.name || ''}`.trim()
      const hostname = `${device.hostname || device.host || device.pi_host || device.address || device.device_name || ''}`.trim()
      const deviceKey = hardwareId || hostname
      return {
        ...device,
        hardware_id: deviceKey,
        hostname,
        addresses: Array.isArray(device.addresses) ? device.addresses : Array.isArray(device.addrs) ? device.addrs : [],
        last_seen_at: `${device.last_seen_at || device.lastSeenAt || ''}`.trim(),
        discovery_stale: Boolean(device.discovery_stale ?? device.discoveryStale),
      }
    })
    .filter(device => Boolean(device?.hardware_id))
}

function inventoryDraftFromSelection(target, liveDevice, operatorConfig) {
  const hostname = target?.pi_host || liveDevice?.hostname || ''
  return {
    target_id: target?.target_id || suggestedTargetId(hostname),
    label: target?.label || hostname || 'Unnamed device',
    pi_host: target?.pi_host || hostname,
    pi_user: target?.pi_user || operatorConfig?.default_pi_user || 'penn',
    ssh_key: target?.ssh_key || operatorConfig?.default_ssh_key || '',
    ssh_pass: target?.ssh_pass || '',
    deploy_root: target?.deploy_root || operatorConfig?.default_deploy_root || '/home/penn/pennair-deploy',
    service_unit: target?.service_unit || 'pennair-autonomy.service',
  }
}

function assignmentDraftFromTarget(target, vehicleName = '') {
  return {
    target_id: target?.target_id || '',
    vehicle_name: vehicleName || target?.vehicle_name || '',
    overlay_yaml: target?.overlay_yaml || '',
  }
}

function fleetVehicleByName(buildSource, vehicleName) {
  return (buildSource?.fleetVehicles || []).find(vehicle => vehicle.name === vehicleName) || null
}

function EmptyState({ title, message, actionLabel, onAction }) {
  return (
    <div className="empty-state card card-full">
      <div className="empty-state-eyebrow">Integration</div>
      <h2>{title}</h2>
      <p>{message}</p>
      {actionLabel && onAction && (
        <button className="btn btn-primary empty-state-action" type="button" onClick={onAction}>
          {actionLabel}
        </button>
      )}
    </div>
  )
}

function AppHeader({
  page,
  onPageChange,
  theme,
  onToggleTheme,
  liveCount,
  readyCount,
  assignedCount,
  buildSource,
  buildSourceLoaded = true,
  buildSourceWorking = false,
}) {
  const sourceSummary = describeBuildSource(buildSource, buildSourceLoaded)
  const sourceBadge = buildSourceWorking ? 'Updating...' : buildSourceBadge(buildSource, buildSourceLoaded)

  return (
    <header className="app-header">
      <div className="app-brand">
        <div className="app-brand-kicker">PennAiR Fleet Ops</div>
        <h1 className="title">Fleet-first operations</h1>
        <p className="app-subtitle">Select the build and fleet globally, verify readiness, then drill into a device when runtime control is needed.</p>
      </div>
      <div className="app-header-meta">
        <div className="header-stat">
          <span className="header-stat-label">Live Devices</span>
          <strong>{liveCount}</strong>
        </div>
        <div className="header-stat">
          <span className="header-stat-label">Ready</span>
          <strong>{readyCount}</strong>
        </div>
        <div className="header-stat">
          <span className="header-stat-label">Assigned</span>
          <strong>{assignedCount}</strong>
        </div>
        <div className="header-stat">
          <span className="header-stat-label">Build/Fleet</span>
          <strong>{sourceBadge}</strong>
        </div>
        <button className="theme-toggle-btn" type="button" onClick={onToggleTheme}>
          {theme === THEME_DARK ? 'Light Mode' : 'Dark Mode'}
        </button>
      </div>
      <div className="page-tabs nav-tabs">
        <button className={`tab-btn ${page === 'setup' ? 'tab-active' : ''}`} onClick={() => onPageChange('setup')} disabled={buildSourceWorking}>
          Setup
        </button>
        <button className={`tab-btn ${page === 'readiness' ? 'tab-active' : ''}`} onClick={() => onPageChange('readiness')} disabled={buildSourceWorking}>
          Readiness
        </button>
        <button className={`tab-btn ${page === 'launch-monitor' ? 'tab-active' : ''}`} onClick={() => onPageChange('launch-monitor')} disabled={buildSourceWorking}>
          Launch &amp; Monitor
        </button>
      </div>
      <div className="build-source-banner">
        <span className={`launch-pill ${buildSourceWorking ? 'pill-not-prepared' : sourceSummary.ready ? 'pill-running' : 'pill-not-prepared'}`}>
          {sourceBadge}
        </span>
        <div>
          <strong>{sourceSummary.title}</strong>
          <p>{sourceSummary.detail}</p>
        </div>
      </div>
    </header>
  )
}

function LiveHardwareCard({ device, buildSource, buildSourceLoaded = true, onOpen }) {
  const assignedVehicleName = `${device?.session_vehicle_name || device?.vehicle_name || ''}`.trim()
  const readiness = deviceReadinessSummary(device, buildSource, buildSourceLoaded, assignedVehicleName)
  const discoveryText = device?.discovery_stale
    ? (formatRelativeTimestamp(device.last_seen_at) || 'Recently seen on the network')
    : (device.addresses?.[0] || 'mDNS')
  return (
    <button type="button" className="hardware-card" onClick={() => onOpen(device.hardware_id)}>
      <div className="hardware-card-top">
        <div>
          <div className="hardware-card-kicker">
            {device.discovery_stale ? 'Recent device' : assignedVehicleName ? 'Assigned device' : 'Live device'}
          </div>
          <h3>{device.hostname}</h3>
        </div>
        <span className={`launch-pill ${device.discovery_stale ? 'pill-offline' : readiness.ready ? 'pill-running' : 'pill-not-prepared'}`}>
          {readiness.label}
        </span>
      </div>
      <div className="hardware-card-host">{device.hostname}</div>
      <div className="hardware-card-meta">
        <span>{discoveryText}</span>
        <span>{assignedVehicleName ? `Vehicle: ${assignedVehicleName}` : 'No session assignment'}</span>
      </div>
      <div className="hardware-card-footer">
        <span className="hardware-card-source">{buildSourceBadge(buildSource, buildSourceLoaded)}</span>
        <span>{readiness.detail}</span>
      </div>
    </button>
  )
}

function HardwareGridPage({ devices, buildSource, buildSourceLoaded = true, discoveryResult, onRefresh, onOpenHardware }) {
  if (devices.length === 0) {
    return (
      <EmptyState
        title="No live devices detected"
        message={discoveryResult?.error || 'The dashboard only shows devices visible on the local network. Check mDNS visibility and Pi power before retrying.'}
        actionLabel="Refresh Discovery"
        onAction={onRefresh}
      />
    )
  }

  return (
    <>
      <div className="section-head">
        <div>
          <div className="section-kicker">Fleet readiness</div>
          <h2>Boarded devices</h2>
        </div>
        <button className="btn btn-secondary btn-inline" type="button" onClick={onRefresh}>
          Refresh
        </button>
      </div>
      <div className="hardware-grid">
        {devices.map(device => (
          <LiveHardwareCard
            key={device.hardware_id}
            device={device}
            buildSource={buildSource}
            buildSourceLoaded={buildSourceLoaded}
            onOpen={onOpenHardware}
          />
        ))}
      </div>
    </>
  )
}

function SetupField({ label, value, onChange, placeholder, type = 'text' }) {
  return (
    <div className="settings-field">
      <label>{label}</label>
      <input type={type} value={value} onChange={e => onChange(e.target.value)} placeholder={placeholder} />
    </div>
  )
}

function AssignmentCard({ draft, buildSource, buildSourceWorking, onChange, onSave, saving, saveResult, hostname = '' }) {
  const fleetVehicles = Array.isArray(buildSource?.fleetVehicles) ? buildSource.fleetVehicles : []
  const selectedVehicle = fleetVehicleByName(buildSource, draft?.vehicle_name || '')
  const canSave = Boolean(hostname && buildSource?.fleetExists && !buildSourceWorking)

  return (
    <div className="card">
      <h2 className="card-title">Launch Prep</h2>
      <div className="card-content settings-grid">
        <div className="info-box compact-info settings-field settings-field-full">
          <strong>Fleet context</strong>
          <p>{buildSource?.fleetFile || 'No fleet file selected.'}</p>
        </div>
        <div className="settings-field">
          <label>Controllable</label>
          <select value={draft?.vehicle_name || ''} onChange={e => onChange('vehicle_name', e.target.value)} disabled={buildSourceWorking}>
            <option value="">Select controllable</option>
            {fleetVehicles.map(vehicle => (
              <option key={vehicle.name} value={vehicle.name}>
                {vehicle.name}
                {vehicle.kind ? ` · ${vehicle.kind}` : ''}
                {vehicle.mission ? ` · ${vehicle.mission}` : ''}
              </option>
            ))}
          </select>
        </div>
        <div className="settings-field settings-field-full">
          <label>Selected fleet entry</label>
          <div className="info-box compact-info">
            {selectedVehicle ? (
              <>
                <strong>{selectedVehicle.name}</strong>
                <p>
                  {selectedVehicle.kind || 'unknown kind'}
                  {selectedVehicle.mission ? ` · mission ${selectedVehicle.mission}` : ''}
                  {selectedVehicle.missionPath ? ` · ${selectedVehicle.missionPath}` : ''}
                </p>
              </>
            ) : (
              <p>Select a controllable from the current fleet before deploying this Pi.</p>
            )}
          </div>
        </div>
        <div className="settings-field settings-field-full">
          <div className="info-box compact-info">
            <strong>Derived mission scope</strong>
            <p>The selected fleet entry supplies the base mission and launch config. Runtime overrides live in Mission Control once the host is selected.</p>
          </div>
        </div>
        <div className="settings-field settings-save">
          <button className="btn btn-primary" type="button" onClick={onSave} disabled={saving || !canSave}>
            {saving ? 'Saving...' : 'Save Session Assignment'}
          </button>
        </div>
        <Result data={saveResult} />
      </div>
    </div>
  )
}

function DeployActionCard({
  buildSource,
  buildSourceLoaded = true,
  buildSourceWorking = false,
  connected,
  targetId,
  hostname = '',
  assignedVehicleName,
  buildInfo,
  deploying,
  deployResult,
  onDeploy,
  onRollback,
}) {
  const sourceSummary = describeBuildSource(buildSource, buildSourceLoaded)
  const canDeploy = Boolean(assignedVehicleName) && Boolean(targetId || hostname) && buildSourceLoaded && sourceSummary.ready && !buildSourceWorking

  return (
    <div className="card">
      <h2 className="card-title">Fleet Deploy</h2>
      <div className="card-content">
        <div className="info-box compact-info">
          <strong>{sourceSummary.title}</strong>
          <p>{sourceSummary.detail}</p>
        </div>
        {buildSourceWorking && (
          <div className="busy-banner" aria-live="polite">
            Build and fleet context is updating. Deploy actions are locked.
          </div>
        )}
        {!targetId && !hostname && (
          <p className="subtext left-note">
            Open a device drilldown to deploy or stop that host.
          </p>
        )}
        {(targetId || hostname) && !sourceSummary.ready && !buildSourceLoaded && (
          <p className="subtext left-note">
            Loading build and fleet context...
          </p>
        )}
        {(targetId || hostname) && !sourceSummary.ready && buildSourceLoaded && (
          <p className="subtext left-note">
            Choose a build and fleet in global setup before deploying to this host.
          </p>
        )}
        {(targetId || hostname) && sourceSummary.ready && !assignedVehicleName && (
          <p className="subtext left-note">
            Choose a controllable assignment before deploying to this host.
          </p>
        )}
        <button className="btn btn-primary" type="button" onClick={onDeploy} disabled={!canDeploy || deploying}>
          {deploying ? 'Deploying...' : 'Deploy Fleet Context'}
        </button>
        <button
          className="btn btn-secondary"
          type="button"
          onClick={onRollback}
          disabled={(!targetId && !hostname) || !connected || !buildInfo?.installed || deploying}
        >
          Stop Fleet
        </button>
        <Result data={deployResult} />
      </div>
    </div>
  )
}

function RuntimeOverviewCard({ liveDevice, selectedTarget, connected, buildInfo, sshCommand, pollError }) {
  const [copied, setCopied] = useState(false)
  const reachabilityText = connected ? 'SSH reachable' : 'Not reachable'

  const copy = async () => {
    if (!sshCommand) return
    if (navigator.clipboard?.writeText) {
      try {
        await navigator.clipboard.writeText(sshCommand)
      } catch {
        // Ignore copy failure and leave the command visible.
      }
    }
    setCopied(true)
    window.setTimeout(() => setCopied(false), 1500)
  }

  return (
    <div className="card">
      <h2 className="card-title">Runtime Overview</h2>
      <div className="card-content">
        <div className="overview-stack">
          <div className="overview-row">
            <span>Hostname</span>
            <strong>{liveDevice?.hostname || selectedTarget?.pi_host || 'Unknown'}</strong>
          </div>
          <div className="overview-row">
            <span>Addresses</span>
            <strong>{liveDevice?.addresses?.join(', ') || 'Unavailable'}</strong>
          </div>
          <div className="overview-row">
            <span>Controllable</span>
            <strong>{selectedTarget?.vehicle_name || 'Not assigned'}</strong>
          </div>
          <div className="overview-row">
            <span>Reachability</span>
            <strong>{reachabilityText}</strong>
          </div>
        </div>
        {sshCommand && (
          <>
            <div className="ssh-box" onClick={copy}>
              <code>{sshCommand}</code>
              <span className="copy-tag">{copied ? 'Copied' : 'Copy'}</span>
            </div>
            <p className="subtext">Click to copy SSH command</p>
          </>
        )}
        {buildInfo?.info && (
          <div className="info-box compact-info">
            <pre>{buildInfo.info}</pre>
          </div>
        )}
        <Result data={pollError} />
      </div>
    </div>
  )
}

function HardwareDetailPage({
  liveDevice,
  selectedTarget,
  hostname,
  vehicleName,
  connected,
  wifiStatus,
  buildInfo,
  sshCommand,
  workspacePaths,
  pollError,
  buildSource,
  buildSourceLoaded = true,
  buildSourceWorking = false,
  assignmentDraft,
  onAssignmentChange,
  onSaveAssignment,
  savingAssignment,
  assignmentResult,
  onRefresh,
  onBack,
  detailTab,
  onDetailTabChange,
  onDeploy,
  deploying,
  deployResult,
  onRollback,
}) {
  const sourceSummary = describeBuildSource(buildSource, buildSourceLoaded)
  const selectedTargetId = selectedTarget?.target_id || hostname || ''
  const missionReady = Boolean(hostname || selectedTargetId)
  const liveStateLabel = liveDevice ? (liveDevice.discovery_stale ? 'Offline' : 'Live') : 'Offline'
  const liveStateClass = liveDevice && !liveDevice.discovery_stale ? 'pill-running' : 'pill-offline'

  return (
    <div className="detail-page">
      <div className="detail-header">
        <button className="btn btn-secondary btn-inline" type="button" onClick={onBack}>
          Back to readiness
        </button>
        <div className="detail-header-copy">
          <div className="section-kicker">Launch & monitor</div>
          <h2>{liveDevice?.hostname || hostname || 'Device'}</h2>
          <p>{selectedTarget?.vehicle_name ? `${selectedTarget.vehicle_name} · ${liveDevice?.hostname || hostname || 'live only'}` : `${liveDevice?.hostname || hostname || 'Unknown host'} · live only`}</p>
        </div>
        <div className="detail-header-pills">
          <span className={`launch-pill ${liveStateClass}`}>{liveStateLabel}</span>
          <span className={`launch-pill ${sourceSummary.ready ? 'pill-running' : 'pill-not-prepared'}`}>{buildSourceBadge(buildSource, buildSourceLoaded)}</span>
        </div>
      </div>

      <div className="mini-tabs detail-tabs">
        <button className={`mini-tab ${detailTab === 'setup' ? 'mini-tab-active' : ''}`} type="button" onClick={() => onDetailTabChange('setup')}>
          Launch Prep
        </button>
        <button
          className={`mini-tab ${detailTab === 'mission' ? 'mini-tab-active' : ''}`}
          type="button"
          onClick={() => onDetailTabChange('mission')}
          disabled={!missionReady}
        >
          Mission Control
        </button>
      </div>

      {detailTab === 'setup' ? (
        <>
          <div className="grid">
            <AssignmentCard
              draft={assignmentDraft}
              buildSource={buildSource}
              buildSourceWorking={buildSourceWorking}
              onChange={onAssignmentChange}
              onSave={onSaveAssignment}
              saving={savingAssignment}
              saveResult={assignmentResult}
              hostname={hostname || liveDevice?.hostname || selectedTarget?.pi_host || ''}
            />
            <RuntimeOverviewCard
              liveDevice={liveDevice}
              selectedTarget={selectedTarget}
              connected={connected}
              buildInfo={buildInfo}
              sshCommand={sshCommand}
              pollError={pollError}
            />
          </div>

          <div className="grid">
              <DeployActionCard
                buildSource={buildSource}
                buildSourceLoaded={buildSourceLoaded}
                buildSourceWorking={buildSourceWorking}
                connected={connected}
                targetId={selectedTargetId}
                hostname={hostname || liveDevice?.hostname || selectedTarget?.pi_host || ''}
                assignedVehicleName={assignmentDraft?.vehicle_name || selectedTarget?.vehicle_name || ''}
                buildInfo={buildInfo}
                deploying={deploying}
                deployResult={deployResult}
                onDeploy={onDeploy}
                onRollback={onRollback}
              />
            {(selectedTargetId || hostname || liveDevice?.hostname || selectedTarget?.pi_host) ? (
              <WifiCard
                connected={connected}
                wifiStatus={wifiStatus}
                onRefresh={onRefresh}
                targetId={selectedTargetId}
                hostname={hostname || liveDevice?.hostname || selectedTarget?.pi_host || ''}
                vehicleName={vehicleName || selectedTarget?.vehicle_name || ''}
              />
            ) : (
              <div className="card">
                <h2 className="card-title">Target WiFi</h2>
                <p className="subtext left-note">Select a live host before using SSH-driven WiFi management.</p>
              </div>
            )}
          </div>
        </>
      ) : (
            <MissionControl
              connected={connected}
              buildInfo={buildInfo}
              onRefresh={onRefresh}
              workspacePaths={workspacePaths}
              buildSource={buildSource}
              targetId={selectedTargetId}
              hostname={hostname || liveDevice?.hostname || selectedTarget?.pi_host || ''}
              vehicleName={vehicleName || selectedTarget?.vehicle_name || ''}
              selectedTarget={selectedTarget}
            />
        )}
      </div>
  )
}

function BuildSourcePage({
  buildSource,
  buildSourceLoaded = true,
  buildSourceResult,
  buildSourceWorking,
  releases,
  artifacts,
  artifactPagination,
  loadingBuilds,
  buildListResult,
  onLoadBuilds,
  onSelectGitHubBuild,
  onSelectLocalArtifact,
  onSelectLocalCodebase,
  onSelectFleetFile,
  onClearBuildSource,
}) {
  const [selectedBuildKey, setSelectedBuildKey] = useState('')
  const [buildSearch, setBuildSearch] = useState('')
  const [fleetDraft, setFleetDraft] = useState(fleetSelectionId(buildSource?.fleetFile || ''))
  const sourceSummary = describeBuildSource(buildSource, buildSourceLoaded)
  const fleetOptions = Array.isArray(buildSource?.fleetOptions) ? buildSource.fleetOptions : []

  useEffect(() => {
    if (buildSource?.kind === 'github' && buildSource.build) {
      setSelectedBuildKey(buildSelectionKey(buildSource.build))
    }
  }, [buildSource])

  useEffect(() => {
    const nextDraft = fleetSelectionId(buildSource?.fleetFile || '')
    if (!nextDraft) {
      setFleetDraft('')
      return
    }
    setFleetDraft(fleetOptions.some(option => option.value === nextDraft) ? nextDraft : '')
  }, [buildSource?.fleetFile, fleetOptions])
  const allBuilds = useMemo(() => mergeBuildLists(releases, artifacts), [releases, artifacts])
  const deferredSearch = useDeferredValue(buildSearch.trim().toLowerCase())
  const fleetCatalogWarning = buildSource?.fleetCatalogStale
    ? (buildSource?.fleetCatalogError || 'Using the last exact-build fleet catalog because live GitHub access is degraded.')
    : ''
  const fleetHelpText = buildSource?.kind === 'none'
    ? 'Select a build to load fleet choices.'
    : buildSource?.fleetCatalogError
      ? buildSource.fleetCatalogError
      : fleetOptions.length === 0
        ? 'No fleet files were found in the selected build source.'
        : 'Fleet choices come from the selected build source and are keyed by filename.'
  const filteredBuilds = useMemo(() => {
    if (!deferredSearch) return allBuilds
    return allBuilds.filter(build => buildSearchText(build).includes(deferredSearch))
  }, [allBuilds, deferredSearch])
  const canLoadMoreArtifacts = Boolean(artifactPagination?.hasMore)

  useEffect(() => {
    if (!selectedBuildKey && filteredBuilds.length > 0) {
      setSelectedBuildKey(buildSelectionKey(filteredBuilds[0]))
    }
  }, [filteredBuilds, selectedBuildKey])

  useEffect(() => {
    if (selectedBuildKey && !allBuilds.some(build => buildSelectionKey(build) === selectedBuildKey)) {
      setSelectedBuildKey(allBuilds[0] ? buildSelectionKey(allBuilds[0]) : '')
    }
  }, [allBuilds, selectedBuildKey])

  const buildRow = (build, selectedKey, onSelect, onUse) => {
    const key = buildSelectionKey(build)
    const active = key === selectedKey
    const busy = buildSourceWorking || loadingBuilds
    const artifactLabel = build.artifactName || build.artifact_name || build.name || build.tag || ''
    return (
      <div key={key} className={`build-row ${active ? 'build-row-active' : ''} ${busy ? 'build-row-busy' : ''}`}>
        <button className="build-row-select" type="button" onClick={onSelect} disabled={busy}>
          <div className="build-row-top">
            <span className="launch-pill">{formatBuildType(build)}</span>
            <strong>{buildDisplayTitle(build)}</strong>
          </div>
          <div className="build-row-meta">
            <span>{build.branch || 'No branch'}</span>
            <span>{formatBuildTimestamp(build) || 'No timestamp'}</span>
          </div>
          <div className="build-row-foot">
            <span>{buildWorkflowMetadata(build)}</span>
            {artifactLabel && <span>{artifactLabel}</span>}
          </div>
        </button>
        <button className="btn btn-primary btn-inline build-row-use" type="button" onClick={onUse} disabled={busy}>
          {busy ? 'Working...' : 'Use'}
        </button>
      </div>
    )
  }

  return (
    <>
      <div className="section-head">
        <div>
          <div className="section-kicker">Global setup</div>
          <h2>Choose the build and fleet</h2>
        </div>
      </div>

      <div className="card card-full">
        <h2 className="card-title">Active build and fleet</h2>
        <div className="info-box compact-info">
          <strong>{sourceSummary.title}</strong>
          <p>{sourceSummary.detail}</p>
          <p>{sourceSummary.fleetDetail || 'No fleet file selected.'}</p>
          {fleetCatalogWarning && (
            <p className="param-help param-help-warn">{fleetCatalogWarning}</p>
          )}
        </div>
        {buildSourceWorking && (
          <div className="busy-banner" aria-live="polite">
            Updating build and fleet context. Controls are locked until the backend finishes.
          </div>
        )}
        <div className="settings-actions">
          <button className="btn btn-secondary" type="button" onClick={onClearBuildSource} disabled={buildSourceWorking || !buildSourceLoaded || buildSource?.kind === 'none'}>
            {buildSourceWorking ? 'Working...' : 'Clear Selection'}
          </button>
        </div>
        <Result data={buildSourceResult} />
      </div>

      <div className="grid">
        <div className="card">
          <h2 className="card-title">Fleet catalog</h2>
          <div className="card-content">
            <p className="subtext left-note">One fleet is active at a time. Device drilldowns only choose the controllable inside this fleet.</p>
            <label>Fleet</label>
            <select
              value={fleetDraft}
              onChange={e => setFleetDraft(e.target.value)}
              disabled={buildSourceWorking || loadingBuilds || fleetOptions.length === 0}
            >
              <option value="">{fleetOptions.length > 0 ? 'Select fleet' : 'No fleet choices available'}</option>
              {fleetOptions.map(option => (
                <option key={option.value} value={option.value} disabled={option.disabled}>
                  {option.label}
                </option>
              ))}
            </select>
            {!fleetOptions.length && (
              <p className="param-help">{fleetHelpText}</p>
            )}
            {fleetOptions.length > 0 && fleetCatalogWarning && (
              <p className="param-help param-help-warn">{fleetCatalogWarning}</p>
            )}
            <button className="btn btn-primary" type="button" onClick={() => onSelectFleetFile(fleetDraft.trim())} disabled={buildSourceWorking || !fleetDraft.trim() || fleetOptions.length === 0}>
              {buildSourceWorking ? 'Saving...' : 'Save Fleet'}
            </button>
          </div>
        </div>

        <div className="card card-full">
          <h2 className="card-title">Build catalog</h2>
          <div className="card-content">
            <div className="build-source-toolbar">
              <input
                type="text"
                value={buildSearch}
                onChange={e => setBuildSearch(e.target.value)}
                placeholder="Search SHA, title, branch"
                disabled={loadingBuilds || buildSourceWorking}
              />
              <button className="btn btn-secondary btn-inline" type="button" onClick={onLoadBuilds} disabled={loadingBuilds || buildSourceWorking}>
                {loadingBuilds ? 'Loading...' : 'Refresh'}
              </button>
            </div>
            <div className="subtext build-source-count left-note">
              {filteredBuilds.length} shown
              {allBuilds.length !== filteredBuilds.length ? ` of ${allBuilds.length} loaded` : ''}
            </div>
            <div className={`build-list ${buildSourceWorking || loadingBuilds ? 'build-list-busy' : ''}`} aria-busy={buildSourceWorking || loadingBuilds}>
              {filteredBuilds.length > 0 ? filteredBuilds.map(build => buildRow(
                build,
                selectedBuildKey,
                () => setSelectedBuildKey(buildSelectionKey(build)),
                () => onSelectGitHubBuild(build)
              )) : (
                <p className="subtext left-note">
                  {allBuilds.length > 0
                    ? 'No builds match this search.'
                    : 'No builds loaded yet. Refresh to fetch releases and artifacts.'}
                </p>
              )}
            </div>
            {canLoadMoreArtifacts && (
              <button className="btn btn-secondary" type="button" onClick={() => onLoadBuilds({ appendArtifacts: true })} disabled={loadingBuilds || buildSourceWorking}>
                {loadingBuilds ? 'Loading...' : 'Load more history'}
              </button>
            )}
            <Result data={buildListResult} />
          </div>
        </div>

        <div className="card card-full">
          <h2 className="card-title">Source options</h2>
          <div className="card-content local-source-actions">
            <div className="file-upload">
              <input
                type="file"
                accept=".tar.gz,.tgz,.tar,.gz,application/gzip,application/x-gzip,application/x-tar,application/octet-stream"
                onChange={async e => {
                  const file = e.target.files?.[0] || null
                  if (file) {
                    await onSelectLocalArtifact(file)
                  }
                  e.target.value = ''
                }}
                id="global-artifact-file"
                disabled={buildSourceWorking}
              />
              <label htmlFor="global-artifact-file" className="file-label">
                {buildSourceWorking ? 'Uploading...' : 'Upload artifact bundle'}
              </label>
            </div>
            <button className="btn btn-primary" type="button" onClick={onSelectLocalCodebase} disabled={buildSourceWorking}>
              Use current workspace
            </button>
          </div>
        </div>
      </div>
    </>
  )
}

function InventoryPage({
  operatorDraft,
  onOperatorChange,
  onSaveOperator,
  operatorSaving,
  operatorResult,
  targets,
  selectedInventoryTargetId,
  onSelectInventoryTarget,
  inventoryDraft,
  onInventoryDraftChange,
  onSaveInventoryTarget,
  onDeleteInventoryTarget,
  inventorySaving,
  inventoryResult,
  inventoryFile,
  onInventoryFileChange,
  onExportInventory,
  onImportInventory,
  inventoryIoLoading,
  liveLookup,
  inventoryMode,
  onCreateInventoryTarget,
}) {
  const [showOperatorAdvanced, setShowOperatorAdvanced] = useState(false)
  const [showDeviceAdvanced, setShowDeviceAdvanced] = useState(false)

  return (
    <>
      <div className="section-head">
        <div>
          <div className="section-kicker">Inventory</div>
          <h2>Saved devices and operator defaults</h2>
        </div>
      </div>

      <div className="grid">
        <div className="card">
          <h2 className="card-title">Operator Defaults</h2>
          <div className="card-content settings-grid">
            <SetupField label="GitHub repo" value={operatorDraft.github_repo} onChange={value => onOperatorChange('github_repo', value)} placeholder="org/repo" />
            <SetupField label="GitHub token" value={operatorDraft.github_token} onChange={value => onOperatorChange('github_token', value)} placeholder="Optional token" type="password" />
            <SetupField label="Hotspot name" value={operatorDraft.hotspot_name} onChange={value => onOperatorChange('hotspot_name', value)} placeholder="penn-desktop" />
            <SetupField label="Default deploy root" value={operatorDraft.default_deploy_root} onChange={value => onOperatorChange('default_deploy_root', value)} placeholder="/home/penn/pennair-deploy" />
            <SetupField label="Default Pi user" value={operatorDraft.default_pi_user} onChange={value => onOperatorChange('default_pi_user', value)} placeholder="penn" />
            <SetupField label="Default SSH key" value={operatorDraft.default_ssh_key} onChange={value => onOperatorChange('default_ssh_key', value)} placeholder="~/.ssh/pennair_pi_ed25519" />
            <div className="settings-field settings-field-full">
              <button className="btn btn-secondary btn-inline" type="button" onClick={() => setShowOperatorAdvanced(value => !value)}>
                {showOperatorAdvanced ? 'Hide recovery auth' : 'Show recovery auth'}
              </button>
            </div>
            {showOperatorAdvanced && (
              <SetupField label="Default SSH password" value={operatorDraft.default_ssh_pass} onChange={value => onOperatorChange('default_ssh_pass', value)} placeholder="Optional password" type="password" />
            )}
            <div className="settings-field settings-save">
              <button className="btn btn-primary" type="button" onClick={onSaveOperator} disabled={operatorSaving}>
                {operatorSaving ? 'Saving...' : 'Save Defaults'}
              </button>
            </div>
            <Result data={operatorResult} />
          </div>
        </div>

        <div className="card">
          <h2 className="card-title">Saved Devices</h2>
          {targets.length > 0 ? (
            <div className="inventory-list">
              {targets.map(target => {
                const liveDevice = liveLookup.get(normalizeHostname(target.pi_host))
                return (
                  <button
                    key={target.target_id}
                    type="button"
                    className={`inventory-item ${selectedInventoryTargetId === target.target_id ? 'inventory-item-active' : ''}`}
                    onClick={() => onSelectInventoryTarget(target.target_id)}
                  >
                    <div>
                      <strong>{target.label}</strong>
                      <div className="subtext">{target.pi_host}</div>
                    </div>
                    <span className={`launch-pill ${liveDevice ? 'pill-running' : 'pill-offline'}`}>
                      {liveDevice ? 'Live' : 'Offline'}
                    </span>
                  </button>
                )
              })}
            </div>
          ) : (
            <div className="inventory-empty">
              <p className="subtext left-note">No saved devices yet. Open a live hardware card to draft one from the hostname, or create a new record below.</p>
            </div>
          )}
          <div className="divider" />
          <div className="card-content">
            <button className="btn btn-secondary" type="button" onClick={onCreateInventoryTarget}>
              New Device
            </button>
            <button className="btn btn-secondary" type="button" onClick={onExportInventory} disabled={inventoryIoLoading}>
              {inventoryIoLoading ? 'Working...' : 'Export Inventory'}
            </button>
            <div className="file-upload">
              <input
                type="file"
                accept=".json,application/json"
                onChange={e => onInventoryFileChange(e.target.files?.[0] || null)}
                id="inventory-import-file"
              />
              <label htmlFor="inventory-import-file" className="file-label">
                {inventoryFile ? inventoryFile.name : 'Choose inventory JSON'}
              </label>
            </div>
            <button className="btn btn-secondary" type="button" onClick={onImportInventory} disabled={inventoryIoLoading || !inventoryFile}>
              {inventoryIoLoading ? 'Working...' : 'Import Inventory'}
            </button>
          </div>
        </div>
      </div>

      {inventoryDraft && (
        <div className="card card-full">
          <h2 className="card-title">{inventoryMode === 'new' ? 'New Device Draft' : 'Device Record'}</h2>
          <div className="card-content settings-grid">
            <SetupField label="Label" value={inventoryDraft.label} onChange={value => onInventoryDraftChange('label', value)} placeholder="Payload Pi" />
            <SetupField label="Pi host" value={inventoryDraft.pi_host} onChange={value => onInventoryDraftChange('pi_host', value)} placeholder="payload-pi.local" />
            <div className="settings-field settings-field-full">
              <div className="info-box compact-info">
                <strong>Target ID</strong>
                <p>{inventoryDraft.target_id}</p>
                <p>{inventoryDraft.pi_host || 'Hostname not set yet.'}</p>
              </div>
            </div>
            <div className="settings-field settings-field-full">
              <button className="btn btn-secondary btn-inline" type="button" onClick={() => setShowDeviceAdvanced(value => !value)}>
                {showDeviceAdvanced ? 'Hide overrides' : 'Show overrides'}
              </button>
            </div>
            {showDeviceAdvanced && (
              <>
                <SetupField label="Pi user" value={inventoryDraft.pi_user} onChange={value => onInventoryDraftChange('pi_user', value)} placeholder="penn" />
                <SetupField label="SSH key path" value={inventoryDraft.ssh_key} onChange={value => onInventoryDraftChange('ssh_key', value)} placeholder="~/.ssh/pennair_pi_ed25519" />
                <SetupField label="Deploy root" value={inventoryDraft.deploy_root} onChange={value => onInventoryDraftChange('deploy_root', value)} placeholder="/home/penn/pennair-deploy" />
                <SetupField label="Systemd unit" value={inventoryDraft.service_unit} onChange={value => onInventoryDraftChange('service_unit', value)} placeholder="pennair-autonomy.service" />
                <SetupField label="SSH password" value={inventoryDraft.ssh_pass} onChange={value => onInventoryDraftChange('ssh_pass', value)} placeholder="Optional recovery password" type="password" />
              </>
            )}
            <div className="settings-actions">
              <button className="btn btn-primary" type="button" onClick={onSaveInventoryTarget} disabled={inventorySaving}>
                {inventorySaving ? 'Saving...' : 'Save Device'}
              </button>
              {inventoryMode !== 'new' && (
                <button className="btn btn-secondary" type="button" onClick={onDeleteInventoryTarget} disabled={inventorySaving}>
                  Delete Device
                </button>
              )}
            </div>
            <Result data={inventoryResult} />
          </div>
        </div>
      )}
    </>
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
  const [page, setPage] = useState('setup')
  const [theme, setTheme] = useState(readStoredTheme)
  const [operatorConfig, setOperatorConfig] = useState(null)
  const [targets, setTargets] = useState([])
  const [liveDevices, setLiveDevices] = useState([])
  const [selectedHardwareId, setSelectedHardwareId] = useState('')
  const [selectedInventoryTargetId, setSelectedInventoryTargetId] = useState('')
  const [inventoryMode, setInventoryMode] = useState('existing')
  const [detailTab, setDetailTab] = useState('setup')
  const [buildSource, setBuildSource] = useState({ kind: 'none' })
  const [buildSourceLoaded, setBuildSourceLoaded] = useState(false)
  const [buildSourceResult, setBuildSourceResult] = useState(null)
  const [buildSourceWorking, setBuildSourceWorking] = useState(false)
  const [releaseBuilds, setReleaseBuilds] = useState([])
  const [artifactBuilds, setArtifactBuilds] = useState([])
  const [artifactPagination, setArtifactPagination] = useState({ hasMore: false, cursor: '', offset: 0, page: 0, limit: 0 })
  const [loadingBuilds, setLoadingBuilds] = useState(false)
  const [buildListResult, setBuildListResult] = useState(null)

  const [connected, setConnected] = useState(false)
  const [wifiStatus, setWifiStatus] = useState(null)
  const [buildInfo, setBuildInfo] = useState(null)
  const [sshCommand, setSshCommand] = useState('')
  const [inventoryResult, setInventoryResult] = useState(null)
  const [discoveryResult, setDiscoveryResult] = useState(null)
  const [pollError, setPollError] = useState(null)
  const [assignmentDraft, setAssignmentDraft] = useState(null)
  const [assignmentDirty, setAssignmentDirty] = useState(false)
  const [assignmentSaving, setAssignmentSaving] = useState(false)
  const [assignmentResult, setAssignmentResult] = useState(null)
  const [sessionAssignments, setSessionAssignments] = useState({})
  const [inventoryDraft, setInventoryDraft] = useState(null)
  const [inventoryDirty, setInventoryDirty] = useState(false)
  const [operatorDraft, setOperatorDraft] = useState(null)
  const [operatorDirty, setOperatorDirty] = useState(false)
  const [operatorSaving, setOperatorSaving] = useState(false)
  const [operatorResult, setOperatorResult] = useState(null)
  const [inventorySaving, setInventorySaving] = useState(false)
  const [inventoryFile, setInventoryFile] = useState(null)
  const [inventoryIoLoading, setInventoryIoLoading] = useState(false)
  const [deploying, setDeploying] = useState(false)
  const [deployResult, setDeployResult] = useState(null)
  const buildSourceRefreshRequestRef = useRef(0)
  const boardRefreshRequestRef = useRef(0)
  const buildSourceRefreshInFlightRef = useRef(null)
  const boardRefreshInFlightRef = useRef(null)
  const assignmentScopeKeyRef = useRef('')
  const deployScopeKeyRef = useRef('')

  const targetMap = useMemo(
    () => Object.fromEntries(uniqueTargets(targets).map(target => [target.target_id, target])),
    [targets]
  )
  const liveLookup = useMemo(
    () => new Map(liveDevices.map(device => [normalizeHostname(device.hostname), device])),
    [liveDevices]
  )
  const displayedLiveDevices = useMemo(
    () => liveDevices.map(device => ({
      ...device,
      session_vehicle_name: sessionAssignments[normalizeHostname(device.hostname)] || `${device.vehicle_name || ''}`.trim(),
    })),
    [liveDevices, sessionAssignments]
  )
  const selectedLiveDevice = useMemo(
    () => displayedLiveDevices.find(device => device.hardware_id === selectedHardwareId) || null,
    [displayedLiveDevices, selectedHardwareId]
  )
  const selectedDeviceHostKey = normalizeHostname(selectedLiveDevice?.hostname || selectedHardwareId)
  const selectedDeviceHostname = selectedLiveDevice?.hostname || ''
  const selectedVehicleName = assignmentDraft?.vehicle_name
    || sessionAssignments[selectedDeviceHostKey]
    || `${selectedLiveDevice?.vehicle_name || ''}`.trim()
    || ''
  const selectedTarget = useMemo(() => {
    if (!selectedDeviceHostname && !selectedVehicleName) return null
    const hostname = `${selectedDeviceHostname || ''}`.trim()
    return {
      target_id: hostname || suggestedTargetId(hostname),
      pi_host: hostname,
      hostname,
      label: hostname || 'Selected device',
      vehicle_name: selectedVehicleName,
      workspace_paths: null,
    }
  }, [selectedDeviceHostname, selectedVehicleName])
  const selectedTargetId = selectedTarget?.target_id || selectedDeviceHostname || ''
  const workspacePaths = null
  const buildSourceSummary = useMemo(
    () => describeBuildSource(buildSource, buildSourceLoaded),
    [buildSource, buildSourceLoaded]
  )

  const refreshBuildSource = useCallback(async ({ preserveResult = false } = {}) => {
    if (buildSourceRefreshInFlightRef.current) {
      return buildSourceRefreshInFlightRef.current
    }
    const requestId = ++buildSourceRefreshRequestRef.current
    const refreshPromise = (async () => {
      const data = await api('/api/build-source')
      if (requestId !== buildSourceRefreshRequestRef.current) {
        return data
      }
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        if (!preserveResult) {
          setBuildSourceResult(prev => (prev && prev.success === false ? null : prev))
        }
      } else if (!buildSourceLoaded) {
        setBuildSource({ kind: 'none' })
        if (!preserveResult) {
          setBuildSourceResult(data)
        }
      } else if (!preserveResult) {
        setBuildSourceResult(data)
      }
      setBuildSourceLoaded(true)
      return data
    })()
    buildSourceRefreshInFlightRef.current = refreshPromise
    try {
      return await refreshPromise
    } finally {
      if (buildSourceRefreshInFlightRef.current === refreshPromise) {
        buildSourceRefreshInFlightRef.current = null
      }
    }
  }, [buildSourceLoaded])

  const refreshGlobal = useCallback(async () => {
    if (boardRefreshInFlightRef.current) {
      return boardRefreshInFlightRef.current
    }
    const requestId = ++boardRefreshRequestRef.current
    const refreshPromise = (async () => {
      const [configData, boardData] = await Promise.all([
        api('/api/config'),
        api('/api/fleet/board'),
      ])
      if (requestId !== boardRefreshRequestRef.current) {
        return
      }

      if (configData.success) {
        setOperatorConfig(configData.config)
      }

      if (boardData.success) {
        const boardTargets = uniqueTargets(
          boardData.targets
          || boardData.saved_targets
          || boardData.board?.targets
          || []
        )
        const boardDevices = normalizeFleetBoardDevices(
          boardData.devices
          || boardData.live_devices
          || boardData.board?.devices
          || boardData.hardware
          || []
        )
        setTargets(prev => (boardTargets.length > 0 ? boardTargets : prev))
        setLiveDevices(prev => mergeDiscoveryDevices(prev, boardDevices))
        setDiscoveryResult(null)
        return
      }

      const liveData = await api('/api/hardware/live')
      if (requestId !== boardRefreshRequestRef.current) {
        return
      }

      if (liveData.success) {
        setLiveDevices(prev => mergeDiscoveryDevices(prev, normalizeFleetBoardDevices(liveData.devices || [])))
        setDiscoveryResult(null)
      } else {
        setDiscoveryResult(liveData)
        setLiveDevices(prev => mergeDiscoveryDevices(prev, []))
      }
    })()
    boardRefreshInFlightRef.current = refreshPromise
    try {
      return await refreshPromise
    } finally {
      if (boardRefreshInFlightRef.current === refreshPromise) {
        boardRefreshInFlightRef.current = null
      }
    }
  }, [])

  const refreshDetail = useCallback(async ({ targetId = '', hostname = '', vehicleName = '' } = {}) => {
    if (!targetId && !hostname) {
      setConnected(false)
      setWifiStatus(null)
      setBuildInfo(null)
      setSshCommand('')
      setPollError(null)
      return
    }

    const targetUrl = url => withDeviceScope(url, { targetId, hostname, vehicleName })
    const conn = await api(targetUrl('/api/connection/status'))
    const sshPromise = api(targetUrl('/api/connection/ssh-command'))

    setConnected(Boolean(conn?.connected))

    if (!conn?.connected) {
      const ssh = await sshPromise
      if (ssh.success) setSshCommand(ssh.command)
      setWifiStatus(null)
      setBuildInfo(null)
      setPollError(conn?.error ? { success: false, error: conn.error } : null)
      return
    }

    const [wifi, build, ssh] = await Promise.all([
      api(targetUrl('/api/wifi/status')),
      api(targetUrl('/api/builds/current')),
      sshPromise,
    ])

    setWifiStatus(wifi.success ? wifi : null)
    setBuildInfo(build.success ? build : null)
    if (ssh.success) setSshCommand(ssh.command)

    const err = (!wifi.success ? wifi?.error : null) || (!build.success ? build?.error : null) || null
    setPollError(err ? { success: false, error: err } : null)
  }, [])

  const refreshSelectedDeviceState = useCallback(async () => {
    await Promise.allSettled([
      refreshDetail({
        targetId: selectedTargetId,
        hostname: selectedDeviceHostname,
        vehicleName: selectedVehicleName,
      }),
      refreshGlobal(),
    ])
  }, [refreshDetail, refreshGlobal, selectedDeviceHostname, selectedTargetId, selectedVehicleName])

  useEffect(() => {
    const refreshIntervalMs = page === 'launch-monitor' ? 2000 : page === 'readiness' ? 3000 : 5000
    refreshBuildSource()
    refreshGlobal()
    const interval = setInterval(refreshGlobal, refreshIntervalMs)
    const sourceInterval = setInterval(refreshBuildSource, refreshIntervalMs)
    return () => {
      clearInterval(interval)
      clearInterval(sourceInterval)
    }
  }, [page, refreshBuildSource, refreshGlobal])

  useEffect(() => {
    if (page !== 'launch-monitor') {
      refreshDetail()
      return undefined
    }
    refreshDetail({
      targetId: selectedTargetId,
      hostname: selectedDeviceHostname,
      vehicleName: selectedVehicleName,
    })
    if (!selectedTargetId && !selectedDeviceHostname) return undefined
    const interval = setInterval(() => refreshDetail({
      targetId: selectedTargetId,
      hostname: selectedDeviceHostname,
      vehicleName: selectedVehicleName,
    }), 5000)
    return () => clearInterval(interval)
  }, [page, refreshDetail, selectedDeviceHostname, selectedTargetId, selectedVehicleName])

  useEffect(() => {
    const fleetVehicles = new Set((buildSource?.fleetVehicles || []).map(vehicle => vehicle.name))
    setSessionAssignments(prev => {
      const next = {}
      const claimedVehicles = new Set()

      Object.entries(prev).forEach(([hostKey, vehicleName]) => {
        if (!liveLookup.has(hostKey)) return
        if (fleetVehicles.size > 0 && !fleetVehicles.has(vehicleName)) return
        if (claimedVehicles.has(vehicleName)) return
        next[hostKey] = vehicleName
        claimedVehicles.add(vehicleName)
      })

      liveDevices.forEach(device => {
        const hostKey = normalizeHostname(device.hostname)
        const vehicleName = `${device.vehicle_name || ''}`.trim()
        if (!vehicleName) return
        if (fleetVehicles.size > 0 && !fleetVehicles.has(vehicleName)) return
        if (next[hostKey] || claimedVehicles.has(vehicleName)) return
        next[hostKey] = vehicleName
        claimedVehicles.add(vehicleName)
      })

      const prevEntries = Object.entries(prev).sort()
      const nextEntries = Object.entries(next).sort()
      if (
        prevEntries.length === nextEntries.length
        && prevEntries.every((entry, index) => entry[0] === nextEntries[index][0] && entry[1] === nextEntries[index][1])
      ) {
        return prev
      }
      return next
    })
  }, [buildSource?.fleetVehicles, liveDevices, liveLookup])

  useEffect(() => {
    if (!operatorConfig || operatorDirty) return
    setOperatorDraft(operatorConfig)
  }, [operatorConfig, operatorDirty])

  useEffect(() => {
    if (assignmentDirty) return
    const sessionVehicleName = sessionAssignments[selectedDeviceHostKey] || ''
    if (selectedTarget) {
      setAssignmentDraft(assignmentDraftFromTarget(selectedTarget, sessionVehicleName))
    } else if (selectedLiveDevice) {
      setAssignmentDraft(assignmentDraftFromTarget(selectedTarget, sessionVehicleName))
    } else {
      setAssignmentDraft(null)
    }
    const scopeKey = [
      selectedDeviceHostKey,
      selectedTarget?.target_id || '',
      selectedLiveDevice?.hostname || '',
    ].join('|')
    if (assignmentScopeKeyRef.current && assignmentScopeKeyRef.current !== scopeKey) {
      setAssignmentResult(null)
    }
    if (deployScopeKeyRef.current && deployScopeKeyRef.current !== scopeKey) {
      setDeployResult(null)
    }
    assignmentScopeKeyRef.current = scopeKey
    deployScopeKeyRef.current = scopeKey
    setDetailTab('setup')
  }, [
    selectedLiveDevice?.hardware_id,
    selectedLiveDevice?.hostname,
    selectedTarget?.overlay_yaml,
    selectedTarget?.target_id,
    selectedTarget?.vehicle_name,
    assignmentDirty,
    selectedDeviceHostKey,
    sessionAssignments,
  ])

  useEffect(() => {
    if (page !== 'launch-monitor' || !selectedHardwareId) return
    if (buildSourceSummary.ready) return
    setSelectedHardwareId('')
    setDetailTab('setup')
    setDeployResult(null)
    setPage('setup')
  }, [buildSourceSummary.ready, page, selectedHardwareId])

  useEffect(() => {
    if (page === 'launch-monitor') return
    if (!selectedHardwareId) return
    setSelectedHardwareId('')
    setDetailTab('setup')
    setDeployResult(null)
  }, [page, selectedHardwareId])

  useEffect(() => {
    if (inventoryMode === 'new') {
      return
    }
    if (!selectedInventoryTargetId) {
      setInventoryDraft(null)
      return
    }
    const target = targetMap[selectedInventoryTargetId] || null
    if (inventoryDirty) return
    setInventoryDraft(target ? inventoryDraftFromSelection(target, null, operatorConfig) : null)
  }, [inventoryMode, operatorConfig, selectedInventoryTargetId, targetMap, inventoryDirty])

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

  const handleSaveOperator = async () => {
    if (!operatorDraft) return
    setOperatorSaving(true)
    const fd = new FormData()
    Object.entries(operatorDraft).forEach(([key, value]) => {
      if (key === 'inventory_path') return
      fd.append(key, value ?? '')
    })
    const data = await api('/api/config', { method: 'POST', body: fd })
    setOperatorResult(data)
    setOperatorSaving(false)
    if (data.success) {
      setOperatorDirty(false)
      await refreshGlobal()
    }
  }

  const handleSaveTargetDraft = async (draft, setSaving, setResult, scope) => {
    if (!draft) return
    setSaving(true)
    const fd = new FormData()
    Object.entries(draft).forEach(([key, value]) => fd.append(key, value ?? ''))
    const data = await api('/api/inventory', { method: 'POST', body: fd })
    setResult(data)
    setSaving(false)
    if (data.success) {
      const savedTarget = {
        ...(targetMap[draft.target_id] || {}),
        ...draft,
      }
      setTargets(prev => uniqueTargets([...prev.filter(target => target.target_id !== savedTarget.target_id), savedTarget]))
      if (scope === 'assignment') {
        setAssignmentDirty(false)
      }
      if (scope === 'inventory') {
        setInventoryDirty(false)
        setInventoryMode('existing')
      }
      if (draft.target_id) {
        setSelectedInventoryTargetId(draft.target_id)
      }
      await refreshGlobal()
    }
  }

  const handleDeleteInventoryTarget = async () => {
    if (!inventoryDraft?.target_id) return
    setInventorySaving(true)
    const fd = new FormData()
    fd.append('target_id', inventoryDraft.target_id)
    const data = await api('/api/inventory/delete', { method: 'POST', body: fd })
    setInventoryResult(data)
    setInventorySaving(false)
    if (data.success) {
      setTargets(prev => prev.filter(target => target.target_id !== inventoryDraft.target_id))
      setSelectedInventoryTargetId('')
      await refreshGlobal()
    }
  }

  const handleExportInventory = async () => {
    setInventoryIoLoading(true)
    try {
      const res = await fetch('/api/inventory/export')
      const raw = await res.text()
      if (!res.ok) throw new Error(raw || `HTTP ${res.status}`)
      const blob = new Blob([raw], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      const anchor = document.createElement('a')
      anchor.href = url
      anchor.download = 'integration-inventory.json'
      document.body.appendChild(anchor)
      anchor.click()
      anchor.remove()
      URL.revokeObjectURL(url)
      setInventoryResult({ success: true, output: 'Inventory exported' })
    } catch (error) {
      setInventoryResult({ success: false, error: error.message || 'Inventory export failed' })
    } finally {
      setInventoryIoLoading(false)
    }
  }

  const handleImportInventory = async () => {
    if (!inventoryFile) return
    setInventoryIoLoading(true)
    const fd = new FormData()
    fd.append('file', inventoryFile)
    const data = await api('/api/inventory/import', { method: 'POST', body: fd })
    setInventoryResult(data)
    setInventoryIoLoading(false)
    if (data.success) {
      setInventoryFile(null)
      await refreshGlobal()
    }
  }

  const loadBuilds = async ({ appendArtifacts = false } = {}) => {
    setLoadingBuilds(true)
    setBuildListResult(null)
    const query = appendArtifacts ? buildListQueryParams(artifactPagination) : {}
    const url = withQueryParams('/api/builds/list', query)
    const data = await api(url)
    if (data.success) {
      const normalizedReleases = Array.isArray(data.releases)
        ? data.releases.map(normalizeBuildRecord).filter(Boolean)
        : []
      const normalizedArtifacts = Array.isArray(data.artifacts)
        ? data.artifacts.map(normalizeBuildRecord).filter(Boolean)
        : []
      const normalizedBuilds = Array.isArray(data.builds)
        ? data.builds.map(normalizeBuildRecord).filter(Boolean)
        : []
      const nextReleases = Array.isArray(data.releases)
        ? normalizedReleases
        : Array.isArray(data.builds)
          ? normalizedBuilds.filter(build => `${build?.source || ''}`.toLowerCase() !== 'actions')
          : []
      const nextArtifacts = Array.isArray(data.artifacts)
        ? normalizedArtifacts
        : Array.isArray(data.builds)
          ? normalizedBuilds.filter(build => `${build?.source || ''}`.toLowerCase() === 'actions')
          : []
      setReleaseBuilds(nextReleases)
      setArtifactBuilds(prev => (
        appendArtifacts ? mergeBuildLists(prev, nextArtifacts) : nextArtifacts
      ))
      setArtifactPagination(prev => buildListPaginationFromResponse(data, appendArtifacts ? prev : {}))
    }
    setBuildListResult(
      data.success
        ? {
          success: true,
          output: data.error || (
            appendArtifacts
              ? `Loaded ${data.artifacts?.length || 0} more artifact(s)`
              : `Loaded ${(data.releases || []).length + (data.artifacts || []).length || (data.builds || []).length || 0} deployable build(s)`
          ),
        }
        : data
    )
    setLoadingBuilds(false)
  }

  const selectGithubBuildSource = async build => {
    if (!build) return
    buildSourceRefreshRequestRef.current += 1
    setBuildSourceWorking(true)
    try {
      const fd = new FormData()
      fd.append('tag', build.tag || '')
      fd.append('source', build.source || 'release')
      fd.append('sha', build.sha || '')
      fd.append('commit_subject', build.commitSubject || '')
      fd.append('name', build.name || '')
      fd.append('date', build.date || '')
      fd.append('download_url', build.download_url || '')
      if (build.size_mb !== null && build.size_mb !== undefined && build.size_mb !== '') {
        fd.append('size_mb', `${build.size_mb}`)
      }
      fd.append('branch', build.branch || '')
      fd.append('workflow_name', build.workflowName || '')
      fd.append('workflow_event', build.event || '')
      fd.append('workflow_conclusion', build.conclusion || '')
      fd.append('run_number', build.runNumber || '')
      fd.append('timestamp', build.timestamp || '')
      if (build.artifact_id) {
        fd.append('artifact_id', build.artifact_id)
      }
      if (build.artifact_name) {
        fd.append('artifact_name', build.artifact_name)
      }
      if (build.run_id) {
        fd.append('run_id', build.run_id)
      }
      const data = await api('/api/build-source/github', { method: 'POST', body: fd })
      setBuildSourceResult(data)
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        setBuildSourceLoaded(true)
        await refreshBuildSource({ preserveResult: true })
      }
    } finally {
      setBuildSourceWorking(false)
    }
  }

  const saveGlobalFleetFile = async fleetFile => {
    buildSourceRefreshRequestRef.current += 1
    setBuildSourceWorking(true)
    try {
      const selectedFleetId = fleetSelectionId(fleetFile)
      const fd = new FormData()
      fd.append('fleet_file', selectedFleetId || '')
      const data = await api('/api/build-source/fleet', { method: 'POST', body: fd })
      setBuildSourceResult(data)
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        setBuildSourceLoaded(true)
        await refreshBuildSource({ preserveResult: true })
      }
    } finally {
      setBuildSourceWorking(false)
    }
  }

  const selectLocalArtifactSource = async file => {
    if (!file) return
    buildSourceRefreshRequestRef.current += 1
    setBuildSourceWorking(true)
    try {
      const fd = new FormData()
      fd.append('file', file)
      const data = await api('/api/build-source/local-artifact', { method: 'POST', body: fd })
      setBuildSourceResult(data)
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        setBuildSourceLoaded(true)
        await refreshBuildSource({ preserveResult: true })
      }
    } finally {
      setBuildSourceWorking(false)
    }
  }

  const selectLocalCodebaseSource = async () => {
    buildSourceRefreshRequestRef.current += 1
    setBuildSourceWorking(true)
    try {
      const data = await api('/api/build-source/local-codebase', { method: 'POST', body: new FormData() })
      setBuildSourceResult(data)
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        setBuildSourceLoaded(true)
        await refreshBuildSource({ preserveResult: true })
      }
    } finally {
      setBuildSourceWorking(false)
    }
  }

  const clearBuildSource = async () => {
    buildSourceRefreshRequestRef.current += 1
    setBuildSourceWorking(true)
    try {
      const data = await api('/api/build-source/clear', { method: 'POST', body: new FormData() })
      setBuildSourceResult(data)
      if (data.success) {
        setBuildSource(normalizeBackendBuildSource(data))
        setBuildSourceLoaded(true)
        await refreshBuildSource({ preserveResult: true })
      }
    } finally {
      setBuildSourceWorking(false)
    }
  }

  const handleDeploySelectedSource = async () => {
    if (!buildSourceLoaded || !buildSource || buildSource.kind === 'none') return
    if (!selectedTargetId && !selectedDeviceHostname) return
    setDeploying(true)
    setDeployResult(null)
    try {
      const data = normalizeFleetActionResult(await api(withDeviceScope('/api/fleet/deploy', {
        targetId: selectedTargetId,
        hostname: selectedDeviceHostname,
        vehicleName: selectedVehicleName,
      }), { method: 'POST' }))

      setDeployResult(data)
    } finally {
      await refreshSelectedDeviceState()
      setDeploying(false)
    }
  }

  const handleStopFleet = async () => {
    if (!selectedTargetId && !selectedDeviceHostname) return
    setDeploying(true)
    try {
      const data = normalizeFleetActionResult(await api(withDeviceScope('/api/fleet/stop', {
        targetId: selectedTargetId,
        hostname: selectedDeviceHostname,
        vehicleName: selectedVehicleName,
      }), { method: 'POST' }))
      setDeployResult(data)
    } finally {
      await refreshSelectedDeviceState()
      setDeploying(false)
    }
  }

  const openHardware = hardwareId => {
    if (!buildSourceSummary.ready) {
      setBuildSourceResult({
        success: false,
        error: 'Select the global build and fleet before opening device controls.',
      })
      setPage('setup')
      return
    }
    setAssignmentDirty(false)
    setSelectedHardwareId(hardwareId)
    setDetailTab('setup')
    setPage('launch-monitor')
  }

  const handleSaveSessionAssignment = async () => {
    const hostname = `${selectedDeviceHostname || ''}`.trim()
    if (!hostname) return

    setAssignmentSaving(true)
    setAssignmentResult(null)
    setDeployResult(null)

    const hostKey = normalizeHostname(hostname)
    const vehicleName = `${assignmentDraft?.vehicle_name || ''}`.trim()
    const conflictingHost = Object.entries(sessionAssignments).find(
      ([candidateHost, assignedVehicle]) => candidateHost !== hostKey && assignedVehicle === vehicleName
    )?.[0]

    if (vehicleName && conflictingHost) {
      setAssignmentResult({
        success: false,
        error: `${vehicleName} is already assigned to ${conflictingHost}. Clear that session assignment first.`,
      })
      setAssignmentSaving(false)
      return
    }

    setSessionAssignments(prev => {
      const next = { ...prev }
      delete next[hostKey]
      if (vehicleName) {
        next[hostKey] = vehicleName
      }
      return next
    })
    setAssignmentDirty(false)
    setAssignmentResult({
      success: true,
      output: vehicleName
        ? `Assigned ${vehicleName} to ${hostname} for this session.`
        : `Cleared the session assignment for ${hostname}.`,
    })
    setAssignmentSaving(false)
  }

  return (
    <div className="app">
      <AppHeader
        page={page}
        onPageChange={setPage}
        theme={theme}
        onToggleTheme={toggleTheme}
        liveCount={displayedLiveDevices.length}
        readyCount={displayedLiveDevices.filter(device => deviceReadinessSummary(device, buildSource, buildSourceLoaded, device.session_vehicle_name).ready).length}
        assignedCount={displayedLiveDevices.filter(device => device.session_vehicle_name).length}
        buildSource={buildSource}
        buildSourceLoaded={buildSourceLoaded}
        buildSourceWorking={buildSourceWorking}
      />

      {page === 'setup' && (
        <BuildSourcePage
          buildSource={buildSource}
          buildSourceLoaded={buildSourceLoaded}
          buildSourceResult={buildSourceResult}
          buildSourceWorking={buildSourceWorking}
          releases={releaseBuilds}
          artifacts={artifactBuilds}
          artifactPagination={artifactPagination}
          loadingBuilds={loadingBuilds}
          buildListResult={buildListResult}
          onLoadBuilds={loadBuilds}
          onSelectGitHubBuild={selectGithubBuildSource}
          onSelectLocalArtifact={selectLocalArtifactSource}
          onSelectLocalCodebase={selectLocalCodebaseSource}
          onSelectFleetFile={saveGlobalFleetFile}
          onClearBuildSource={clearBuildSource}
        />
      )}

      {page === 'readiness' && (
        <HardwareGridPage
          devices={displayedLiveDevices}
          buildSource={buildSource}
          buildSourceLoaded={buildSourceLoaded}
          discoveryResult={discoveryResult}
          onRefresh={refreshGlobal}
          onOpenHardware={openHardware}
        />
      )}

      {page === 'launch-monitor' && !selectedHardwareId && (
        <div className="card card-full">
          <h2 className="card-title">Launch Monitor</h2>
          <div className="card-content">
            <div className="info-box compact-info">
              <strong>{buildSourceSummary.title}</strong>
              <p>{buildSourceSummary.detail}</p>
              <p>{buildSourceSummary.fleetDetail || 'No fleet file selected.'}</p>
            </div>
            <p className="subtext left-note">
              Select a live device from the readiness board to open mission control, launch prep, and runtime monitoring.
            </p>
          </div>
        </div>
      )}

      {page === 'launch-monitor' && selectedHardwareId && (
        <HardwareDetailPage
          liveDevice={selectedLiveDevice}
          selectedTarget={selectedTarget}
          hostname={selectedDeviceHostname}
          vehicleName={selectedVehicleName}
          connected={connected}
          wifiStatus={wifiStatus}
          buildInfo={buildInfo}
          sshCommand={sshCommand}
          workspacePaths={workspacePaths}
          pollError={pollError}
          buildSource={buildSource}
          buildSourceLoaded={buildSourceLoaded}
          buildSourceWorking={buildSourceWorking}
          assignmentDraft={assignmentDraft || assignmentDraftFromTarget(selectedTarget, selectedVehicleName)}
          onAssignmentChange={(field, value) => {
            setAssignmentDirty(true)
            setAssignmentDraft(prev => ({ ...(prev || {}), [field]: value }))
          }}
          onSaveAssignment={handleSaveSessionAssignment}
          savingAssignment={assignmentSaving}
          assignmentResult={assignmentResult}
          onRefresh={() => {
            refreshGlobal()
            refreshDetail({
              targetId: selectedTargetId,
              hostname: selectedDeviceHostname,
              vehicleName: selectedVehicleName,
            })
          }}
          onBack={() => {
            setAssignmentDirty(false)
            setSelectedHardwareId('')
            setPage('readiness')
          }}
          detailTab={detailTab}
          onDetailTabChange={setDetailTab}
          onDeploy={handleDeploySelectedSource}
          deploying={deploying}
          deployResult={deployResult}
          onRollback={handleStopFleet}
        />
      )}
    </div>
  )
}

export default App
