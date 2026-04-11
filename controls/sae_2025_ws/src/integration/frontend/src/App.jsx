import { useState, useEffect, useCallback, useMemo } from 'react'
import '@xterm/xterm/css/xterm.css'
import './App.css'
import { useMissionControl } from './hooks/useMissionControl'
import { api, withTargetFormData, withTargetId } from './services/api'

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

function StatusBar({ connected, wifiStatus, buildInfo, selectedTarget }) {
  const wifiText = connected
    ? (wifiStatus?.is_hotspot ? 'Hotspot' : wifiStatus?.current_wifi || 'No client WiFi')
    : 'Unavailable (target offline)'

  const buildText = connected
    ? (buildInfo?.installed ? 'Build active' : 'No build')
    : 'Unavailable (target offline)'

  const targetText = connected
    ? getTargetLabel(selectedTarget || 'No target selected')
    : 'Unavailable (target offline)'

  return (
    <div className="status-bar">
      <div className="status-item">
        <span className={`status-dot ${connected ? 'dot-ok' : 'dot-err'}`} />
        <span>{connected ? 'Target connected' : 'Target unreachable'}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${connected && wifiStatus?.current_wifi ? 'dot-ok' : 'dot-warn'}`} />
        <span>{wifiText}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${connected && buildInfo?.installed ? 'dot-ok' : 'dot-warn'}`} />
        <span>{buildText}</span>
      </div>
      <div className="status-item">
        <span className={`status-dot ${connected && selectedTarget ? 'dot-ok' : 'dot-warn'}`} />
        <span>{targetText}</span>
      </div>
    </div>
  )
}

function TargetSelector({ connected, targets, selectedTargetId, onChange, loading }) {
  const selectedLabel = targets.find(target => target.target_id === selectedTargetId)
    ? getTargetLabel(targets.find(target => target.target_id === selectedTargetId))
    : 'No target selected'

  return (
    <div className="target-selector">
      <div className="target-selector-head">
        <label>Target</label>
        <span className="target-selector-label">{connected ? selectedLabel : 'Offline'}</span>
      </div>
      <select
        value={selectedTargetId}
        onChange={e => onChange(e.target.value)}
        disabled={loading || targets.length === 0}
      >
        {targets.length === 0 && (
          <option value="">No targets available</option>
        )}
        {targets.map(target => (
          <option key={target.target_id} value={target.target_id}>
            {getTargetLabel(target)}
          </option>
        ))}
      </select>
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
      <h2 className="card-title">Target SSH</h2>
      <div className="ssh-box" onClick={copy}>
        <code>{sshCommand || '...'}</code>
        <span className="copy-tag">{copied ? 'Copied' : 'Copy'}</span>
      </div>
      <p className="subtext">Click to copy SSH command</p>
    </div>
  )
}

function WifiCard({ connected, wifiStatus, onRefresh, targetId }) {
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
  }, [selectedTarget, targetId])

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
    const data = await api(withTargetId('/api/wifi/scan', targetId))
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
    const data = await api(withTargetId('/api/wifi/connect', targetId), { method: 'POST', body: withTargetFormData(fd, targetId) })
    setResult(data)
    setLoading(false)
    if (data.success) {
      const macFd = new FormData()
      macFd.append('ssid', selectedSsid)
      macFd.append('password', password)
      await api(withTargetId('/api/wifi/switch-local', targetId), { method: 'POST', body: withTargetFormData(macFd, targetId) })
    }
    onRefresh()
  }

  const hotspot = async () => {
    if (!connected) return
    setLoading(true)
    setResult(null)
    const data = await api(withTargetId('/api/wifi/hotspot', targetId), { method: 'POST' })
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

function BuildCard({ connected, buildInfo, onRefresh, targetId }) {
  const [deployMode, setDeployMode] = useState('artifact')
  const [artifactFile, setArtifactFile] = useState(null)
  const [artifactUploading, setArtifactUploading] = useState(false)
  const [sourceFile, setSourceFile] = useState(null)
  const [sourceUploading, setSourceUploading] = useState(false)
  const [builds, setBuilds] = useState([])
  const [selectedTag, setSelectedTag] = useState('')
  const [loadingBuilds, setLoadingBuilds] = useState(false)
  const [downloading, setDownloading] = useState(false)
  const [buildResult, setBuildResult] = useState(null)

  useEffect(() => {
    setDeployMode('artifact')
    setBuildResult(null)
    setBuilds([])
    setSelectedTag('')
    setArtifactFile(null)
    setSourceFile(null)
  }, [targetId])

  useEffect(() => {
    if (!connected) {
      setBuildResult(null)
      setBuilds([])
      setSelectedTag('')
      setArtifactFile(null)
      setSourceFile(null)
    }
  }, [connected])

  const upload = async (mode) => {
    const file = mode === 'source' ? sourceFile : artifactFile
    if (!connected || !file) return
    const setUploading = mode === 'source' ? setSourceUploading : setArtifactUploading

    setUploading(true)
    setBuildResult(null)
    const fd = new FormData()
    fd.append('file', file)
    const endpoint = mode === 'source' ? '/api/builds/upload-source' : '/api/builds/upload'
    const data = await api(withTargetId(endpoint, targetId), {
      method: 'POST',
      body: withTargetFormData(fd, targetId),
    })
    setBuildResult(
      data.success && mode === 'source'
        ? { ...data, output: `Source bundle deployed: ${data.output || 'Done'}` }
        : data
    )
    setUploading(false)
    if (mode === 'source') {
      setSourceFile(null)
    } else {
      setArtifactFile(null)
    }
    onRefresh()
  }

  const listBuilds = async () => {
    if (!connected) return
    setLoadingBuilds(true)
    setBuildResult(null)
    const data = await api(withTargetId('/api/builds/list', targetId))
    if (data.success && data.builds) {
      setBuilds(data.builds)
      if (data.builds.length > 0 && !selectedTag) setSelectedTag(data.builds[0].tag)
    } else {
      setBuildResult(data)
    }
    setLoadingBuilds(false)
  }

  const selectedBuild = builds.find(build => build.tag === selectedTag) || null

  const download = async () => {
    if (!connected || !selectedTag) return
    setDownloading(true)
    setBuildResult(null)
    const fd = new FormData()
    fd.append('tag', selectedTag)
    if (selectedBuild?.source) fd.append('source', selectedBuild.source)
    if (selectedBuild?.artifact_id) fd.append('artifact_id', selectedBuild.artifact_id)
    const data = await api(withTargetId('/api/builds/download', targetId), {
      method: 'POST',
      body: withTargetFormData(fd, targetId),
    })
    setBuildResult(data)
    setDownloading(false)
    onRefresh()
  }

  const rollback = async () => {
    if (!connected) return
    setBuildResult(null)
    const data = await api(withTargetId('/api/builds/rollback', targetId), {
      method: 'POST',
      body: withTargetFormData(new FormData(), targetId),
    })
    setBuildResult(data)
    onRefresh()
  }

  return (
    <div className="card">
      <h2 className="card-title">Target Build</h2>
      <div className="card-content">
        {connected && buildInfo?.info && (
          <div className="info-box">
            <pre>{buildInfo.info}</pre>
          </div>
        )}
        {!connected && (
          <p className="subtext left-note">Connect to the target WiFi to view deployed build info and deploy updates.</p>
        )}

        <div className="mini-tabs build-tabs">
          <button className={`mini-tab ${deployMode === 'artifact' ? 'mini-tab-active' : ''}`} onClick={() => setDeployMode('artifact')}>
            Artifact Deploy
          </button>
          <button className={`mini-tab ${deployMode === 'source' ? 'mini-tab-active' : ''}`} onClick={() => setDeployMode('source')}>
            Source Build
          </button>
        </div>

        {deployMode === 'artifact' ? (
          <>
            <p className="subtext left-note build-hint">
              Deploy a release tarball from GitHub or upload a local artifact bundle. This replaces the current install on the selected target.
            </p>

            <label>Upload artifact</label>
            <div className="file-upload">
              <input
                type="file"
                accept=".tar.gz,.tgz,.tar,.gz,application/gzip,application/x-gzip,application/x-tar,application/octet-stream"
                onChange={e => setArtifactFile(e.target.files?.[0] || null)}
                id="artifact-build-file"
              />
              <label htmlFor="artifact-build-file" className="file-label">
                {artifactFile ? artifactFile.name : 'Choose .tar.gz'}
              </label>
            </div>

            <button className="btn btn-primary" onClick={() => upload('artifact')} disabled={artifactUploading || !artifactFile || !connected}>
              {artifactUploading ? 'Uploading...' : 'Upload & replace install'}
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
                      {b.source === 'actions' ? '[Actions] ' : '[Release] '}
                      {b.sha || b.tag} — {b.date}
                      {b.branch ? ` · ${b.branch}` : ''}
                      {b.size_mb ? ` · ${b.size_mb} MB` : ''}
                    </option>
                  ))}
                </select>

                <button className="btn btn-primary" onClick={download} disabled={downloading || !connected}>
                  {downloading ? 'Downloading...' : 'Download & deploy'}
                </button>
              </>
            )}
          </>
        ) : (
          <>
            <p className="subtext left-note build-hint">
              Upload a package bundle archive for the on-target build path. This requires an existing deployed base release on the target and installs through the same release contract as artifact deploys.
            </p>

            <label>Upload source bundle</label>
            <div className="file-upload">
              <input
                type="file"
                accept=".tar.gz,.tgz,.zip,application/gzip,application/x-gzip,application/zip,application/x-zip-compressed,application/octet-stream"
                onChange={e => setSourceFile(e.target.files?.[0] || null)}
                id="source-build-file"
              />
              <label htmlFor="source-build-file" className="file-label">
                {sourceFile ? sourceFile.name : 'Choose source bundle (.tar.gz or .zip)'}
              </label>
            </div>

            <button className="btn btn-primary" onClick={() => upload('source')} disabled={sourceUploading || !sourceFile || !connected}>
              {sourceUploading ? 'Uploading...' : 'Upload source bundle'}
            </button>

            <p className="subtext left-note build-hint">
              The backend installs the uploaded bundle into the same deploy root contract as artifacts, so source-built and artifact-installed releases remain interchangeable on the target.
            </p>
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

        <Result data={buildResult} />
      </div>
    </div>
  )
}

function SettingsPanel({ onRefresh, targetId, selectedTarget }) {
  const [open, setOpen] = useState(false)
  const [cfg, setCfg] = useState(null)
  const [saving, setSaving] = useState(false)
  const [inventoryFile, setInventoryFile] = useState(null)
  const [inventoryIoLoading, setInventoryIoLoading] = useState(false)
  const [result, setResult] = useState(null)

  const load = useCallback(async () => {
    const [configData, inventoryData] = await Promise.all([
      api('/api/config'),
      api('/api/inventory'),
    ])
    if (!configData.success) {
      setResult(configData)
      return
    }
    const targets = Array.isArray(inventoryData?.targets) ? inventoryData.targets : []
    const nextTarget = targets.find(target => target.target_id === targetId) || selectedTarget || null
    setCfg({
      operator: configData.config,
      target: nextTarget,
    })
  }, [targetId])

  const toggle = () => {
    setOpen(prev => {
      setResult(null)
      return !prev
    })
  }

  const update = (section, key, value) => {
    setCfg(prev => ({
      ...prev,
      [section]: {
        ...(prev?.[section] || {}),
        [key]: value,
      },
    }))
  }

  const save = async () => {
    if (!cfg) return
    setSaving(true)
    setResult(null)
    let operatorResult = { success: true }
    let targetResult = { success: true }

    if (cfg.operator) {
      const fd = new FormData()
      Object.entries(cfg.operator).forEach(([k, v]) => fd.append(k, v ?? ''))
      operatorResult = await api('/api/config', { method: 'POST', body: fd })
    }

    if (cfg.target?.target_id) {
      const fd = new FormData()
      ;[
        'target_id',
        'label',
        'pi_user',
        'pi_host',
        'deploy_root',
        'ssh_key',
        'ssh_pass',
        'fleet_file',
        'vehicle_name',
        'service_unit',
      ].forEach(key => fd.append(key, cfg.target[key] ?? ''))
      targetResult = await api('/api/inventory', { method: 'POST', body: fd })
    }

    const data = !operatorResult.success ? operatorResult : targetResult
    setResult(data.success ? { success: true, output: 'Settings saved' } : data)
    setSaving(false)
    onRefresh()
  }

  const exportInventory = async () => {
    setInventoryIoLoading(true)
    setResult(null)
    try {
      const res = await fetch('/api/inventory/export')
      const raw = await res.text()
      if (!res.ok) {
        throw new Error(raw || `HTTP ${res.status}`)
      }
      const blob = new Blob([raw], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      const anchor = document.createElement('a')
      anchor.href = url
      anchor.download = 'integration-inventory.json'
      document.body.appendChild(anchor)
      anchor.click()
      anchor.remove()
      URL.revokeObjectURL(url)
      setResult({ success: true, output: 'Inventory exported' })
    } catch (error) {
      setResult({ success: false, error: error.message || 'Inventory export failed' })
    } finally {
      setInventoryIoLoading(false)
    }
  }

  const importInventory = async () => {
    if (!inventoryFile) return
    setInventoryIoLoading(true)
    setResult(null)
    const fd = new FormData()
    fd.append('file', inventoryFile)
    const data = await api('/api/inventory/import', { method: 'POST', body: fd })
    setResult(data)
    setInventoryIoLoading(false)
    if (data.success) {
      setInventoryFile(null)
      onRefresh()
      load()
    }
  }

  useEffect(() => {
    if (open) {
      load()
    }
  }, [load, open])

  const operatorFields = [
    { key: 'github_repo', label: 'GitHub repo', placeholder: 'org/repo' },
    { key: 'github_token', label: 'GitHub token', placeholder: 'Optional token', type: 'password' },
    { key: 'hotspot_name', label: 'Hotspot connection name', placeholder: 'penn-desktop' },
    { key: 'default_deploy_root', label: 'Default deploy root', placeholder: '/home/penn/pennair-deploy' },
  ]

  const targetFields = [
    { key: 'label', label: 'Target label', placeholder: 'Payload Pi' },
    { key: 'pi_host', label: 'Pi host', placeholder: 'penn-desktop.local' },
    { key: 'pi_user', label: 'Pi user', placeholder: 'penn' },
    { key: 'ssh_pass', label: 'SSH password', placeholder: 'Leave blank for SSH key auth', type: 'password' },
    { key: 'ssh_key', label: 'SSH key path', placeholder: '~/.ssh/id_rsa' },
    { key: 'deploy_root', label: 'Deploy root', placeholder: '/home/penn/pennair-deploy' },
    { key: 'fleet_file', label: 'Fleet file', placeholder: 'src/uav/uav/fleets/example_fleet.yaml' },
    { key: 'vehicle_name', label: 'Fleet vehicle', placeholder: 'uav_0' },
    { key: 'service_unit', label: 'Systemd unit', placeholder: 'pennair-autonomy.service' },
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
            {operatorFields.map(f => (
              <div key={f.key} className="settings-field">
                <label>{f.label}</label>
                <input
                  type={f.type || 'text'}
                  value={cfg.operator?.[f.key] || ''}
                  onChange={e => update('operator', f.key, e.target.value)}
                  placeholder={f.placeholder}
                />
              </div>
            ))}
            {targetFields.map(f => (
              <div key={f.key} className="settings-field">
                <label>{f.label}</label>
                <input
                  type={f.type || 'text'}
                  value={cfg.target?.[f.key] || ''}
                  onChange={e => update('target', f.key, e.target.value)}
                  placeholder={f.placeholder}
                />
              </div>
            ))}
            <div className="settings-field settings-save">
              <button className="btn btn-primary" onClick={save} disabled={saving}>
                {saving ? 'Saving...' : 'Save'}
              </button>
            </div>
            <div className="divider" />
            <div className="settings-field">
              <label>Inventory export</label>
              <button className="btn btn-secondary" onClick={exportInventory} disabled={inventoryIoLoading}>
                {inventoryIoLoading ? 'Working...' : 'Download inventory JSON'}
              </button>
            </div>
            <div className="settings-field">
              <label>Inventory import</label>
              <div className="file-upload">
                <input
                  type="file"
                  accept=".json,application/json"
                  onChange={e => setInventoryFile(e.target.files?.[0] || null)}
                  id="inventory-file"
                />
                <label htmlFor="inventory-file" className="file-label">
                  {inventoryFile ? inventoryFile.name : 'Choose inventory JSON'}
                </label>
              </div>
              <button
                className="btn btn-secondary"
                onClick={importInventory}
                disabled={inventoryIoLoading || !inventoryFile}
              >
                {inventoryIoLoading ? 'Working...' : 'Import inventory'}
              </button>
            </div>
            <Result data={result} />
          </div>
        </div>
      )}
    </div>
  )
}

function MissionControl({ connected, buildInfo, onRefresh, workspacePaths, targetId, selectedTarget }) {
  const [paramsMode, setParamsMode] = useState('form')
  const [missionViewMode, setMissionViewMode] = useState('graph')
  const [integerDrafts, setIntegerDrafts] = useState({})
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
  } = useMissionControl({ connected, onRefresh, targetId })
  const parsedMission = useMemo(() => parseMissionDocument(missionFileText), [missionFileText])

  useEffect(() => {
    setIntegerDrafts({})
  }, [paramsText])

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

  const selectedMissionName = `${getYamlFieldValue(paramsText, 'mission', 'string') || ''}`.trim()
  const workspaceRoot = workspacePaths?.deploy_root || ''
  const launchParamsDisplayPath =
    displayWorkspacePath(workspacePaths?.overlay_file, workspaceRoot) ||
    'config/overlay.yaml'
  const missionsDirDisplayPath =
    displayWorkspacePath(workspacePaths?.missions_dir, workspaceRoot) ||
    'config/missions'
  const fleetDisplayPath =
    displayWorkspacePath(selectedTarget?.fleet_file || workspacePaths?.fleet_file, workspaceRoot) ||
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

  const renderCoreField = (field) => {
    const value = getYamlFieldValue(paramsText, field.key, field.type)

    if (field.key === 'mission') {
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
                {missionNamesLoading ? 'Loading overlay files...' : 'Select overlay file'}
              </option>
            )}
            {options.length === 0 && (
              <option value="" disabled>
                No overlay YAML files found
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
              Overlay file list unavailable from `{missionsDirDisplayPath}`: {missionNamesError}
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
          <h2 className="card-title">Current Build on Target</h2>
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
        <h2 className="card-title">Mission Overlay Config ({launchParamsDisplayPath})</h2>
        <div className="card-content">
          <div className="mini-tabs">
            <button className={`mini-tab ${paramsMode === 'form' ? 'mini-tab-active' : ''}`} onClick={() => setParamsMode('form')}>Form View</button>
            <button className={`mini-tab ${paramsMode === 'raw' ? 'mini-tab-active' : ''}`} onClick={() => setParamsMode('raw')}>Raw YAML</button>
          </div>

          {!connected && (
            <p className="subtext left-note">Connect to the target WiFi to load and edit the overlay config.</p>
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
              {paramsLoading ? 'Loading...' : 'Reload From Target'}
            </button>
            <button className="btn btn-primary" onClick={saveParams} disabled={paramsLoading || !connected}>
              {paramsLoading ? 'Saving...' : 'Save Overlay'}
            </button>
          </div>
          <p className="subtext left-note">Reload discards unsaved local edits and re-reads the file from the target.</p>
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
          Mission Editor (
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
            <p className="subtext left-note">Set the mission name in the overlay to load a mission YAML file.</p>
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

function DeployPage({ connected, sshCommand, wifiStatus, buildInfo, onRefresh, targetId }) {
  return (
    <>
      <ConnectionCard sshCommand={sshCommand} />

      <div className="grid">
        <WifiCard connected={connected} wifiStatus={wifiStatus} onRefresh={onRefresh} targetId={targetId} />
        <BuildCard connected={connected} buildInfo={buildInfo} onRefresh={onRefresh} targetId={targetId} />
      </div>
    </>
  )
}

function MissionGraphEditor({
  connected,
  setMissionFileText,
  parsedMission,
  selectedTarget,
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
  const fleetDisplayPath = selectedTarget?.fleet_file || ''

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
            Fleet source: `{fleetDisplayPath || 'No fleet file on selected target'}`
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
  const [targets, setTargets] = useState([])
  const [selectedTargetId, setSelectedTargetId] = useState('')
  const [wifiStatus, setWifiStatus] = useState(null)
  const [buildInfo, setBuildInfo] = useState(null)
  const [sshCommand, setSshCommand] = useState('')
  const [workspacePaths, setWorkspacePaths] = useState(null)
  const [inventoryResult, setInventoryResult] = useState(null)
  const [inventoryLoading, setInventoryLoading] = useState(false)
  const [targetSwitching, setTargetSwitching] = useState(false)
  const [pollError, setPollError] = useState(null)

  const refreshAll = useCallback(async () => {
    setInventoryLoading(true)
    const inventory = await api('/api/inventory')
    const normalizedTargets = uniqueTargets(inventory?.targets || [])
    const inventoryTargetId = `${inventory?.active_target_id || ''}`.trim()
    const nextTargetId = normalizedTargets.some(target => target.target_id === inventoryTargetId)
      ? inventoryTargetId
      : (normalizedTargets[0]?.target_id || inventoryTargetId || selectedTargetId)

    if (inventory.success) {
      setTargets(normalizedTargets)
      setSelectedTargetId(nextTargetId)
      setInventoryResult(null)
    } else {
      setInventoryResult(inventory)
    }

    const effectiveTargetId = nextTargetId || selectedTargetId
    const targetUrl = url => withTargetId(url, effectiveTargetId)

    const conn = await api(targetUrl('/api/connection/status'))
    const sshPromise = api(targetUrl('/api/connection/ssh-command'))
    const configPromise = api(targetUrl('/api/config'))

    const isConnected = Boolean(conn?.connected)
    setConnected(isConnected)

    if (!isConnected) {
      const [ssh, config] = await Promise.all([sshPromise, configPromise])
      if (ssh.success) setSshCommand(ssh.command)
      if (config.success) setWorkspacePaths(config.workspace_paths || null)
      setWifiStatus(null)
      setBuildInfo(null)
      setPollError(conn?.error ? { success: false, error: conn.error } : null)
      setInventoryLoading(false)
      return
    }

    const [wifi, build, ssh, config] = await Promise.all([
      api(targetUrl('/api/wifi/status')),
      api(targetUrl('/api/builds/current')),
      sshPromise,
      configPromise,
    ])

    setWifiStatus(wifi.success ? wifi : null)
    setBuildInfo(build.success ? build : null)
    if (ssh.success) setSshCommand(ssh.command)
    if (config.success) setWorkspacePaths(config.workspace_paths || null)

    const err = (!wifi.success ? wifi?.error : null) || (!build.success ? build?.error : null) || null
    setPollError(err ? { success: false, error: err } : null)
    setInventoryLoading(false)
  }, [selectedTargetId])

  useEffect(() => {
    refreshAll()
    const interval = setInterval(refreshAll, 5000)
    return () => clearInterval(interval)
  }, [refreshAll])

  const selectedTarget = targets.find(target => target.target_id === selectedTargetId) || null

  const handleTargetChange = useCallback(async (nextTargetId) => {
    if (!nextTargetId || nextTargetId === selectedTargetId) return
    setTargetSwitching(true)
    setInventoryResult(null)
    const fd = withTargetFormData(new FormData(), nextTargetId)
    const data = await api('/api/inventory/active', { method: 'POST', body: fd })
    if (data.success) {
      setSelectedTargetId(nextTargetId)
      await refreshAll()
    } else {
      setInventoryResult(data)
    }
    setTargetSwitching(false)
  }, [refreshAll, selectedTargetId])

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
      <SettingsPanel onRefresh={refreshAll} targetId={selectedTargetId} selectedTarget={selectedTarget} />

      <h1 className="title">PennAiR Auton Deploy</h1>
      <StatusBar connected={connected} wifiStatus={wifiStatus} buildInfo={buildInfo} selectedTarget={selectedTarget} />
      <Result data={pollError} />
      <Result data={inventoryResult} />

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
        <TargetSelector
          connected={connected}
          targets={targets}
          selectedTargetId={selectedTargetId}
          onChange={handleTargetChange}
          loading={inventoryLoading || targetSwitching}
        />
        <button className="theme-toggle-btn" type="button" onClick={toggleTheme}>
          {theme === THEME_DARK ? 'Light Mode' : 'Dark Mode'}
        </button>
      </div>

      {page === 'mission' ? (
        <MissionControl
          connected={connected}
          buildInfo={buildInfo}
          onRefresh={refreshAll}
          workspacePaths={workspacePaths}
          targetId={selectedTargetId}
          selectedTarget={selectedTarget}
        />
      ) : (
        <DeployPage connected={connected} sshCommand={sshCommand} wifiStatus={wifiStatus} buildInfo={buildInfo} onRefresh={refreshAll} targetId={selectedTargetId} />
      )}
    </div>
  )
}

export default App
