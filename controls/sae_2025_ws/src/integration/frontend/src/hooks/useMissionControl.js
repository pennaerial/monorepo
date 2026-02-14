import { useCallback, useEffect, useRef, useState } from 'react'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'

import { api } from '../services/api'

const MAX_TERMINAL_BUFFER_CHARS = 1_000_000

function trimTerminalBuffer(text) {
  if (text.length <= MAX_TERMINAL_BUFFER_CHARS) return text
  return text.slice(text.length - MAX_TERMINAL_BUFFER_CHARS)
}

function offlineMissionState() {
  return {
    phase: 'offline',
    launch_state: 'offline',
    running: false,
    pid: null,
    message: 'Pi offline',
    error: null,
    updated_at: Date.now() / 1000,
  }
}

function initialMissionState(connected) {
  if (!connected) return offlineMissionState()
  return {
    phase: 'idle',
    launch_state: 'not_prepared',
    running: false,
    pid: null,
    message: null,
    error: null,
    updated_at: Date.now() / 1000,
  }
}

export function useMissionControl({ connected, onRefresh }) {
  const [actionLoading, setActionLoading] = useState('')
  const [actionResult, setActionResult] = useState(null)

  const [missionState, setMissionState] = useState(initialMissionState(connected))
  const [logsResult, setLogsResult] = useState(null)
  const [streamConnected, setStreamConnected] = useState(false)

  const [paramsText, setParamsText] = useState('')
  const [paramsLoading, setParamsLoading] = useState(false)
  const [paramsResult, setParamsResult] = useState(null)

  const terminalHostRef = useRef(null)
  const terminalRef = useRef(null)
  const fitAddonRef = useRef(null)
  const terminalBufferRef = useRef('')

  const statusInFlightRef = useRef(false)
  const streamRef = useRef(null)
  const streamActiveRef = useRef(false)
  const streamReconnectTimerRef = useRef(null)
  const hasLoadedLogsRef = useRef(false)
  const logCursorRef = useRef({ offset: 0, inode: 0 })

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

    terminalRef.current = term
    fitAddonRef.current = fitAddon

    if (terminalBufferRef.current) {
      term.write(terminalBufferRef.current)
    }

    const onResize = () => fitAddonRef.current?.fit()
    window.addEventListener('resize', onResize)

    return () => {
      window.removeEventListener('resize', onResize)
      fitAddonRef.current = null
      terminalRef.current = null
      term.dispose()
    }
  }, [])

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
      // Ignore close failures during cleanup.
    }

    streamRef.current = null
    setStreamConnected(false)
  }, [])

  const openTerminalStream = useCallback(() => {
    if (streamRef.current || !connected) return

    const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
    const cursor = logCursorRef.current
    const url = `${proto}://${window.location.host}/ws/mission/terminal?offset=${cursor.offset}&inode=${cursor.inode}`
    const ws = new WebSocket(url)

    streamRef.current = ws

    ws.onopen = () => {
      setStreamConnected(true)
      setLogsResult(null)
    }

    ws.onmessage = event => {
      if (typeof event.data !== 'string') return

      let parsed = null
      try {
        parsed = JSON.parse(event.data)
      } catch {
        appendTerminal(event.data)
        return
      }

      if (parsed?.type === 'chunk') {
        const nextOffset = Number(parsed.next_offset ?? logCursorRef.current.offset)
        const nextInode = Number(parsed.inode ?? logCursorRef.current.inode)
        const reset = Boolean(parsed.reset)
        const data = typeof parsed.data === 'string' ? parsed.data : ''

        if (reset) {
          resetTerminal(data)
        } else {
          appendTerminal(data)
        }

        logCursorRef.current = {
          offset: Number.isFinite(nextOffset) ? nextOffset : logCursorRef.current.offset,
          inode: Number.isFinite(nextInode) ? nextInode : logCursorRef.current.inode,
        }
        return
      }

      if (parsed?.type === 'error') {
        setLogsResult({ success: false, error: parsed.message || 'Live terminal stream error' })
        return
      }

      if (parsed?.type === 'info') {
        return
      }

      appendTerminal(event.data)
    }

    ws.onerror = () => {
      setLogsResult({ success: false, error: 'Live SSH terminal stream encountered an error.' })
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
  }, [appendTerminal, connected, resetTerminal])

  const loadFullLogs = useCallback(async () => {
    if (!connected) {
      setLogsResult(null)
      return false
    }

    const data = await api('/api/mission/launch/logs?offset=0&inode=0')
    if (data.success) {
      const text = data.logs || ''
      resetTerminal(text)
      hasLoadedLogsRef.current = true
      logCursorRef.current = {
        offset: Number(data.next_offset || 0),
        inode: Number(data.inode || 0),
      }
      setLogsResult(null)
      return true
    }

    setLogsResult(data)
    return false
  }, [connected, resetTerminal])

  const refreshMissionState = useCallback(async (forceFullLogs = false) => {
    if (!connected) {
      streamActiveRef.current = false
      closeTerminalStream()
      setMissionState(offlineMissionState())
      setLogsResult(null)
      setStreamConnected(false)
      return
    }

    if (statusInFlightRef.current) return
    statusInFlightRef.current = true

    try {
      const stateRes = await api('/api/mission/state')
      if (!stateRes.success || !stateRes.state) {
        const fallbackError = stateRes?.error || 'Failed to refresh mission state'
        streamActiveRef.current = false
        closeTerminalStream()
        setMissionState({
          ...offlineMissionState(),
          phase: 'error',
          launch_state: 'error',
          error: fallbackError,
        })
        setLogsResult({ success: false, error: fallbackError })
        return
      }

      const nextState = stateRes.state
      setMissionState(nextState)

      if (nextState.launch_state === 'running') {
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
  }, [closeTerminalStream, connected, loadFullLogs, openTerminalStream])

  const refreshLaunchData = useCallback(async (forceFullLogs = false) => {
    await refreshMissionState(forceFullLogs)
  }, [refreshMissionState])

  const loadParams = useCallback(async () => {
    if (!connected) {
      setParamsResult(null)
      return
    }

    setParamsLoading(true)
    setParamsResult(null)

    const data = await api('/api/mission/launch-params')
    if (data.success) {
      setParamsText(data.content || '')
    } else {
      setParamsResult(data)
    }

    setParamsLoading(false)
  }, [connected])

  const saveParams = useCallback(async () => {
    if (!connected) {
      setParamsResult({ success: false, error: 'Connect to the Pi WiFi before editing launch params.' })
      return
    }

    setParamsLoading(true)
    setParamsResult(null)

    const fd = new FormData()
    fd.append('content', paramsText)

    const data = await api('/api/mission/launch-params', { method: 'POST', body: fd })
    setParamsResult(data)
    setParamsLoading(false)
  }, [connected, paramsText])

  const runAction = useCallback(async (name, url) => {
    if (!connected) {
      setActionResult({ success: false, error: 'Connect to the Pi WiFi before running mission actions.' })
      return
    }

    setActionLoading(name)
    setActionResult(null)

    if (name === 'prepare') {
      resetTerminal('')
      hasLoadedLogsRef.current = false
      logCursorRef.current = { offset: 0, inode: 0 }
    }

    const data = await api(url, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')

    await refreshLaunchData(true)
    await onRefresh()
  }, [connected, onRefresh, refreshLaunchData, resetTerminal])

  useEffect(() => {
    if (!connected) {
      streamActiveRef.current = false
      closeTerminalStream()
      setMissionState(offlineMissionState())
      setLogsResult(null)
      setParamsResult(null)
      setActionResult(null)
      setActionLoading('')
      setStreamConnected(false)
      hasLoadedLogsRef.current = false
      logCursorRef.current = { offset: 0, inode: 0 }
      resetTerminal('Connect to the Pi WiFi to view launch output.\r\n')
      return undefined
    }

    loadFullLogs()
    refreshMissionState(false)
    loadParams()

    const interval = setInterval(() => {
      refreshMissionState(false)
    }, 500)

    return () => {
      clearInterval(interval)
      streamActiveRef.current = false
      closeTerminalStream()
    }
  }, [closeTerminalStream, connected, loadFullLogs, loadParams, refreshMissionState, resetTerminal])

  return {
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
    setParamsResult,
    loadParams,
    saveParams,
    runAction,
    refreshLaunchData,
  }
}
