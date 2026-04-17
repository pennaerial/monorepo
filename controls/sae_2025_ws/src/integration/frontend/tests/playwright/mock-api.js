import {
  delay,
  jsonResponse,
  makeConfigResponse,
  makeEmptyBuildSource,
  makeFleetFileResponse,
  makeMissionCatalogResponse,
  makeFleetSchemaResponse,
  makeGithubBuildSource,
  makeLocalArtifactBuildSource,
  makeLocalCodebaseBuildSource,
} from './fixtures.js'

async function fulfill(route, body, status = 200) {
  await route.fulfill(jsonResponse(body, status))
}

async function resolveValue(value, context) {
  const resolved = typeof value === 'function' ? await value(context) : value
  return resolved === undefined ? context.fallback : resolved
}

function makeFleetActionResponse(action, overrides = {}) {
  const message = `${action} completed`
  return {
    success: true,
    output: message,
    summary: {
      total_devices: 1,
      failed_devices: 0,
      succeeded_devices: 1,
    },
    results: [
      {
        success: true,
        output: message,
      },
    ],
    ...overrides,
  }
}

export async function installApiMocks(page, scenario = {}) {
  const state = {
    buildSourceGetCount: 0,
    boardGetCount: 0,
    liveGetCount: 0,
    buildListGetCount: 0,
    inventoryGetCount: 0,
    connectionStatusGetCount: 0,
    sshCommandGetCount: 0,
    wifiStatusGetCount: 0,
    buildCurrentGetCount: 0,
    missionStateGetCount: 0,
    missionNamesGetCount: 0,
    schemaGetCount: 0,
    fleetSchemaGetCount: 0,
    missionCatalogGetCount: 0,
    missionFileGetCount: 0,
    fleetFileGetCount: 0,
    launchLogsGetCount: 0,
    selectedBuildSource: scenario.initialBuildSource || makeEmptyBuildSource(),
    fleetFileContent: scenario.initialFleetFileContent || makeFleetFileResponse().content,
  }

  await page.route('**/api/**', async route => {
    const request = route.request()
    const { pathname } = new URL(request.url())
    const method = request.method()

    const context = {
      route,
      request,
      method,
      pathname,
      state,
      fallback: makeEmptyBuildSource(),
    }

    if (pathname === '/api/config' && method === 'GET') {
      const response = await resolveValue(scenario.config || makeConfigResponse(), context)
      return fulfill(route, response)
    }

    if (pathname === '/api/inventory' && method === 'GET') {
      const index = state.inventoryGetCount++
      const response = await resolveValue(
        scenario.inventoryGet || {
          success: true,
          targets: [],
          active_target_id: '',
          request_index: index,
        },
        { ...context, index, fallback: { success: true, targets: [], active_target_id: '' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/fleet/board' && method === 'GET') {
      const index = state.boardGetCount++
      const response = await resolveValue(
        scenario.boardGet || {
          success: true,
          devices: [],
          targets: [],
          build_source: makeEmptyBuildSource(),
          request_index: index,
        },
        { ...context, index, fallback: { success: true, devices: [], targets: [], build_source: makeEmptyBuildSource() } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/hardware/live' && method === 'GET') {
      const index = state.liveGetCount++
      const response = await resolveValue(
        scenario.liveGet || {
          success: true,
          devices: [],
          request_index: index,
        },
        { ...context, index, fallback: { success: true, devices: [] } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source' && method === 'GET') {
      const index = state.buildSourceGetCount++
      const response = await resolveValue(
        scenario.buildSourceGet || (async ({ state: mockState }) => mockState.selectedBuildSource),
        { ...context, index, fallback: state.selectedBuildSource || makeEmptyBuildSource() }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/builds/list' && method === 'GET') {
      const index = state.buildListGetCount++
      const response = await resolveValue(
        scenario.buildListGet || scenario.buildList || {
          success: true,
          releases: [],
          artifacts: [],
          request_index: index,
        },
        { ...context, index, fallback: { success: true, releases: [], artifacts: [] } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/github' && method === 'POST') {
      const response = await resolveValue(
        scenario.buildSourcePostGithub || makeGithubBuildSource(scenario.selectedGithubBuild || state.selectedBuildSource.build || null, scenario.selectedGithubBuildSource || {}),
        { ...context, fallback: state.selectedBuildSource }
      )
      state.selectedBuildSource = response
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/fleet' && method === 'POST') {
      const response = await resolveValue(
        scenario.buildSourcePostFleet || (({ state: mockState }) => {
          const next = { ...(mockState.selectedBuildSource || makeEmptyBuildSource()) }
          next.fleet_file = scenario.selectedFleetFile || next.fleet_file || ''
          next.fleet_options = Array.isArray(next.fleet_options) ? next.fleet_options : []
          return next
        }),
        { ...context, fallback: state.selectedBuildSource }
      )
      state.selectedBuildSource = response
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/local-artifact' && method === 'POST') {
      const response = await resolveValue(
        scenario.buildSourcePostLocalArtifact || makeLocalArtifactBuildSource(),
        { ...context, fallback: state.selectedBuildSource }
      )
      state.selectedBuildSource = response
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/local-codebase' && method === 'POST') {
      const response = await resolveValue(
        scenario.buildSourcePostLocalCodebase || makeLocalCodebaseBuildSource(),
        { ...context, fallback: state.selectedBuildSource }
      )
      state.selectedBuildSource = response
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/fleet-file' && method === 'GET') {
      const index = state.fleetFileGetCount++
      const response = await resolveValue(
        scenario.fleetFileGet || (({ state: mockState }) => makeFleetFileResponse({ content: mockState.fleetFileContent })),
        { ...context, index, fallback: makeFleetFileResponse({ content: state.fleetFileContent }) }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/fleet-file' && method === 'POST') {
      const postData = await request.postDataBuffer()
      const rawText = postData ? postData.toString('utf-8') : ''
      const urlEncodedMatch = rawText.match(/(?:^|&)content=([^&]*)/)
      const multipartMatch = rawText.match(/name="content"\r\n\r\n([\s\S]*?)\r\n--/)
      if (urlEncodedMatch) {
        state.fleetFileContent = decodeURIComponent(urlEncodedMatch[1].replace(/\+/g, ' '))
      } else if (multipartMatch) {
        state.fleetFileContent = multipartMatch[1]
      }
      const response = await resolveValue(
        scenario.fleetFilePost || {
          success: true,
          fleet_file: state.selectedBuildSource?.fleet_file || 'payload_corner_navigation.yaml',
          output: 'Fleet YAML updated',
        },
        { ...context, fallback: { success: true, fleet_file: state.selectedBuildSource?.fleet_file || 'payload_corner_navigation.yaml', output: 'Fleet YAML updated' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/build-source/clear' && method === 'POST') {
      state.selectedBuildSource = makeEmptyBuildSource()
      return fulfill(route, { success: true, kind: 'none', fleet_options: [] })
    }

    if (pathname === '/api/connection/status' && method === 'GET') {
      const index = state.connectionStatusGetCount++
      const response = await resolveValue(
        scenario.connectionStatusGet || {
          success: true,
          connected: true,
          hostname: 'air-01.local',
        },
        { ...context, index, fallback: { success: true, connected: true, hostname: 'air-01.local' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/connection/ssh-command' && method === 'GET') {
      const index = state.sshCommandGetCount++
      const response = await resolveValue(
        scenario.sshCommandGet || {
          success: true,
          command: 'ssh penn@air-01.local',
        },
        { ...context, index, fallback: { success: true, command: 'ssh penn@air-01.local' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/wifi/status' && method === 'GET') {
      const index = state.wifiStatusGetCount++
      const response = await resolveValue(
        scenario.wifiStatusGet || {
          success: true,
          connected: true,
          ssid: 'air-01.local',
        },
        { ...context, index, fallback: { success: true, connected: true, ssid: 'air-01.local' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/builds/current' && method === 'GET') {
      const index = state.buildCurrentGetCount++
      const response = await resolveValue(
        scenario.buildCurrentGet || {
          success: true,
          installed: true,
          info: 'Current build: release-ready',
        },
        { ...context, index, fallback: { success: true, installed: true, info: 'Current build: release-ready' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/state' && method === 'GET') {
      const index = state.missionStateGetCount++
      const response = await resolveValue(
        scenario.missionStateGet || {
          success: true,
          state: {
            phase: 'idle',
            launch_state: 'not_prepared',
            running: false,
            pid: null,
            message: null,
            error: null,
            updated_at: '2026-04-13T12:34:56Z',
          },
        },
        {
          ...context,
          index,
          fallback: {
            success: true,
            state: {
              phase: 'idle',
              launch_state: 'not_prepared',
              running: false,
              pid: null,
              message: null,
              error: null,
              updated_at: '2026-04-13T12:34:56Z',
            },
          },
        }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/mission-names' && method === 'GET') {
      const index = state.missionNamesGetCount++
      const response = await resolveValue(
        scenario.missionNamesGet || {
          success: true,
          missions: ['hover'],
        },
        { ...context, index, fallback: { success: true, missions: ['hover'] } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/schema' && method === 'GET') {
      const index = state.schemaGetCount++
      const response = await resolveValue(
        scenario.schemaGet || {
          success: true,
          mode_registry: {},
          mission_document_schema: {},
          fleet_document_schema: {},
        },
        {
          ...context,
          index,
          fallback: {
            success: true,
            mode_registry: {},
            mission_document_schema: {},
            fleet_document_schema: {},
          },
        }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/schema/fleet' && method === 'GET') {
      const index = state.fleetSchemaGetCount++
      const response = await resolveValue(
        scenario.fleetSchemaGet || makeFleetSchemaResponse(),
        { ...context, index, fallback: makeFleetSchemaResponse() }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/schema/missions' && method === 'GET') {
      const index = state.missionCatalogGetCount++
      const response = await resolveValue(
        scenario.missionCatalogGet || makeMissionCatalogResponse(),
        { ...context, index, fallback: makeMissionCatalogResponse() }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/mission-file' && method === 'GET') {
      const index = state.missionFileGetCount++
      const response = await resolveValue(
        scenario.missionFileGet || {
          success: true,
          content: 'modes:\n  hover:\n    mode: uav.HoverMode\n',
        },
        {
          ...context,
          index,
          fallback: {
            success: true,
            content: 'modes:\n  hover:\n    mode: uav.HoverMode\n',
          },
        }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/launch/logs' && method === 'GET') {
      const index = state.launchLogsGetCount++
      const response = await resolveValue(
        scenario.launchLogsGet || {
          success: true,
          logs: '',
          next_offset: 0,
          inode: 0,
        },
        { ...context, index, fallback: { success: true, logs: '', next_offset: 0, inode: 0 } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/launch-params' && method === 'POST') {
      const response = await resolveValue(
        scenario.launchParamsPost || {
          success: true,
          output: 'Launch params saved',
        },
        { ...context, fallback: { success: true, output: 'Launch params saved' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/launch-params' && method === 'GET') {
      const response = await resolveValue(
        scenario.launchParamsGet || {
          success: true,
          content: 'mission: hover\nmodes:\n  hover:\n    mode: uav.HoverMode\n',
        },
        {
          ...context,
          fallback: {
            success: true,
            content: 'mission: hover\nmodes:\n  hover:\n    mode: uav.HoverMode\n',
          },
        }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/mission/mission-file' && method === 'POST') {
      const response = await resolveValue(
        scenario.missionFilePost || {
          success: true,
          output: 'Mission file saved',
        },
        { ...context, fallback: { success: true, output: 'Mission file saved' } }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/fleet/deploy' && method === 'POST') {
      const response = await resolveValue(
        scenario.fleetDeploy || makeFleetActionResponse('deploy'),
        { ...context, fallback: makeFleetActionResponse('deploy') }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/fleet/prepare' && method === 'POST') {
      const response = await resolveValue(
        scenario.fleetPrepare || makeFleetActionResponse('prepare'),
        { ...context, fallback: makeFleetActionResponse('prepare') }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/fleet/start' && method === 'POST') {
      const response = await resolveValue(
        scenario.fleetStart || makeFleetActionResponse('start'),
        { ...context, fallback: makeFleetActionResponse('start') }
      )
      return fulfill(route, response)
    }

    if (pathname === '/api/fleet/stop' && method === 'POST') {
      const response = await resolveValue(
        scenario.fleetStop || makeFleetActionResponse('stop'),
        { ...context, fallback: makeFleetActionResponse('stop') }
      )
      return fulfill(route, response)
    }

    return fulfill(route, { success: false, error: `Unhandled mock route: ${method} ${pathname}` }, 404)
  })

  return state
}

export { delay }
