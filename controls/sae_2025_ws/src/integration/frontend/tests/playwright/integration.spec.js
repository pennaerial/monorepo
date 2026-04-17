import { expect, test } from '@playwright/test'
import {
  delay,
  makeActionsBuild,
  makeBuildListResponse,
  makeConfigResponse,
  makeEmptyBuildSource,
  makeFleetFileResponse,
  makeMissionCatalogResponse,
  makeFleetOptions,
  makeFleetSchemaResponse,
  makeGithubBuildSource,
  makeLocalArtifactBuildSource,
  makeLocalCodebaseBuildSource,
  makeLiveDevice,
  makeReleaseBuild,
} from './fixtures.js'
import { installApiMocks } from './mock-api.js'

async function openSetupPage(page) {
  await page.goto('/')
  await expect(page.getByRole('heading', { name: 'Choose the build and fleet' })).toBeVisible()
}

async function loadBuildCatalog(page) {
  await page.getByRole('button', { name: 'Refresh' }).click()
  await expect(page.getByPlaceholder('Search SHA, title, branch')).toBeVisible()
}

async function openLaunchMonitorForPayload(page, {
  buildBannerText = 'Manual merge build for flight stack',
  hostname = 'air-01.local',
} = {}) {
  await openSetupPage(page)
  await expect(page.locator('.build-source-banner')).toContainText(buildBannerText)
  await page.getByRole('button', { name: 'Readiness' }).click()
  await expect(page.getByRole('heading', { name: 'Boarded devices' })).toBeVisible()
  const hardwareCard = page.locator('.hardware-card').filter({ hasText: hostname })
  await expect(hardwareCard).toBeVisible()
  await hardwareCard.click()
  await expect(page.getByRole('heading', { name: 'Launch Prep' })).toBeVisible()

  const assignmentCard = page.locator('.card').filter({ hasText: 'Launch Prep' }).first()
  await assignmentCard.locator('select').selectOption('payload_0')
  await expect(assignmentCard.getByRole('button', { name: 'Deploy Fleet Context' })).toBeEnabled()

  return {
    assignmentCard,
    missionActionsCard: page.locator('.card').filter({ hasText: 'Mission Actions' }).first(),
  }
}

function makePayloadCornerBuildSource(overrides = {}) {
  return makeGithubBuildSource(makeActionsBuild(), {
    fleet_file: 'payload_corner_navigation.yaml',
    fleet_options: makeFleetOptions(['payload_corner_navigation.yaml']),
    fleet_vehicles: [
      {
        name: 'payload_0',
        kind: 'payload',
        mission: 'payload_corner_navigation',
        mission_path: 'src/uav/uav/fleets/payload_corner_navigation.yaml',
      },
    ],
    ...overrides,
  })
}

function makePayloadCornerLocalArtifactSource(overrides = {}) {
  return makeLocalArtifactBuildSource({
    fleet_file: 'payload_corner_navigation.yaml',
    fleet_options: makeFleetOptions(['payload_corner_navigation.yaml']),
    fleet_vehicles: [
      {
        name: 'payload_0',
        kind: 'payload',
        mission: 'payload_corner_navigation',
        mission_path: 'src/uav/uav/fleets/payload_corner_navigation.yaml',
      },
    ],
    ...overrides,
  })
}

function makePayloadCornerLocalCodebaseSource(overrides = {}) {
  return makeLocalCodebaseBuildSource({
    fleet_file: 'payload_corner_navigation.yaml',
    fleet_options: makeFleetOptions(['payload_corner_navigation.yaml']),
    fleet_vehicles: [
      {
        name: 'payload_0',
        kind: 'payload',
        mission: 'payload_corner_navigation',
        mission_path: 'src/uav/uav/fleets/payload_corner_navigation.yaml',
      },
    ],
    ...overrides,
  })
}

test('renders build source promptly even when fleet board is slow', async ({ page }) => {
  const build = makeActionsBuild()
  await installApiMocks(page, {
    config: makeConfigResponse(),
    buildSourceGet: makeGithubBuildSource(build, {
      fleet_file: 'fleet-alpha.yaml',
      fleet_options: makeFleetOptions(['fleet-alpha.yaml']),
    }),
    boardGet: async () => {
      await delay(2200)
      return {
        success: true,
        devices: [],
        targets: [],
        build_source: makeEmptyBuildSource(),
      }
    },
  })

  await openSetupPage(page)

  await expect(page.locator('.build-source-banner')).toContainText('Manual merge build for flight stack')
  await expect(page.locator('.build-source-banner')).not.toContainText('Loading build and fleet...')
})

test('keeps a selected GitHub build after a delayed stale refresh', async ({ page }) => {
  const build = makeActionsBuild()
  await installApiMocks(page, {
    config: makeConfigResponse(),
    boardGet: { success: false, error: 'board unavailable' },
    inventoryGet: { success: true, targets: [], active_target_id: '' },
    liveGet: { success: true, devices: [] },
    buildList: makeBuildListResponse({
      releases: [makeReleaseBuild()],
      artifacts: [build],
    }),
    buildSourceGet: async ({ index, state }) => {
      if (index === 0) {
        await delay(2200)
        return makeEmptyBuildSource()
      }
      return state.selectedBuildSource
    },
    buildSourcePostGithub: async ({ state }) => {
      const response = makeGithubBuildSource(build, {
        fleet_file: 'fleet-alpha.yaml',
        fleet_options: makeFleetOptions(['fleet-alpha.yaml']),
        output: 'GitHub build source selected',
      })
      state.selectedBuildSource = response
      return response
    },
  })

  await openSetupPage(page)
  await loadBuildCatalog(page)

  await page.getByPlaceholder('Search SHA, title, branch').fill('manual-merge')
  await expect(page.getByText('GitHub Actions run 24327571117')).toBeVisible()

  await page.locator('.build-row').first().getByRole('button', { name: 'Use' }).click()
  await expect(page.locator('.build-source-banner')).toContainText('Manual merge build for flight stack')
  await expect(page.locator('.build-source-banner')).toContainText('Actions Artifact')

  await page.waitForTimeout(2300)
  await expect(page.locator('.build-source-banner')).toContainText('Manual merge build for flight stack')
  await expect(page.locator('.build-source-banner')).toContainText('fleet-alpha.yaml')
  await expect(page.locator('.build-source-banner')).not.toContainText('No build and fleet selected')
})

test('renders a useful Actions metadata fallback when workflow name is missing', async ({ page }) => {
  const build = makeActionsBuild()
  await installApiMocks(page, {
    config: makeConfigResponse(),
    boardGet: {
      success: true,
      devices: [],
      targets: [],
      build_source: makeEmptyBuildSource(),
    },
    buildList: makeBuildListResponse({
      releases: [makeReleaseBuild()],
      artifacts: [build],
    }),
  })

  await openSetupPage(page)
  await loadBuildCatalog(page)

  await page.getByPlaceholder('Search SHA, title, branch').fill('manual-merge')
  await expect(page.getByText('GitHub Actions run 24327571117')).toBeVisible()
  await expect(page.getByText('Workflow metadata unavailable')).toHaveCount(0)
})

test('shows a fleet catalog warning when the selected build has no fleet options', async ({ page }) => {
  const build = makeActionsBuild()
  await installApiMocks(page, {
    config: makeConfigResponse(),
    buildSourceGet: makeGithubBuildSource(build, {
      fleet_file: '',
      fleet_options: [],
      fleet_catalog_error: 'GitHub rate limit exceeded while extracting fleet catalog.',
    }),
    boardGet: {
      success: true,
      devices: [],
      targets: [],
      build_source: makeEmptyBuildSource(),
    },
  })

  await openSetupPage(page)

  await expect(page.locator('.card').filter({ hasText: 'Fleet catalog' }).locator('.param-help')).toContainText('GitHub rate limit exceeded while extracting fleet catalog.')
  await expect(page.getByRole('button', { name: 'Save Fleet' })).toBeDisabled()
})

test('allows saving a fleet selection from the current build catalog', async ({ page }) => {
  const build = makeActionsBuild()
  const fleetOptions = makeFleetOptions([
    { value: 'fleet-alpha.yaml', label: 'Fleet Alpha', description: 'Primary fleet' },
    { value: 'fleet-beta.yaml', label: 'Fleet Beta', description: 'Backup fleet' },
  ])

  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makeGithubBuildSource(build, {
      fleet_file: '',
      fleet_options: fleetOptions,
    }),
    boardGet: {
      success: true,
      devices: [],
      targets: [],
      build_source: makeEmptyBuildSource(),
    },
    buildSourcePostFleet: async ({ state }) => {
      const response = makeGithubBuildSource(build, {
        fleet_file: 'fleet-alpha.yaml',
        fleet_options: fleetOptions,
        output: 'Fleet saved',
      })
      state.selectedBuildSource = response
      return response
    },
  })

  await openSetupPage(page)

  const fleetSelect = page.locator('select').first()
  await expect(fleetSelect).toHaveValue('')
  await fleetSelect.selectOption('fleet-alpha.yaml')
  await page.getByRole('button', { name: 'Save Fleet' }).click()

  await expect(page.locator('.build-source-banner')).toContainText('fleet-alpha.yaml')
  await expect(page.getByText('Fleet saved')).toBeVisible()
})

test('loads the fleet YAML editor and saves structured fleet edits from setup', async ({ page }) => {
  let savedFleetContent = ''

  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makePayloadCornerLocalArtifactSource(),
    boardGet: {
      success: true,
      devices: [],
      targets: [],
      build_source: makePayloadCornerLocalArtifactSource(),
    },
    fleetSchemaGet: makeFleetSchemaResponse(),
    missionCatalogGet: makeMissionCatalogResponse(),
    fleetFileGet: ({ state }) => makeFleetFileResponse({
      content: state.fleetFileContent,
      path: '/tmp/payload_corner_navigation.yaml',
    }),
    fleetFilePost: ({ state }) => {
      savedFleetContent = state.fleetFileContent
      return {
        success: true,
        fleet_file: 'payload_corner_navigation.yaml',
        output: 'Fleet YAML updated',
      }
    },
  })

  await openSetupPage(page)
  await expect(page.getByRole('heading', { name: 'Fleet YAML' })).toBeVisible()
  await expect(page.locator('.fleet-editor-summary')).toContainText('payload_corner_navigation.yaml')
  await expect(page.locator('.build-source-banner')).toContainText('Local Artifact')
  await expect(page.locator('.fleet-editor')).not.toContainText('world_name')
  await expect(page.locator('.fleet-editor')).not.toContainText('mission_stage')
  await expect(page.locator('.fleet-editor')).not.toContainText('payload_controller')
  await expect(page.getByLabel('camera_calibration_file')).toHaveValue('payload_0_camera_info.yaml')

  const vehicleCard = page.locator('.fleet-vehicle-card').first()
  await expect(vehicleCard).toContainText('payload_0')
  await expect(vehicleCard.getByLabel('mission')).toHaveValue('payload_corner_navigation')
  await vehicleCard.getByLabel('mission').selectOption('payload_retreat')
  await page.getByRole('button', { name: 'Save Fleet YAML' }).click()

  await expect(page.getByText('Fleet YAML updated')).toBeVisible()
  await page.getByRole('button', { name: 'Raw YAML' }).click()
  await expect(page.locator('.fleet-raw-editor')).toContainText('kind: hardware')
  await expect(page.locator('.fleet-raw-editor')).toContainText('mission: payload_retreat')
  await expect(page.locator('.fleet-raw-editor')).not.toContainText('world_name')
  await expect(page.locator('.fleet-raw-editor')).not.toContainText('mission_stage')
  await expect(page.locator('.fleet-raw-editor')).not.toContainText('payload_controller')
  expect(savedFleetContent).toContain('kind: hardware')
  expect(savedFleetContent).toContain('mission: payload_retreat')
  expect(savedFleetContent).not.toContain('world_name')
  expect(savedFleetContent).not.toContain('mission_stage')
  expect(savedFleetContent).not.toContain('payload_controller')
})

test('preserves an unsaved fleet draft while setup polling refreshes the saved source', async ({ page }) => {
  const savedSource = makePayloadCornerLocalArtifactSource({
    fleet_file: 'payload_peer_fleet_test_dual.yaml',
    fleet_options: makeFleetOptions([
      'payload_corner_navigation.yaml',
      'payload_peer_fleet_test_dual.yaml',
    ]),
  })

  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: savedSource,
    boardGet: {
      success: true,
      devices: [],
      targets: [],
      build_source: savedSource,
    },
  })

  await openSetupPage(page)
  const fleetSelect = page.locator('select').first()
  await fleetSelect.selectOption('payload_corner_navigation.yaml')
  await expect(fleetSelect).toHaveValue('payload_corner_navigation.yaml')

  await page.waitForTimeout(5200)
  await expect(fleetSelect).toHaveValue('payload_corner_navigation.yaml')
})

test('supports local artifact upload through the browser and carries the flow into air-02 launch prep', async ({ page }) => {
  const localArtifactSource = makePayloadCornerLocalArtifactSource({ fleet_file: '' })
  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makeEmptyBuildSource(),
    boardGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
      targets: [],
      build_source: localArtifactSource,
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
    },
    buildSourcePostLocalArtifact: localArtifactSource,
    buildSourcePostFleet: makePayloadCornerLocalArtifactSource(),
  })

  await openSetupPage(page)
  await page.locator('#global-artifact-file').setInputFiles({
    name: 'ros2-build-35948f0.tar.gz',
    mimeType: 'application/gzip',
    buffer: Buffer.from('fake artifact'),
  })

  await expect(page.locator('.build-source-banner')).toContainText('ros2-build-35948f0.tar.gz')
  await page.locator('select').first().selectOption('payload_corner_navigation.yaml')
  await page.getByRole('button', { name: 'Save Fleet' }).click()
  await expect(page.locator('.build-source-banner')).toContainText('payload_corner_navigation.yaml')

  await page.getByRole('button', { name: 'Readiness' }).click()
  await expect(page.locator('.hardware-card').filter({ hasText: 'air-02.local' })).toBeVisible()
  await page.locator('.hardware-card').filter({ hasText: 'air-02.local' }).click()
  await expect(page.getByRole('heading', { name: 'Launch Prep' })).toBeVisible()
  await expect(page.locator('.assignment-card')).toContainText('payload_corner_navigation.yaml')
  await expect(page.locator('.assignment-select')).toBeVisible()
})

test('supports current workspace selection and preserves the air-02 control path', async ({ page }) => {
  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makePayloadCornerLocalArtifactSource(),
    boardGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
      targets: [],
      build_source: makePayloadCornerLocalCodebaseSource(),
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
    },
    buildSourcePostLocalCodebase: makePayloadCornerLocalCodebaseSource(),
  })

  await openSetupPage(page)
  await page.getByRole('button', { name: 'Use current workspace' }).click()
  await expect(page.locator('.build-source-banner')).toContainText('Current local codebase')
  await expect(page.locator('.build-source-banner')).toContainText('payload_corner_navigation.yaml')

  await page.getByRole('button', { name: 'Readiness' }).click()
  await expect(page.locator('.hardware-card').filter({ hasText: 'air-02.local' })).toBeVisible()
  await page.locator('.hardware-card').filter({ hasText: 'air-02.local' }).click()
  await expect(page.locator('.assignment-card')).toContainText('payload_corner_navigation.yaml')
  await page.locator('.assignment-select').selectOption('payload_0')
  await expect(page.locator('.detail-header')).toContainText('payload_0 · air-02.local')
  await expect(page.locator('.assignment-card')).toContainText('Selected controllable: payload_0')
  await expect(page.locator('.assignment-card').getByRole('button', { name: 'Deploy Fleet Context' })).toBeEnabled()
})

test('keeps a hostname-only live device visible on readiness when board refresh fails', async ({ page }) => {
  await installApiMocks(page, {
    config: makeConfigResponse(),
    boardGet: {
      success: false,
      error: 'board unavailable',
    },
    inventoryGet: {
      success: true,
      targets: [],
      active_target_id: '',
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: '',
          hostname: 'air-02.local',
          matched_label: 'Test hardware',
          readiness: {
            connected: false,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['SSH unavailable'],
          },
        }),
      ],
    },
    buildSourceGet: makeEmptyBuildSource(),
  })

  await openSetupPage(page)
  await page.getByRole('button', { name: 'Readiness' }).click()

  await expect(page.getByRole('heading', { name: 'Boarded devices' })).toBeVisible()
  await expect(page.locator('.hardware-card')).toContainText('air-02.local')
  await expect(page.locator('.hardware-card')).toContainText('Live device')
})

test('shows failed fleet deploy envelopes on the device page while still refreshing detail and board', async ({ page }) => {
  const state = await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makePayloadCornerBuildSource(),
    boardGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-01.local',
          hostname: 'air-01.local',
          vehicle_name: '',
          matched_label: 'air-01.local',
          readiness: {
            connected: true,
            build_installed: true,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
      targets: [],
      build_source: makePayloadCornerBuildSource(),
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-01.local',
          hostname: 'air-01.local',
          vehicle_name: '',
          matched_label: 'air-01.local',
          readiness: {
            connected: true,
            build_installed: true,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
    },
    buildCurrentGet: {
      success: true,
      installed: true,
      info: 'Installed build: release-previous',
    },
    connectionStatusGet: {
      success: true,
      connected: true,
      hostname: 'air-01.local',
    },
    sshCommandGet: {
      success: true,
      command: 'ssh penn@air-01.local',
    },
    wifiStatusGet: {
      success: true,
      connected: true,
      ssid: 'air-01.local',
    },
    fleetDeploy: {
      success: true,
      output: 'Deploy request accepted',
      summary: {
        total_devices: 1,
        failed_devices: 1,
        succeeded_devices: 0,
      },
      results: [
        {
          success: false,
          error: 'Deploy failed. Rolled back to release-previous.',
          attempted_release_id: 'release-new',
          active_release_id: 'release-previous',
          rolled_back: true,
        },
      ],
    },
    fleetStop: {
      success: true,
      output: 'Stop request accepted',
      summary: {
        total_devices: 1,
        failed_devices: 1,
        succeeded_devices: 0,
      },
      results: [
        {
          success: false,
          error: 'Stop failed while restarting the runtime.',
          attempted_release_id: 'release-new',
          active_release_id: 'release-previous',
          rolled_back: true,
        },
      ],
    },
  })

  const { assignmentCard } = await openLaunchMonitorForPayload(page)

  const boardBeforeDeploy = state.boardGetCount
  const connectionBeforeDeploy = state.connectionStatusGetCount
  await assignmentCard.getByRole('button', { name: 'Deploy Fleet Context' }).click()
  await expect(assignmentCard.locator('.result')).toContainText('Deploy failed. Rolled back to release-previous.')
  await expect.poll(() => state.boardGetCount).toBeGreaterThan(boardBeforeDeploy)
  await expect.poll(() => state.connectionStatusGetCount).toBeGreaterThan(connectionBeforeDeploy)

  await expect(assignmentCard).toContainText('payload_0')
})

test('keeps Mission Control locked until a controllable is assigned', async ({ page }) => {
  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makePayloadCornerBuildSource(),
    boardGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          vehicle_name: '',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
      targets: [],
      build_source: makePayloadCornerBuildSource(),
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          vehicle_name: '',
          readiness: {
            connected: true,
            build_installed: false,
            runtime_ready: false,
            vehicle_assigned: false,
            ready: false,
            notes: ['Choose a controllable before deploying.'],
          },
        }),
      ],
    },
    buildCurrentGet: {
      success: true,
      installed: false,
      info: 'No build deployed',
      release_id: null,
    },
    connectionStatusGet: {
      success: true,
      connected: true,
      hostname: 'air-02.local',
    },
    sshCommandGet: {
      success: true,
      command: 'ssh penn@air-02.local',
    },
    wifiStatusGet: {
      success: true,
      connected: true,
      ssid: 'air-02.local',
    },
  })

  await openSetupPage(page)
  await page.getByRole('button', { name: 'Readiness' }).click()
  await page.locator('.hardware-card').filter({ hasText: 'air-02.local' }).click()

  await expect(page.getByRole('button', { name: 'Mission Control' })).toBeDisabled()
  await expect(page.getByText('Mission Control unlocks after a controllable is selected for this hardware.')).toBeVisible()
})

test('renders camera calibration as a dropdown in fleet defaults and runtime overrides', async ({ page }) => {
  await installApiMocks(page, {
    config: makeConfigResponse(),
    initialBuildSource: makePayloadCornerBuildSource(),
    boardGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          vehicle_name: '',
          readiness: {
            connected: true,
            build_installed: true,
            runtime_ready: true,
            vehicle_assigned: true,
            ready: true,
            notes: [],
          },
        }),
      ],
      targets: [],
      build_source: makePayloadCornerBuildSource(),
    },
    liveGet: {
      success: true,
      devices: [
        makeLiveDevice({
          hardware_id: 'air-02.local',
          hostname: 'air-02.local',
          vehicle_name: '',
          readiness: {
            connected: true,
            build_installed: true,
            runtime_ready: true,
            vehicle_assigned: true,
            ready: true,
            notes: [],
          },
        }),
      ],
    },
    buildCurrentGet: {
      success: true,
      installed: true,
      info: 'Build Information\n=================\nSource: local package bundle\nBundle: Current local codebase',
    },
    connectionStatusGet: {
      success: true,
      connected: true,
      hostname: 'air-02.local',
    },
    sshCommandGet: {
      success: true,
      command: 'ssh penn@air-02.local',
    },
    missionStateGet: {
      success: true,
      state: {
        phase: 'idle',
        launch_state: 'not_prepared',
        running: false,
        pid: null,
        message: null,
        error: null,
        updated_at: '2026-04-16T09:00:00Z',
      },
    },
    launchParamsGet: {
      success: true,
      content: 'mission: payload_corner_navigation\ncamera_calibration_file: payload_0_camera_info.yaml\n',
    },
    missionFileGet: {
      success: true,
      content: 'target: payload\nmodes:\n  start:\n    mode: payload.PayloadCornerNavigationMode\n',
    },
  })

  await openSetupPage(page)
  await expect(page.getByLabel('camera_calibration_file')).toHaveValue('payload_0_camera_info.yaml')

  await page.getByRole('button', { name: 'Readiness' }).click()
  await page.locator('.hardware-card').filter({ hasText: 'air-02.local' }).click()
  await page.locator('.assignment-card').locator('select').selectOption('payload_0')
  await page.getByRole('button', { name: 'Mission Control' }).click()

  await expect(page.getByLabel('Camera calibration file')).toHaveValue('payload_0_camera_info.yaml')
})

for (const { actionKey, actionLabel, responseKey, errorText } of [
  {
    actionKey: 'prepare',
    actionLabel: 'PREPARE MISSION',
    responseKey: 'fleetPrepare',
    errorText: 'prepare failed despite the top-level success flag.',
  },
  {
    actionKey: 'start',
    actionLabel: 'START MISSION',
    responseKey: 'fleetStart',
    errorText: 'start failed despite the top-level success flag.',
  },
  {
    actionKey: 'stop',
    actionLabel: 'STOP MISSION',
    responseKey: 'fleetStop',
    errorText: 'stop failed despite the top-level success flag.',
  },
]) {
  test(`shows misleading ${actionKey} envelopes as failures in Mission Control`, async ({ page }) => {
    const scenario = {
    config: makeConfigResponse(),
      initialBuildSource: makePayloadCornerBuildSource(),
      boardGet: {
        success: true,
        devices: [
          makeLiveDevice({
            hardware_id: 'air-01.local',
            hostname: 'air-01.local',
            vehicle_name: '',
            matched_label: 'air-01.local',
            readiness: {
              connected: true,
              build_installed: true,
              runtime_ready: true,
              vehicle_assigned: true,
              ready: true,
              notes: [],
            },
          }),
        ],
        targets: [],
        build_source: makePayloadCornerBuildSource(),
      },
      liveGet: {
        success: true,
        devices: [
          makeLiveDevice({
            hardware_id: 'air-01.local',
            hostname: 'air-01.local',
            vehicle_name: 'payload_0',
            matched_label: 'air-01.local',
            readiness: {
              connected: true,
              build_installed: true,
              runtime_ready: true,
              vehicle_assigned: true,
              ready: true,
              notes: [],
            },
          }),
        ],
      },
      buildCurrentGet: {
        success: true,
        installed: true,
        info: 'Installed build: release-previous',
      },
      connectionStatusGet: {
        success: true,
        connected: true,
        hostname: 'air-01.local',
      },
      sshCommandGet: {
        success: true,
        command: 'ssh penn@air-01.local',
      },
      wifiStatusGet: {
        success: true,
        connected: true,
        ssid: 'air-01.local',
      },
      missionStateGet: {
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
      missionNamesGet: {
        success: true,
        missions: ['hover'],
      },
      schemaGet: {
        success: true,
        mode_registry: {},
        mission_document_schema: {},
        fleet_document_schema: {},
      },
      missionFileGet: {
        success: true,
        content: 'modes:\n  hover:\n    mode: uav.HoverMode\n',
      },
      launchLogsGet: {
        success: true,
        logs: '',
        next_offset: 0,
        inode: 0,
      },
      [responseKey]: {
        success: true,
        output: `${actionKey} request accepted`,
        summary: {
          total_devices: 1,
          failed_devices: 1,
          succeeded_devices: 0,
        },
        results: [
          {
            success: false,
            error: errorText,
            rolled_back: false,
          },
        ],
      },
    }

    await installApiMocks(page, scenario)
    const { missionActionsCard } = await openLaunchMonitorForPayload(page)
    await page.getByRole('button', { name: 'Mission Control' }).click()
    await expect(page.getByRole('heading', { name: 'Mission Actions' })).toBeVisible()
    await page.getByRole('button', { name: actionLabel }).click()
    if (actionKey === 'prepare') {
      await expect(page.getByText(errorText)).toBeVisible()
    } else if (actionKey === 'start') {
      await expect(page.locator('.result').last()).toContainText(errorText)
    } else {
      await expect(missionActionsCard.locator('.result')).toContainText(errorText)
    }
  })
}
