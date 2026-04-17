export function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

export function jsonResponse(body, status = 200) {
  return {
    status,
    contentType: 'application/json',
    body: JSON.stringify(body),
  }
}

export function makeConfigResponse(overrides = {}) {
  return {
    success: true,
    config: {
      default_pi_user: 'penn',
      default_ssh_key: '~/.ssh/pennair_pi_ed25519',
      default_deploy_root: '/home/penn/pennair-deploy',
      inventory_path: '/tmp/integration-inventory.json',
      ...overrides,
    },
  }
}

export function makeActionsBuild(overrides = {}) {
  return {
    source: 'actions',
    tag: 'manual-merge',
    sha: 'cafebabedeadbeef',
    commitSha: 'cafebabedeadbeef',
    commitSubject: 'Manual merge build for flight stack',
    name: 'manual-merge-artifact',
    artifactName: 'flight-stack.tar.gz',
    artifact_name: 'flight-stack.tar.gz',
    artifact_id: 'art-24327571117',
    run_id: '24327571117',
    run_number: '112',
    branch: 'main',
    workflow_name: '',
    job_name: '',
    event: 'push',
    conclusion: 'success',
    date: '2026-04-13T12:34:56Z',
    download_url: 'https://example.invalid/flight-stack.tar.gz',
    size_mb: 42.5,
    ...overrides,
  }
}

export function makeReleaseBuild(overrides = {}) {
  return {
    source: 'release',
    tag: 'v1.2.3',
    sha: 'facefeed12345678',
    commitSha: 'facefeed12345678',
    commitSubject: 'ARM Artifact Release',
    name: 'arm-artifact-release',
    artifactName: 'arm-release.tar.gz',
    artifact_name: 'arm-release.tar.gz',
    artifact_id: 'release-art-1',
    run_id: '',
    run_number: '',
    branch: 'main',
    workflow_name: 'ARM Artifact Release',
    job_name: 'release-packaging',
    event: 'release',
    conclusion: 'success',
    date: '2026-04-12T10:00:00Z',
    download_url: 'https://example.invalid/arm-release.tar.gz',
    size_mb: 9.75,
    ...overrides,
  }
}

export function makeBuildListResponse({
  releases = [makeReleaseBuild()],
  artifacts = [makeActionsBuild()],
  builds = null,
  hasMore = false,
} = {}) {
  const response = {
    success: true,
    releases,
    artifacts,
    artifacts_page_info: {
      has_more: hasMore,
      page: 1,
      limit: 20,
    },
  }
  if (Array.isArray(builds)) {
    response.builds = builds
  }
  return response
}

export function makeFleetOptions(options = []) {
  return options
    .map(option => {
      if (!option) return null
      if (typeof option === 'string') {
        return {
          value: option,
          path: option,
          label: option,
          description: '',
          disabled: false,
        }
      }
      return {
        value: option.value || option.path || option.label || '',
        path: option.path || option.value || option.label || '',
        label: option.label || option.value || option.path || '',
        description: option.description || '',
        disabled: Boolean(option.disabled),
      }
    })
    .filter(Boolean)
}

export function makeEmptyBuildSource(overrides = {}) {
  return {
    success: true,
    kind: 'none',
    fleet_options: [],
    ...overrides,
  }
}

export function makeGithubBuildSource(build, overrides = {}) {
  return {
    success: true,
    kind: 'github',
    build,
    source: build?.source || 'release',
    fleet_file: '',
    fleet_options: [],
    ...overrides,
  }
}

export function makeLocalArtifactBuildSource(overrides = {}) {
  return {
    success: true,
    kind: 'local_artifact',
    source_kind: 'local_artifact',
    file_name: 'ros2-build-35948f0.tar.gz',
    source_label: 'ros2-build-35948f0.tar.gz',
    fleet_file: '',
    fleet_options: [],
    ...overrides,
  }
}

export function makeLocalCodebaseBuildSource(overrides = {}) {
  return {
    success: true,
    kind: 'local_codebase',
    source_kind: 'local_codebase',
    source_label: 'Current local codebase',
    codebase_root: '/workspace/sae_2025_ws',
    packages: [],
    fleet_file: '',
    fleet_options: [],
    ...overrides,
  }
}

export function makeLiveDevice(overrides = {}) {
  return {
    hardware_id: 'air-02.local',
    hostname: 'air-02.local',
    saved: false,
    matched_label: 'air-02.local',
    matched_target_id: '',
    addresses: ['air-02.local'],
    readiness: {
      connected: false,
      build_installed: false,
      runtime_ready: false,
      vehicle_assigned: false,
      ready: false,
      notes: ['SSH unavailable'],
    },
    ...overrides,
  }
}

export function makeFleetSchemaResponse(overrides = {}) {
  return {
    available_fleets: ['payload_corner_navigation'],
    backend_kinds: ['sim', 'hardware', 'real'],
    excluded_keys: [],
    document_schema: {},
    sections: [
      {
        name: 'backend',
        description: 'Backend configuration shared by the fleet.',
        applies_to: ['sim', 'hardware', 'real'],
        constraints: ['kind must be sim, hardware, or real.'],
        fields: [
          {
            name: 'kind',
            schema_type: 'enum',
            annotation: 'enum',
            required: true,
            default: 'hardware',
            default_kind: 'value',
            choices: ['sim', 'hardware', 'real'],
          },
          {
            name: 'px4_path',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default: '~/PX4-Autopilot',
            default_kind: 'value',
            choices: [],
          },
          {
            name: 'world_name',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
          {
            name: 'mission_stage',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
        ],
      },
      {
        name: 'defaults',
        description: 'Shared per-vehicle runtime defaults.',
        applies_to: ['sim', 'hardware', 'real'],
        constraints: [],
        fields: [
          {
            name: 'debug',
            schema_type: 'bool',
            annotation: 'bool',
            required: false,
            default: false,
            default_kind: 'value',
            choices: [],
          },
          {
            name: 'save_vision_milliseconds',
            schema_type: 'int',
            annotation: 'int',
            required: false,
            default: 0,
            default_kind: 'value',
            choices: [],
          },
          {
            name: 'camera_calibration_file',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default: 'payload_0_camera_info.yaml',
            default_kind: 'value',
            choices: ['payload_0_camera_info.yaml'],
          },
        ],
      },
      {
        name: 'vehicle.common',
        description: 'Per-vehicle common fields.',
        applies_to: ['sim', 'hardware', 'real'],
        constraints: [],
        fields: [
          {
            name: 'name',
            schema_type: 'str',
            annotation: 'str',
            required: true,
            default_kind: 'missing',
            choices: [],
          },
          {
            name: 'mission',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
          {
            name: 'kind',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
        ],
      },
      {
        name: 'vehicle.hardware',
        description: 'Hardware-only vehicle fields.',
        applies_to: ['hardware', 'real'],
        constraints: [],
        fields: [
          {
            name: 'px4_airframe_id',
            schema_type: 'int',
            annotation: 'int',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
          {
            name: 'px4_namespace',
            schema_type: 'str',
            annotation: 'str',
            required: false,
            default_kind: 'missing',
            choices: [],
          },
        ],
      },
    ],
    ...overrides,
  }
}

export function makeMissionCatalogResponse(overrides = {}) {
  return {
    success: true,
    available_missions: [
      'hover',
      'payload_corner_navigation',
      'payload_retreat',
    ],
    missions: [
      { name: 'hover', target: 'uav' },
      { name: 'payload_corner_navigation', target: 'payload' },
      { name: 'payload_retreat', target: 'payload' },
    ],
    ...overrides,
  }
}

export function makeFleetFileResponse(overrides = {}) {
  return {
    success: true,
    fleet_file: 'payload_corner_navigation.yaml',
    path: '/tmp/payload_corner_navigation.yaml',
    content: [
      'backend:',
      '  kind: sim',
      '  world_name: sae',
      '  mission_stage: payload_retreat',
      'defaults:',
      '  auto_launch: true',
      '  debug: false',
      'vehicles:',
      '  - name: payload_0',
      '    kind: payload',
      '    mission: payload_corner_navigation',
      '    controllable: payload_0',
      '    payload_controller: GPIOController',
      '',
    ].join('\n'),
    ...overrides,
  }
}
