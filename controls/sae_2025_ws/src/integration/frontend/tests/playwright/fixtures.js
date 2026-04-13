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
