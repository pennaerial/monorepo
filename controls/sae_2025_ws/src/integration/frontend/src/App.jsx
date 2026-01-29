import { useState, useEffect } from 'react'
import './App.css'

function App() {
  const [commits, setCommits] = useState([])
  const [selectedCommit, setSelectedCommit] = useState('')
  const [inputValue, setInputValue] = useState('')
  const [showDropdown, setShowDropdown] = useState(false)
  const [target, setTarget] = useState('')
  const [password, setPassword] = useState('')
  const [params, setParams] = useState({
    mission_name: 'test_straight_course',
    uav_debug: false,
    vision_debug: false,
    run_mission: true,
    vehicle_type: 0,
    save_vision: false,
    servo_only: false,
    camera_offsets: '0, 0, 0',
    sim: true
  })
  const [result, setResult] = useState(null)
  const [loading, setLoading] = useState(false)

  const handleParamChange = (key, value) => {
    setParams(prev => ({ ...prev, [key]: value }))
  }

  useEffect(() => {
    fetch('/api/commits')
      .then(res => res.json())
      .then(data => {
        setCommits(data.commits)
        if (data.commits.length > 0) {
          const firstCommit = data.commits[0].hash
          setSelectedCommit(firstCommit)
          setInputValue(firstCommit)
        }
      })
      .catch(err => console.error('Failed to load commits:', err))
  }, [])

  // Filter commits based on input
  const filteredCommits = commits.filter(commit => 
    commit.hash.toLowerCase().includes(inputValue.toLowerCase()) ||
    commit.branch.toLowerCase().includes(inputValue.toLowerCase())
  )

  // Group by branch
  const commitsByBranch = filteredCommits.reduce((acc, commit) => {
    if (!acc[commit.branch]) acc[commit.branch] = []
    acc[commit.branch].push(commit)
    return acc
  }, {})

  // Handle selecting a commit from dropdown
  const selectCommit = (commit) => {
    setSelectedCommit(commit.hash)
    setInputValue(commit.hash)
    setShowDropdown(false)
  }

  // Handle input change
  const handleInputChange = (e) => {
    const value = e.target.value
    setInputValue(value)
    setShowDropdown(true)
    
    // If user types an exact match, auto-select it
    const exactMatch = commits.find(c => c.hash === value)
    if (exactMatch) {
      setSelectedCommit(value)
    }
  }

  const handleSubmit = async (e) => {
    e.preventDefault()
    setLoading(true)

    // Convert params object to command line arguments
    const paramsString = `mission_name:=${params.mission_name} uav_debug:=${params.uav_debug} vision_debug:=${params.vision_debug} run_mission:=${params.run_mission} vehicle_type:=${params.vehicle_type} save_vision:=${params.save_vision} servo_only:=${params.servo_only} camera_offsets:=[${params.camera_offsets}] sim:=${params.sim}`

    const formData = new FormData()
    formData.append('commit', selectedCommit)
    formData.append('target', target)
    formData.append('password', password)
    formData.append('params', paramsString)

    try {
      const response = await fetch('/api/launch', {
        method: 'POST',
        body: formData
      })
      const data = await response.json()
      setResult(data)
    } catch (error) {
      setResult({ success: false, error: error.message })
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="app">
      <div className="container">
        <form onSubmit={handleSubmit}>
          <div className="section">
            <label className="section-label">Select Commit</label>
            <div className="autocomplete-wrapper">
              <div className="autocomplete-input-wrapper">
                <input
                  type="text"
                  placeholder="Search commits or branches..."
                  value={inputValue}
                  onChange={handleInputChange}
                  onFocus={() => setShowDropdown(true)}
                  onBlur={() => setTimeout(() => setShowDropdown(false), 200)}
                  className="autocomplete-input"
                  required
                />
                {inputValue && (
                  <button 
                    type="button" 
                    className="clear-btn"
                    onClick={() => {
                      setInputValue('')
                      setSelectedCommit('')
                      setShowDropdown(true)
                    }}
                  >
                    ×
                  </button>
                )}
              </div>

              {showDropdown && (
                <div className="autocomplete-dropdown">
                  {commits.length === 0 ? (
                    <div className="dropdown-empty">Loading commits...</div>
                  ) : filteredCommits.length === 0 ? (
                    <div className="dropdown-empty">No matches found</div>
                  ) : (
                    <>
                      {Object.entries(commitsByBranch).map(([branch, branchCommits]) => (
                        <div key={branch} className="dropdown-group">
                          <div className="dropdown-branch">{branch}</div>
                          {branchCommits.map(commit => (
                            <div
                              key={commit.hash}
                              className={`dropdown-item ${selectedCommit === commit.hash ? 'selected' : ''}`}
                              onClick={() => selectCommit(commit)}
                            >
                              <span className="commit-hash">{commit.hash}</span>
                            </div>
                          ))}
                        </div>
                      ))}
                    </>
                  )}
                </div>
              )}
            </div>
          </div>

          <div className="section">
            <label className="section-label">Deploy Target</label>
            <input
              type="text"
              placeholder="e.g., pi@192.168.1.50"
              value={target}
              onChange={(e) => setTarget(e.target.value)}
              className="target-input"
            />
          </div>

          <div className="section">
            <label className="section-label">SSH Password</label>
            <input
              type="password"
              placeholder="Enter SSH password for target"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="target-input"
            />
          </div>

          <div className="section">
            <label className="section-label">Launch Parameters</label>
            <div className="params-grid">
              <div className="param-field">
                      <label htmlFor="mission_name">Mission Name</label>
                <input
                  type="text"
                  id="mission_name"
                  value={params.mission_name}
                  onChange={(e) => handleParamChange('mission_name', e.target.value)}
                />
              </div>

              <div className="param-field">
                <label htmlFor="vehicle_type">Vehicle Type</label>
                <select
                  id="vehicle_type"
                  value={params.vehicle_type}
                  onChange={(e) => handleParamChange('vehicle_type', parseInt(e.target.value))}
                >
                  <option value={0}>Type 0</option>
                  <option value={1}>Type 1</option>
                  <option value={2}>Type 2</option>
                  <option value={3}>Type 3</option>
                </select>
              </div>

              <div className="param-field full-width">
                <label htmlFor="camera_offsets">Camera Offsets (x, y, z meters)</label>
                <input
                  type="text"
                  id="camera_offsets"
                  value={params.camera_offsets}
                  onChange={(e) => handleParamChange('camera_offsets', e.target.value)}
                  placeholder="0, 0, 0"
                />
              </div>

              <div className="checkbox-group">
            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.uav_debug}
                onChange={(e) => handleParamChange('uav_debug', e.target.checked)}
              />
              UAV Debug
            </label>

            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.vision_debug}
                onChange={(e) => handleParamChange('vision_debug', e.target.checked)}
              />
              Vision Debug
            </label>

            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.run_mission}
                onChange={(e) => handleParamChange('run_mission', e.target.checked)}
              />
              Run Mission
            </label>

            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.save_vision}
                onChange={(e) => handleParamChange('save_vision', e.target.checked)}
              />
              Save Vision
            </label>

            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.servo_only}
                onChange={(e) => handleParamChange('servo_only', e.target.checked)}
              />
              Servo Only
            </label>

            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={params.sim}
                onChange={(e) => handleParamChange('sim', e.target.checked)}
              />
              Simulation Mode
            </label>
              </div>
            </div>
          </div>

          <button type="submit" className="launch-btn" disabled={loading}>
            {loading ? 'Launching...' : 'Launch'}
          </button>
        </form>

        {(loading || result) && (
          <div className={`result ${loading ? 'loading' : result.success ? 'success' : 'error'}`}>
            <div className="result-header">
              {loading ? 'Launching...' : result.success ? 'Success' : 'Error'}
            </div>
            {!loading && (
              <div className="result-content">
                {result.success ? result.output : result.error}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  )
}

export default App
