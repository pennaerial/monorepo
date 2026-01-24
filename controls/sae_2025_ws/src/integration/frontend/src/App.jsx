import { useState, useEffect } from 'react'
import './App.css'

function App() {
  const [commits, setCommits] = useState([])
  const [selectedCommit, setSelectedCommit] = useState('')
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
          setSelectedCommit(data.commits[0])
        }
      })
      .catch(err => console.error('Failed to load commits:', err))
  }, [])

  const handleSubmit = async (e) => {
    e.preventDefault()
    setLoading(true)

    // Convert params object to command line arguments
    const paramsString = `mission_name:=${params.mission_name} uav_debug:=${params.uav_debug} vision_debug:=${params.vision_debug} run_mission:=${params.run_mission} vehicle_type:=${params.vehicle_type} save_vision:=${params.save_vision} servo_only:=${params.servo_only} camera_offsets:=[${params.camera_offsets}] sim:=${params.sim}`

    const formData = new FormData()
    formData.append('commit', selectedCommit)
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
    <div className="container">
      <form onSubmit={handleSubmit}>
        <label htmlFor="commit">Select Commit Hash:</label>
        <select
          id="commit"
          value={selectedCommit}
          onChange={(e) => setSelectedCommit(e.target.value)}
          required
        >
          {commits.length === 0 ? (
            <option value="">Loading commits...</option>
          ) : (
            commits.map(commit => (
              <option key={commit} value={commit}>{commit}</option>
            ))
          )}
        </select>

        <div className="params-section">
          <h3>Launch Parameters:</h3>

          <label htmlFor="mission_name">Mission Name:</label>
          <input
            type="text"
            id="mission_name"
            value={params.mission_name}
            onChange={(e) => handleParamChange('mission_name', e.target.value)}
          />

          <label htmlFor="vehicle_type">Vehicle Type:</label>
          <select
            id="vehicle_type"
            value={params.vehicle_type}
            onChange={(e) => handleParamChange('vehicle_type', parseInt(e.target.value))}
          >
            <option value={0}>0</option>
            <option value={1}>1</option>
            <option value={2}>2</option>
            <option value={3}>3</option>
          </select>

          <label htmlFor="camera_offsets">Camera Offsets (x, y, z in meters):</label>
          <input
            type="text"
            id="camera_offsets"
            value={params.camera_offsets}
            onChange={(e) => handleParamChange('camera_offsets', e.target.value)}
            placeholder="0, 0, 0"
          />

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

        <button type="submit" className="launch-btn" disabled={loading}>
          {loading ? 'LAUNCHING...' : 'LAUNCH'}
        </button>
      </form>

      {(loading || result) && (
        <div className={`result ${loading ? '' : result.success ? 'success' : 'error'}`}>
          {loading ? (
            'Launching...'
          ) : result.success ? (
            `✓ Success!\n\n${result.output}`
          ) : (
            `✗ Error\n\n${result.error}`
          )}
        </div>
      )}
    </div>
  )
}

export default App
