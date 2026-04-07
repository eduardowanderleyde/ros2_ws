import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'
const WS_URL = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/status`

// Fallback quando o mapa SLAM ainda não chegou
const WORLD_DEFAULT = { minX: -5, maxX: 5, minY: -5, maxY: 5 }

function worldToPixel(wx, wy, bounds, cw, ch) {
  const px = ((wx - bounds.minX) / (bounds.maxX - bounds.minX)) * cw
  const py = (1 - (wy - bounds.minY) / (bounds.maxY - bounds.minY)) * ch
  return { px, py }
}

function pixelToWorld(px, py, bounds, cw, ch) {
  const x = bounds.minX + (px / cw) * (bounds.maxX - bounds.minX)
  const y = bounds.maxY - (py / ch) * (bounds.maxY - bounds.minY)
  return { x, y }
}

export default function App() {
  const [status, setStatus] = useState({ robots: [], pose: { x: 0, y: 0, yaw: 0, valid: false } })
  const [wsLive, setWsLive] = useState(false)
  const [selectedRobot, setSelectedRobot] = useState('')
  const [routeName, setRouteName] = useState('r1')
  const [routeList, setRouteList] = useState([])
  const [target, setTarget] = useState(null)
  const [toast, setToast] = useState(null)
  const [mapData, setMapData] = useState(null)       // {resolution, origin_x, origin_y, width, height, png_b64}
  const mapImgRef = useRef(null)                      // HTMLImageElement do mapa SLAM
  const mapRef = useRef(null)
  const wsRef = useRef(null)
  const boundsRef = useRef(WORLD_DEFAULT)

  // Atualiza bounds quando mapa muda
  useEffect(() => {
    if (!mapData) { boundsRef.current = WORLD_DEFAULT; return }
    boundsRef.current = {
      minX: mapData.origin_x,
      maxX: mapData.origin_x + mapData.width * mapData.resolution,
      minY: mapData.origin_y,
      maxY: mapData.origin_y + mapData.height * mapData.resolution,
    }
  }, [mapData])

  const showToast = (msg, isError = false) => {
    setToast({ msg, error: isError })
    setTimeout(() => setToast(null), 3500)
  }

  const fetchJson = async (url, options = {}) => {
    const r = await fetch(url, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } })
    const data = await r.json().catch(() => ({}))
    return { ok: r.ok, data }
  }

  const call = async (path, method = 'POST') => {
    const { ok, data } = await fetchJson(`${API}${path}`, { method })
    return { ok, data }
  }

  const refetchStatus = useCallback(() => {
    fetch(`${API}/status`)
      .then((r) => r.json())
      .then((data) => data && setStatus(data))
      .catch(() => {})
  }, [])

  const afterAction = useCallback(() => {
    refetchStatus()
    const t1 = setTimeout(refetchStatus, 800)
    const t2 = setTimeout(refetchStatus, 1600)
    return () => { clearTimeout(t1); clearTimeout(t2) }
  }, [refetchStatus])

  const loadRoutes = useCallback(async () => {
    const { data } = await call(`/list_routes?robot_id=${encodeURIComponent(selectedRobot)}`, 'GET')
    setRouteList(data.route_names || [])
  }, [selectedRobot])

  // Busca mapa SLAM periodicamente
  useEffect(() => {
    const fetchMap = () => {
      fetch(`${API}/map`)
        .then((r) => r.json())
        .then((data) => {
          if (!data.available) return
          setMapData(data)
          const img = new Image()
          img.src = `data:image/png;base64,${data.png_b64}`
          img.onload = () => { mapImgRef.current = img }
        })
        .catch(() => {})
    }
    fetchMap()
    const t = setInterval(fetchMap, 3000)
    return () => clearInterval(t)
  }, [])

  useEffect(() => {
    fetch(`${API}/list_robots`)
      .then((r) => r.json())
      .then((data) => {
        const ids = data.robot_ids || []
        if (ids.length) setSelectedRobot((prev) => prev || ids[0] || '')
      })
      .catch(() => {})
  }, [])

  useEffect(() => { loadRoutes() }, [loadRoutes, selectedRobot])

  useEffect(() => {
    let closed = false
    const connect = () => {
      const ws = new WebSocket(WS_URL)
      ws.onopen = () => { setWsLive(true); wsRef.current = ws }
      ws.onclose = () => { setWsLive(false); wsRef.current = null; if (!closed) setTimeout(connect, 2000) }
      ws.onerror = () => {}
      ws.onmessage = (e) => {
        try { setStatus(JSON.parse(e.data)) } catch (_) {}
      }
    }
    connect()
    return () => { closed = true }
  }, [])

  useEffect(() => {
    if (wsLive) return
    const t = setInterval(refetchStatus, 1000)
    return () => clearInterval(t)
  }, [wsLive, refetchStatus])

  const handleMapClick = (e) => {
    const canvas = mapRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    const px = (e.clientX - rect.left) * scaleX
    const py = (e.clientY - rect.top) * scaleY
    const dpr = window.devicePixelRatio || 1
    const { x, y } = pixelToWorld(px / dpr, py / dpr, boundsRef.current, rect.width, rect.height)
    setTarget({ x, y })
  }

  const drawCanvas = useCallback(() => {
    const canvas = mapRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const dpr = window.devicePixelRatio || 1
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr)
      canvas.height = Math.round(rect.height * dpr)
    }
    const ctx = canvas.getContext('2d')
    ctx.setTransform(1, 0, 0, 1, 0, 0)
    ctx.scale(dpr, dpr)
    const w = rect.width
    const h = rect.height

    // Fundo
    ctx.fillStyle = '#0f1219'
    ctx.fillRect(0, 0, w, h)

    // Mapa SLAM como fundo
    if (mapImgRef.current) {
      ctx.globalAlpha = 0.85
      ctx.drawImage(mapImgRef.current, 0, 0, w, h)
      ctx.globalAlpha = 1.0
    } else {
      // Grid de fallback
      ctx.strokeStyle = '#1e2433'
      ctx.lineWidth = 1
      for (let i = 0; i <= w; i += 40) { ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, h); ctx.stroke() }
      for (let j = 0; j <= h; j += 40) { ctx.beginPath(); ctx.moveTo(0, j); ctx.lineTo(w, j); ctx.stroke() }
    }

    const bounds = boundsRef.current

    // Eixos
    const { px: cx, py: cy } = worldToPixel(0, 0, bounds, w, h)
    ctx.strokeStyle = 'rgba(255,255,255,0.2)'
    ctx.lineWidth = 1
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke()
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke()

    // Destino clicado
    if (target) {
      const { px, py } = worldToPixel(target.x, target.y, bounds, w, h)
      ctx.fillStyle = '#6ee7b7'
      ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2); ctx.fill()
      ctx.strokeStyle = '#0d0f14'; ctx.lineWidth = 2; ctx.stroke()
      // Cruz
      ctx.strokeStyle = '#6ee7b7'; ctx.lineWidth = 1.5
      ctx.beginPath(); ctx.moveTo(px - 12, py); ctx.lineTo(px + 12, py); ctx.stroke()
      ctx.beginPath(); ctx.moveTo(px, py - 12); ctx.lineTo(px, py + 12); ctx.stroke()
    }

    // Robot (pose via amcl_pose)
    const pose = status.pose
    if (pose && pose.valid) {
      const { px: rx, py: ry } = worldToPixel(pose.x, pose.y, bounds, w, h)
      const yaw = pose.yaw
      const R = 10

      ctx.save()
      ctx.translate(rx, ry)
      ctx.rotate(-yaw)  // Y flipado no canvas → negativo

      // Corpo
      ctx.beginPath(); ctx.arc(0, 0, R, 0, Math.PI * 2)
      ctx.fillStyle = '#3b82f6'
      ctx.fill()
      ctx.strokeStyle = '#93c5fd'; ctx.lineWidth = 2; ctx.stroke()

      // Seta de direcção
      ctx.beginPath()
      ctx.moveTo(0, 0)
      ctx.lineTo(R * 1.8, 0)
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5
      ctx.stroke()

      // Ponta da seta
      ctx.beginPath()
      ctx.moveTo(R * 1.8, 0)
      ctx.lineTo(R * 1.2, -4)
      ctx.lineTo(R * 1.2, 4)
      ctx.closePath()
      ctx.fillStyle = '#fff'; ctx.fill()

      ctx.restore()
    }
  }, [target, status])

  // Redesenha sempre que status ou target mudam
  useEffect(() => {
    drawCanvas()
    window.addEventListener('resize', drawCanvas)
    return () => window.removeEventListener('resize', drawCanvas)
  }, [drawCanvas])

  const q = (params) => new URLSearchParams(params).toString()

  const goToPoint = async () => {
    if (!isMobile) { showToast('Role não permite movimento (somente MUUT).', true); return }
    if (!target) return
    const { ok, data } = await call(`/go_to_point?${q({ robot_id: selectedRobot, x: target.x, y: target.y, yaw: 0 })}`, 'POST')
    showToast(ok ? 'Go to point enviado.' : (data?.message || 'Erro'), !ok)
    if (ok) afterAction()
  }

  const startRecord = async () => {
    const { ok, data } = await call(`/start_record?${q({ robot_id: selectedRobot, route_name: routeName })}`, 'POST')
    showToast(ok ? 'Record iniciado.' : (data?.message || 'Erro'), !ok)
    if (ok) { loadRoutes(); afterAction() }
  }

  const stopRecord = async () => {
    const { ok, data } = await call(`/stop_record?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Record parado. Rota salva.' : (data?.message || 'Erro'), !ok)
    if (ok) { loadRoutes(); afterAction() }
  }

  const playRoute = async () => {
    const { ok, data } = await call(`/play_route?${q({ robot_id: selectedRobot, route_name: routeName })}`, 'POST')
    showToast(ok ? 'Play route enviado.' : (data?.message || 'Erro'), !ok)
    if (ok) afterAction()
  }

  const cancel = async () => {
    const { ok, data } = await call(`/cancel?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Cancelado.' : (data?.message || 'Erro'), !ok)
    if (ok) afterAction()
  }

  const enableCollection = async () => {
    const { ok, data } = await call(`/enable_collection?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Coleta ativada.' : (data?.message || 'Erro'), !ok)
    if (ok) afterAction()
  }

  const disableCollection = async () => {
    const { ok, data } = await call(`/disable_collection?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Coleta desativada.' : (data?.message || 'Erro'), !ok)
    if (ok) afterAction()
  }

  const robots = status.robots.length
    ? status.robots
    : [{ robot_id: '', role: 'MUUT', nav_state: 'idle', current_route: '', collection_on: false, collection_file: '', last_error: '', bytes_written: 0 }]

  const selectedRobotState = robots.find((r) => r.robot_id === selectedRobot) || robots[0]
  const collectionOn = selectedRobotState.collection_on
  const selectedRole = selectedRobotState.role || 'MUUT'
  const isMobile = selectedRole === 'MUUT'
  const pose = status.pose || {}

  return (
    <div className="app">
      <header className="header">
        <h1>Fleet UI</h1>
        <span className={`status-badge ${wsLive ? 'live' : ''}`}>
          {wsLive ? 'Status ao vivo' : 'Polling'}
        </span>
      </header>

      <div className="grid">
        <div>
          <div className="panel">
            <h2>
              Mapa{mapData ? ` (SLAM ${mapData.width}×${mapData.height} @ ${mapData.resolution}m)` : ' (aguardando /map…)'}
            </h2>
            <div className="map-container">
              <canvas
                ref={mapRef}
                className="map-canvas"
                onClick={handleMapClick}
                style={{ width: '100%', height: '100%' }}
              />
              <div className="map-overlay">
                <span>
                  {target
                    ? `x: ${target.x.toFixed(2)} m  y: ${target.y.toFixed(2)} m`
                    : pose.valid
                      ? `robot: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) yaw: ${(pose.yaw * 180 / Math.PI).toFixed(1)}°`
                      : 'Clique no mapa para definir destino'}
                </span>
                <button className="btn btn-go" disabled={!target || !isMobile} onClick={goToPoint}>
                  Ir para ponto
                </button>
              </div>
            </div>
          </div>

          <div className="panel" style={{ marginTop: '1rem' }}>
            <h2>Controles ({selectedRole})</h2>
            <div className="controls">
              <label>
                Robô:
                <select value={selectedRobot} onChange={(e) => setSelectedRobot(e.target.value)}>
                  {robots.map((r) => (
                    <option key={r.robot_id || 'default'} value={r.robot_id}>{r.robot_id || '(default)'}</option>
                  ))}
                </select>
              </label>
              <label>
                Rota:
                <input
                  type="text"
                  value={routeName}
                  onChange={(e) => setRouteName(e.target.value)}
                  placeholder="r1"
                  list="routes"
                />
                <datalist id="routes">
                  {routeList.map((name) => <option key={name} value={name} />)}
                </datalist>
              </label>
            </div>
            <div className="controls">
              <button className="btn btn-primary" onClick={startRecord} disabled={!isMobile}>Iniciar gravação</button>
              <button className="btn" onClick={stopRecord} disabled={!isMobile}>Parar gravação</button>
              <button className="btn btn-primary" onClick={playRoute} disabled={!isMobile}>Reproduzir rota</button>
              <button className="btn btn-danger" onClick={cancel} disabled={!isMobile}>Cancelar</button>
              {collectionOn
                ? <button className="btn btn-danger" onClick={disableCollection}>Desligar coleta</button>
                : <button className="btn btn-primary" onClick={enableCollection}>Ligar coleta</button>}
            </div>
          </div>
        </div>

        <div className="panel">
          <h2>Status da frota</h2>
          <div className="robot-cards">
            {robots.map((r) => (
              <div key={r.robot_id} className="robot-card">
                <h3>{r.robot_id || '(default)'}</h3>
                <div className="row"><span>Papel</span><span>{r.role || 'MUUT'}</span></div>
                <div className="row">
                  <span>Nav</span>
                  <span className={`nav-state ${r.nav_state}`}>{r.nav_state}</span>
                </div>
                <div className="row"><span>Rota atual</span><span>{r.current_route || '—'}</span></div>
                <div className={`row ${r.collection_on ? 'collection-on' : ''}`}>
                  <span>Coleta</span><span>{r.collection_on ? 'ON' : 'OFF'}</span>
                </div>
                {r.collection_file && (
                  <div className="row">
                    <span>Arquivo</span>
                    <span style={{ fontSize: '0.7rem', wordBreak: 'break-all' }}>{r.collection_file}</span>
                  </div>
                )}
                {r.last_error && <div className="error-msg">{r.last_error}</div>}
              </div>
            ))}
          </div>

          {pose.valid && (
            <div className="robot-card" style={{ marginTop: '1rem' }}>
              <h3>Pose (AMCL)</h3>
              <div className="row"><span>x</span><span>{pose.x.toFixed(3)} m</span></div>
              <div className="row"><span>y</span><span>{pose.y.toFixed(3)} m</span></div>
              <div className="row"><span>yaw</span><span>{(pose.yaw * 180 / Math.PI).toFixed(1)}°</span></div>
            </div>
          )}
        </div>
      </div>

      {toast && (
        <div className={`toast ${toast.error ? 'error' : 'success'}`}>
          {toast.msg}
        </div>
      )}
    </div>
  )
}
