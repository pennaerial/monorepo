// Leaflet bridge for the GCS field map. This file owns the map; Rust drives it through
// `window.fieldMap.*` (see src/components/map/bridge.rs) and never touches Leaflet itself.
//
// Everything arrives as field coordinates in metres: x along the field, y across it,
// (0, 0) at the field's origin corner. The map uses L.CRS.Simple (flat, 1 unit = 1 m,
// no tiles / blank background), which takes [lat, lng] = [y, x]. That flip lives
// ONLY in `ll()` below.
window.fieldMap = window.fieldMap || (() => {
  const MINE_COLORS = { detected: "#f59e0b", confirmed: "#ef4444", cleared: "#22c55e" };
  // Grid cell sizes (m); the smallest one at least MIN_CELL_PX on screen is used.
  const NICE_SPACINGS_M = [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
  const MIN_CELL_PX = 40;
  const FIT = { padding: [24, 24] };

  let map = null;
  let field = null; // { width, height } in metres
  let grid, gridLabel, route, waypoints, person, target, resizeObserver;
  const mines = new Map();
  const drones = new Map();

  const ll = (p) => [p.y, p.x];

  function init(id, size) {
    if (map) destroy();
    field = size;
    map = L.map(id, {
      crs: L.CRS.Simple,
      minZoom: -3,
      maxZoom: 7, // 128 px per metre
      zoomSnap: 0,
      zoomDelta: 0.5,
      wheelPxPerZoomLevel: 120,
      attributionControl: false,
    });
    map.getContainer().style.background = "#f4f4f5";

    // Panes fix draw order regardless of when layers are (re)added.
    [["grid", 350], ["route", 410], ["markers", 420], ["person", 430]].forEach(([name, z]) => {
      map.createPane(name).style.zIndex = z;
    });

    grid = L.polyline([], { pane: "grid", color: "rgb(113 113 122)", opacity: 0.35, weight: 1, interactive: false }).addTo(map);
    L.rectangle([[0, 0], [field.height, field.width]], {
      pane: "grid", color: "#a16207", weight: 2, fillColor: "rgb(250 204 21)", fillOpacity: 0.08, interactive: false,
    }).addTo(map);

    const Label = L.Control.extend({
      onAdd() {
        gridLabel = L.DomUtil.create("div");
        gridLabel.style.cssText = "padding: 2px 8px; border-radius: 4px; background: rgb(255 255 255 / 0.8); color: #3f3f46; font: 12px sans-serif;";
        return gridLabel;
      },
    });
    new Label({ position: "bottomleft" }).addTo(map);

    route = L.polyline([], { pane: "route", color: "#0284c7", weight: 3, dashArray: "8 6", lineJoin: "round", lineCap: "round" }).addTo(map);
    waypoints = L.layerGroup().addTo(map);
    target = L.layerGroup([
      L.circleMarker([0, 0], { pane: "markers", radius: 12, color: "#7c3aed", weight: 3, fill: false }),
      L.circleMarker([0, 0], { pane: "markers", radius: 4, stroke: false, fillColor: "#7c3aed", fillOpacity: 1 }),
    ]);
    person = L.layerGroup([
      L.circleMarker([0, 0], { pane: "person", radius: 16, stroke: false, fillColor: "#2563eb", fillOpacity: 0.2 }),
      L.circleMarker([0, 0], { pane: "person", radius: 7, color: "white", weight: 3, fillColor: "#2563eb", fillOpacity: 1 }),
    ]);

    map.on("zoomend moveend resize", drawGrid);
    // Fit the field as soon as the container has a real size, and keep Leaflet in
    // sync when the layout changes (sidebar, window resize).
    let fitted = false;
    resizeObserver = new ResizeObserver(() => {
      map.invalidateSize();
      const { x, y } = map.getSize();
      if (!fitted && x > 0 && y > 0) {
        fitted = true;
        api.fitField();
      }
    });
    resizeObserver.observe(map.getContainer());
    map.setView([field.height / 2, field.width / 2], 0); // placeholder until the first fit
  }

  function destroy() {
    resizeObserver.disconnect();
    map.remove();
    map = null;
    mines.clear();
    drones.clear();
  }

  function drawGrid() {
    const metresPerPx = Math.pow(2, -map.getZoom()); // CRS.Simple: zoom 0 = 1 px per metre
    const step = NICE_SPACINGS_M.find((s) => s / metresPerPx >= MIN_CELL_PX) ?? NICE_SPACINGS_M.at(-1);
    const b = map.getBounds();
    const [s, n, w, e] = [b.getSouth(), b.getNorth(), b.getWest(), b.getEast()];
    const lines = [];
    // Integer indices (not `x += step`) so lines stay exactly on multiples of step,
    // anchored on the field origin.
    for (let i = Math.ceil(w / step); i * step <= e; i++) lines.push([[s, i * step], [n, i * step]]);
    for (let i = Math.ceil(s / step); i * step <= n; i++) lines.push([[i * step, w], [i * step, e]]);
    grid.setLatLngs(lines);
    gridLabel.textContent = step < 1 ? `grid ${step * 100} cm` : `grid ${step} m`;
  }

  // Update circle markers in place by id; add new ones, remove ones no longer listed.
  function sync(store, items, style) {
    const seen = new Set();
    for (const item of items) {
      seen.add(item.id);
      const marker = store.get(item.id);
      if (marker) {
        marker.setLatLng(ll(item)).setStyle(style(item));
      } else {
        store.set(item.id, L.circleMarker(ll(item), { pane: "markers", ...style(item) }).addTo(map));
      }
    }
    for (const [id, marker] of store) {
      if (!seen.has(id)) {
        marker.remove();
        store.delete(id);
      }
    }
  }

  // Shows a layer group of concentric circle markers at `point`, or hides it for null.
  function placeGroup(group, point) {
    if (!point) return group.remove();
    group.eachLayer((layer) => layer.setLatLng(ll(point)));
    group.addTo(map);
  }

  const api = {
    init,
    // [{ id, x, y, status: "detected" | "confirmed" | "cleared" }]
    setMines(items) {
      sync(mines, items, (m) => ({ radius: 7, color: "white", weight: 2, fillColor: MINE_COLORS[m.status], fillOpacity: 1 }));
    },
    // [{ id, x, y }]
    setDrones(items) {
      sync(drones, items, () => ({ radius: 8, color: "#38bdf8", weight: 3, fillColor: "#0f172a", fillOpacity: 1 }));
    },
    // [{ x, y }, ...] from the person to the target
    setPath(points) {
      route.setLatLngs(points.map(ll));
      waypoints.clearLayers();
      points.slice(1, -1).forEach((p) => {
        L.circleMarker(ll(p), { pane: "route", radius: 3, stroke: false, fillColor: "#0284c7", fillOpacity: 1 }).addTo(waypoints);
      });
    },
    setPerson: (point) => placeGroup(person, point),
    setTarget: (point) => placeGroup(target, point),
    fitField() {
      map.fitBounds([[0, 0], [field.height, field.width]], FIT);
    },
    fitAll() {
      const points = [...mines.values(), ...drones.values()].map((m) => m.getLatLng()).concat(route.getLatLngs());
      [person, target].forEach((g) => map.hasLayer(g) && points.push(g.getLayers()[0].getLatLng()));
      if (points.length) map.fitBounds(L.latLngBounds(points), { ...FIT, maxZoom: 5 });
    },
    panTo: (point) => map.panTo(ll(point)),
  };
  // Everything but init is a no-op until the map exists, so an early call can't throw.
  for (const name of Object.keys(api)) {
    if (name === "init") continue;
    const fn = api[name];
    api[name] = (...args) => map && fn(...args);
  }
  return api;
})();
