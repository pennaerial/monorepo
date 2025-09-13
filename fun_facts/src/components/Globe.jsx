import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
// Photo texture removed: we use a procedural ocean base and optional land-mask overlay

// pins: Array<{ lat: number, lng: number, label: string, members: string[] }>
// Optional props:
// - landMask: an imported image (PNG) with black land and transparent ocean.
export default function Globe({
  pins = [],
  height = 560,
  longitudeOffset = 0,
  autoRotate = false,
  landMask = null,
  landMaskProjection = "equirect", // "equirect" or "mercator"
  centerLat = 0,
  centerLon = 0,
  flipLongitude = false,
  landColor = "#169c19",
  oceanTop = "#083d77",
  oceanBottom = "#0b5fa5",
  showGraticules = true,
  debugLogOnClick = false,
}) {
  const containerRef = useRef(null);
  const tooltipRef = useRef(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const width = container.clientWidth;
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = null;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(0, 0, 3.5);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.minDistance = 2.2;
    controls.maxDistance = 6;
    controls.autoRotate = autoRotate;
    controls.autoRotateSpeed = 0.5;

    // Lights
    const ambient = new THREE.AmbientLight(0xffffff, 0.85);
    scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(5, 3, 5);
    scene.add(dir);

    // Earth with a procedural ocean base
    const earthRadius = 1;
    const geometry = new THREE.SphereGeometry(earthRadius, 64, 64);

    const makeProceduralEarthTexture = (w = 1024, h = 512) => {
      const canvas = document.createElement("canvas");
      canvas.width = w;
      canvas.height = h;
      const ctx = canvas.getContext("2d");
      // Ocean gradient
      const grad = ctx.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, oceanTop);
      grad.addColorStop(1, oceanBottom);
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, w, h);
      // Graticules
      if (showGraticules) {
        ctx.strokeStyle = "rgba(255,255,255,0.06)";
        ctx.lineWidth = 1;
        for (let lon = 0; lon <= w; lon += w / 12) {
          ctx.beginPath(); ctx.moveTo(lon + 0.5, 0); ctx.lineTo(lon + 0.5, h); ctx.stroke();
        }
        for (let lat = 0; lat <= h; lat += h / 12) {
          ctx.beginPath(); ctx.moveTo(0, lat + 0.5); ctx.lineTo(w, lat + 0.5); ctx.stroke();
        }
      }
      const tx = new THREE.CanvasTexture(canvas);
      tx.anisotropy = 8;
      tx.wrapS = THREE.RepeatWrapping;
      tx.wrapT = THREE.ClampToEdgeWrapping;
      tx.needsUpdate = true;
      return tx;
    };

    let activeTexture = makeProceduralEarthTexture();
    const material = new THREE.MeshPhongMaterial({ map: activeTexture, shininess: 12, specular: new THREE.Color(0x222222) });

    // Group globe elements so we can rotate them together (to hide seam, etc.)
    const globeGroup = new THREE.Group();
    scene.add(globeGroup);

    const earth = new THREE.Mesh(geometry, material);
    globeGroup.add(earth);

    // Texture loader (used for optional land mask)
    const loader = new THREE.TextureLoader();

    // Optional land-mask overlay (black countries with transparent background)
    let overlayGeometry = null;
    let overlayMaterial = null;
    let overlayTexture = null; // not used in alphaMap path
    let overlayAlphaTexture = null; // grayscale alpha map
    if (landMask) {
      loader.load(
        landMask,
        (tex) => {
          const img = tex.image;
          const w = img.width, h = img.height;
          try { console.info("[Globe] land-mask loaded", { w, h }); } catch {}

          // Draw source to canvas to access pixels
          const src = document.createElement("canvas");
          src.width = w; src.height = h;
          const sctx = src.getContext("2d");
          sctx.drawImage(img, 0, 0, w, h);

          // If mask is Web Mercator, reproject it to equirectangular (plate carrée)
          // so that latitude lines match the sphere's UVs.
          const sample = document.createElement("canvas");
          sample.width = w; sample.height = h;
          const pctx = sample.getContext("2d");
          if (landMaskProjection === "mercator") {
            // Reproject by scanline: for each output row (equirect lat), sample the corresponding Mercator Y.
            const latMax = Math.atan(Math.sinh(Math.PI)); // ~85.05113° in radians
            for (let y = 0; y < h; y++) {
              const v = (y + 0.5) / h; // 0 top -> 1 bottom
              let lat = (0.5 - v) * Math.PI; // equirect latitude in radians [-pi/2, pi/2]
              if (lat > latMax) lat = latMax; if (lat < -latMax) lat = -latMax;
              const mercV = 0.5 - (Math.log(Math.tan(Math.PI / 4 + lat / 2)) / (2 * Math.PI));
              let sy = mercV * h;
              if (sy < 0) sy = 0; if (sy > h - 1) sy = h - 1;
              // Copy a 1px-high row with filtering
              pctx.drawImage(src, 0, sy, w, 1, 0, y, w, 1);
            }
          } else {
            // Already equirectangular; just pass through
            pctx.drawImage(src, 0, 0, w, h);
          }

          // Read pixels from the (possibly reprojected) sample
          const imgData = pctx.getImageData(0, 0, w, h);
          const data = imgData.data;

          // Detect whether the PNG actually has a meaningful alpha channel.
          let minA = 1, maxA = 0;
          for (let i = 0; i < w * h; i++) {
            const a = data[i * 4 + 3] / 255;
            if (a < minA) minA = a;
            if (a > maxA) maxA = a;
          }
          const hasAlpha = (maxA - minA) > 0.05; // if true, trust alpha; else derive from luminance
          try { console.info("[Globe] land-mask hasAlpha:", hasAlpha, { minA, maxA }); } catch {}

          // Build an alpha image where land = opaque, ocean = transparent
          const alpha = document.createElement("canvas");
          alpha.width = w; alpha.height = h;
          const actx = alpha.getContext("2d");
          const aimg = actx.createImageData(w, h);
          for (let i = 0; i < w * h; i++) {
            const r = data[i * 4 + 0] / 255;
            const g = data[i * 4 + 1] / 255;
            const b = data[i * 4 + 2] / 255;
            const a = data[i * 4 + 3] / 255;
            const luma = 0.2126 * r + 0.7152 * g + 0.0722 * b; // 0=black, 1=white
            // If valid alpha exists, use it. Otherwise, black (low luma) -> opaque; white -> transparent.
            let out = hasAlpha ? a : (1 - luma);
            // Small cleanup thresholding
            if (out < 0.01) out = 0;
            if (out > 0.99) out = 1;
            const v = Math.round(out * 255);
            aimg.data[i * 4 + 0] = v; // grayscale
            aimg.data[i * 4 + 1] = v;
            aimg.data[i * 4 + 2] = v;
            aimg.data[i * 4 + 3] = 255; // opaque color channel
          }
          actx.putImageData(aimg, 0, 0);

          // Seam fix: duplicate edge columns on the alpha map
          const alphaSeam = document.createElement("canvas");
          alphaSeam.width = w + 2; alphaSeam.height = h;
          const a2 = alphaSeam.getContext("2d");
          a2.drawImage(alpha, 0, 0, w, h, 1, 0, w, h);
          a2.drawImage(alpha, w - 1, 0, 1, h, 0, 0, 1, h);
          a2.drawImage(alpha, 0, 0, 1, h, w + 1, 0, 1, h);

          overlayAlphaTexture = new THREE.CanvasTexture(alphaSeam);
          overlayAlphaTexture.anisotropy = 8;
          overlayAlphaTexture.wrapS = THREE.RepeatWrapping;
          overlayAlphaTexture.wrapT = THREE.ClampToEdgeWrapping;
          overlayAlphaTexture.repeat.set(w / (w + 2), 1);
          overlayAlphaTexture.offset.set(1 / (w + 2), 0);
          if ("NoColorSpace" in THREE) {
            overlayAlphaTexture.colorSpace = THREE.NoColorSpace; // treat as data
          }

          // Create overlay using alphaMap and a solid color (green)
          overlayGeometry = new THREE.SphereGeometry(earthRadius * 1.001, 64, 64);
          overlayMaterial = new THREE.MeshBasicMaterial({
            color: new THREE.Color(landColor),
            alphaMap: overlayAlphaTexture,
            transparent: true,
            depthWrite: false,
            alphaTest: 0.02,
          });
          const overlay = new THREE.Mesh(overlayGeometry, overlayMaterial);
          overlay.renderOrder = 2;
          globeGroup.add(overlay);
          try { console.info("[Globe] land-mask overlay added"); } catch {}
          tex.dispose();
        },
        undefined,
        () => {
          // If land mask fails to load, continue without overlay
        }
      );
    }

    // Atmosphere
    const atmosphereGeo = new THREE.SphereGeometry(earthRadius * 1.03, 64, 64);
    const atmosphereMat = new THREE.MeshBasicMaterial({ color: 0x60a5fa, transparent: true, opacity: 0.08 });
    const atmosphere = new THREE.Mesh(atmosphereGeo, atmosphereMat);
    atmosphere.renderOrder = 3;
    globeGroup.add(atmosphere);

    // Pins
    const pinMeshes = [];
    const pinGroup = new THREE.Group();
    globeGroup.add(pinGroup);
    const pinMaterial = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const pinGeom = new THREE.SphereGeometry(0.02, 12, 12);

    const toCartesian = (lat, lng, r) => {
      // Geographic to Cartesian for camera facing -Z: lon 0 sits at front (-Z)
      const latRad = THREE.MathUtils.degToRad(lat);
      const lonInput = flipLongitude ? -lng : lng;
      const lonRad = THREE.MathUtils.degToRad(lonInput);
      const cosLat = Math.cos(latRad);
      const x = r * cosLat * Math.sin(lonRad);
      const z = -r * cosLat * Math.cos(lonRad);
      const y = r * Math.sin(latRad);
      return new THREE.Vector3(x, y, z);
    };

    pins.forEach((p) => {
      const { lat, lng, label, members = [] } = p;
      if (typeof lat !== "number" || typeof lng !== "number") return;
      const m = new THREE.Mesh(pinGeom, pinMaterial.clone());
      m.position.copy(toCartesian(lat, lng, earthRadius + 0.02));
      m.lookAt(new THREE.Vector3(0, 0, 0));
      m.userData = { label, members };
      pinGroup.add(m);
      pinMeshes.push(m);
    });

    // Raycasting tooltips
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    const handleMove = (event) => {
      const rect = renderer.domElement.getBoundingClientRect();
      const x = (event.clientX - rect.left) / rect.width;
      const y = (event.clientY - rect.top) / rect.height;
      mouse.x = x * 2 - 1;
      mouse.y = -(y * 2 - 1);
      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObjects(pinMeshes, false);
      const tooltip = tooltipRef.current;
      if (!tooltip) return;
      if (intersects.length > 0) {
        const { label, members } = intersects[0].object.userData;
        tooltip.style.display = "block";
        tooltip.style.left = `${event.clientX + 12}px`;
        tooltip.style.top = `${event.clientY + 12}px`;
        const names = members && members.length ? members.join(", ") : "";
        tooltip.innerHTML = `<div class=\"text-sm font-semibold text-slate-800\">${label}</div>` +
          (names ? `<div class=\"text-xs text-slate-600 mt-1\">${names}</div>` : "");
      } else {
        tooltip.style.display = "none";
      }
    };
    renderer.domElement.addEventListener("mousemove", handleMove);
    renderer.domElement.addEventListener("mouseleave", () => {
      if (tooltipRef.current) tooltipRef.current.style.display = "none";
    });

    // Optional: click to log lat/lon under cursor on the globe
    const toLatLngLocal = (worldVec) => {
      // Convert a world-space point on the sphere to the globeGroup's local space
      const local = worldVec.clone();
      globeGroup.worldToLocal(local);
      const r = local.length();
      const lat = THREE.MathUtils.radToDeg(Math.asin(local.y / r));
      let lon = THREE.MathUtils.radToDeg(Math.atan2(local.x, -local.z));
      if (flipLongitude) lon = -lon;
      if (lon > 180) lon -= 360;
      if (lon <= -180) lon += 360;
      return { lat, lon };
    };
    const handleClick = (event) => {
      if (!debugLogOnClick) return;
      const rect = renderer.domElement.getBoundingClientRect();
      const x = (event.clientX - rect.left) / rect.width;
      const y = (event.clientY - rect.top) / rect.height;
      mouse.x = x * 2 - 1;
      mouse.y = -(y * 2 - 1);
      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObject(globeGroup, true);
      // Find the first hit on the earth mesh
      const hit = intersects.find((i) => i.object === earth || i.object.geometry === geometry);
      if (hit) {
        const { lat, lon } = toLatLngLocal(hit.point.clone());
        try { console.info("[Globe] click lat/lon:", lat, lon); } catch {}
      }
    };
    renderer.domElement.addEventListener("click", handleClick);

    // Resize
    const resize = () => {
      const w = container.clientWidth;
      camera.aspect = w / height;
      camera.updateProjectionMatrix();
      renderer.setSize(w, height);
    };
    const ro = new ResizeObserver(resize);
    ro.observe(container);

    // Center view precisely: rotate globe so (centerLat, centerLon) points to camera (-Z), and keep north up.
    // 1) Shortest-arc rotation from center vector to -Z
    const centerVec = toCartesian(centerLat, centerLon + longitudeOffset, 1).normalize();
    const targetVec = new THREE.Vector3(0, 0, -1);
    const qYawPitch = new THREE.Quaternion().setFromUnitVectors(centerVec, targetVec);
    // 2) Compute local north tangent at center and keep it pointing up (+Y) by adding a Z-roll
    const latRadC = THREE.MathUtils.degToRad(centerLat);
    const lonInputC = flipLongitude ? -(centerLon + longitudeOffset) : (centerLon + longitudeOffset);
    const lonRadC = THREE.MathUtils.degToRad(lonInputC);
    // Tangent toward geographic north at (lat, lon)
    const nTan = new THREE.Vector3(
      -Math.sin(latRadC) * Math.sin(lonRadC),
      Math.cos(latRadC),
      Math.sin(latRadC) * Math.cos(lonRadC)
    ).normalize();
    const nDir = nTan.clone().applyQuaternion(qYawPitch);
    const nProj = new THREE.Vector3(nDir.x, nDir.y, 0);
    let qRoll = new THREE.Quaternion();
    if (nProj.lengthSq() > 1e-6) {
      nProj.normalize();
      const angle = Math.atan2(nProj.x, nProj.y); // rotate so north aligns to +Y
      qRoll.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -angle);
    }
    const qFinal = qRoll.multiply(qYawPitch);
    globeGroup.quaternion.copy(qFinal);

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      ro.disconnect();
      renderer.domElement.removeEventListener("mousemove", handleMove);
      renderer.domElement.removeEventListener("click", handleClick);
      renderer.dispose();
      container.removeChild(renderer.domElement);
      pinMeshes.splice(0, pinMeshes.length);
      if (activeTexture) activeTexture.dispose();
      if (overlayTexture) overlayTexture.dispose();
      if (overlayAlphaTexture) overlayAlphaTexture.dispose();
      if (overlayMaterial) overlayMaterial.dispose();
      if (overlayGeometry) overlayGeometry.dispose();
      geometry.dispose();
    };
  }, [pins, height, longitudeOffset, autoRotate, landMask, oceanTop, oceanBottom, showGraticules, centerLat, centerLon, flipLongitude]);

  return (
    <div ref={containerRef} className="relative w-full" style={{ height }}>
      <div
        ref={tooltipRef}
        className="pointer-events-none absolute z-10 hidden rounded-md border border-slate-200 bg-white px-2 py-1 shadow-md"
        style={{ display: "none" }}
      />
    </div>
  );
}
